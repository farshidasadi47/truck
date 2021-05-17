#include <ros.h>                                   // ros main library, use before any other ros library.
#include <std_msgs/Float32.h>                      // ros message type,
#include <sensor_msgs/Imu.h>                       // ros message for imu.
#include <ackermann_msgs/AckermannDriveStamped.h>  // ros message for ackerman steering.
#include <std_srvs/SetBool.h>                      // ros services.
#include <std_srvs/Empty.h>                        // ros services.
#include <Encoder.h>                               // arduino encoder library
#include <Servo.h>                                 // arduino servo library.
#include <Wire.h>                                  // arduino I2C library
#include <Adafruit_Sensor.h>                       // adafruint sensor library, read and convert sensor data to standard description.
#include <Adafruit_BNO055.h>                       // adafruit imu library.
#include <utility/imumaths.h>                      // adafruit labrary for imu calculations.
#include "TeensyThreads.h"                         // teensy library for multithreading.
#define USE_TEENSY_HW_SERIAL                       // define this to use teensy hardware serial.

/****PIN CONNECTION MACROS****/
#define _SERVO 2
#define _ESC 3
#define _FL_A 5
#define _FL_B 6
#define _FR_A 7
#define _FR_B 8
#define _RL_A 9
#define _RL_B 10
#define _RR_A 11
#define _RR_B 12
#define _REC_SERVO 14
#define _REC_ESC 15
#define _STEER 23
/*******Reciever PWM range macros*********/
#define _REC_ESC_MIN 1065
#define _REC_ESC_MAX 2006
#define _REC_SERVO_MIN 1065
#define _REC_SERVO_MAX 2006
#define _ESC_NEUTRAL 1512
#define _SERVO_NEUTRAL 1523


 //TODO add service for switching to computer control


/************STRUCT DEFINTIONS************/
struct ros_message{ //contains all ROS messages
  std_msgs::Float32 flFeedback;
  std_msgs::Float32 frFeedback;
  std_msgs::Float32 rlFeedback;
  std_msgs::Float32 rrFeedback;
  std_msgs::Float32 steeringFeedback;
  std_msgs::Float32 manualSteering;
  std_msgs::Float32 manualThrottle;
  sensor_msgs::Imu imu_dat;
};

struct isr_variables{                        // volatile variables changed by ISR's
  volatile unsigned long steering_pwm;       // current pwm width in microseconds for steering, 0.351564 degrees per microsecond
  volatile unsigned long reciever_esc_pwm;   // current pwm width in microseconds for reciever esc, 1512 neurtral
  volatile unsigned long reciever_servo_pwm; // current pwm width in microseconds for reciever esc, 1523 neurtral
  volatile unsigned long esc_micros;
  volatile unsigned long servo_micros;
  volatile unsigned long steering_micros;
  volatile float steering_angle;
  };
  
struct steering_calibration_parameters{ //PWM parameters for absolute steering sensor, calculated on startup or on service call
  float MIN_pwm; // typically 476 microseconds, range of PWM is about 112 microseconds
  float MAX_pwm; //typically 581 microseconds
  float MIN_angle;
  float MAX_angle; //typically 37 degrees
};

struct thread_ids{ // thread id's to use for killing threads
  int imu;
  int manual_controller;
  int actuator_controller;
  int pid_controller;
  int kinematic_controller;
};

/************ENUM DEFINTIONS************/
enum drive_mode {manual, actuator, pid, kinematic, neutral};
/*********PROTOTYPES**************/
void steering_isr();                         // reads steering encoder pwm
void rec_servo_isr();                        // reads steering command from second channel of radio controller
void rec_esc_isr();                          // reads throttle command from second channel of radio controller
void imu_read();                             // reads imu
void actuator_controller();
void manual_controller();
void pid_controller();
void kinematic_controller();
void set_drive_mode(enum drive_mode);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);                      // maps x from [in_min, in_max] to [out_min, out_max]
void moveCb(const ackermann_msgs::AckermannDriveStamped &msg);
void calibrate_steering(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);
bool switchManualCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);
bool switchActuatorCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);
bool switchNeutralCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);
bool switchPidCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);
bool switchKinematicCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res);

/************VARIABLES********************/

Encoder fl(_FL_A, _FL_B);                 // encoders
Encoder fr(_FR_A, _FR_B);
Encoder rl(_RL_A, _RL_B);
Encoder rr(_RR_A, _RR_B);
Servo esc;                               // speed controller
Servo steer;                             // steering servo

enum drive_mode mode;                                         // current drive mode variable
struct steering_calibration_parameters steering_calab_params; // stores vehicle calibration parameters

struct thread_ids t_id;
struct isr_variables isr_vars;           // interrupt variables

ros::NodeHandle nh;                      // ros node
struct ros_message ros_msgs;             // ros message

ros::Publisher pub_fl_feedback("truck/feedback/wheel/fl", &ros_msgs.flFeedback);
ros::Publisher pub_fr_feedback("truck/feedback/wheel/fr", &ros_msgs.frFeedback);
ros::Publisher pub_rl_feedback("truck/feedback/wheel/rl", &ros_msgs.rlFeedback);
ros::Publisher pub_rr_feedback("truck/feedback/wheel/rr", &ros_msgs.rrFeedback);
ros::Publisher pub_steering_feedback("truck/feedback/steering", &ros_msgs.steeringFeedback);
ros::Publisher pub_manual_steering_command("truck/manual/steering", &ros_msgs.manualSteering);
ros::Publisher pub_manual_throttle_command("truck/manual/throttle", &ros_msgs.manualThrottle);
ros::Publisher pub_imu("truck/imu/data", &ros_msgs.imu_dat);

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub_command("truck/in/command", &moveCb); // statically allocated for rosserial using templates

//services for switching drive modes
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_mode_manual("truck/mode/manual", &switchManualCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_mode_actuator("truck/mode/actuator", &switchActuatorCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_mode_kinematic("truck/mode/kinematic", &switchKinematicCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_mode_pid("truck/mode/pid", &switchPidCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_mode_neutral("truck/mode/neutral", &switchNeutralCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> serv_calib_steering("truck/calibrate/steering", &calibrate_steering);

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);                               // imu related, 0x28 is I2C address of the imu board
volatile sensors_event_t orientationData , angVelocityData , linearAccelData; // these vars are used in imu_read function

/******************************************************************************/
void setup(){

  steering_calab_params.MIN_pwm = 476;
  steering_calab_params.MAX_pwm = 581;
  steering_calab_params.MIN_angle = -37;
  steering_calab_params.MAX_angle = 37;
  
  // configuring pins for servo library commands: servo.attach(pin, min, max)
  steer.attach(_SERVO);
  esc.attach(_ESC);
  
  steer.writeMicroseconds(_SERVO_NEUTRAL);          // send neutral commands for safety
  esc.writeMicroseconds(_ESC_NEUTRAL);
  
  attachInterrupt(_STEER,steering_isr,CHANGE);      // steering encoder pwm
  attachInterrupt(_REC_ESC,rec_esc_isr,CHANGE);     // reciever esc pwm
  attachInterrupt(_REC_SERVO,rec_servo_isr,CHANGE); // reciever servo pwm
  
  nh.initNode();
  nh.advertise(pub_fl_feedback);
  nh.advertise(pub_fr_feedback);
  nh.advertise(pub_rl_feedback);
  nh.advertise(pub_rr_feedback);
  nh.advertise(pub_steering_feedback);
  nh.advertise(pub_manual_steering_command);
  nh.advertise(pub_manual_throttle_command);
  nh.advertise(pub_imu);
  nh.subscribe(sub_command);
  nh.advertiseService(serv_mode_manual);
  nh.advertiseService(serv_mode_actuator);
  nh.advertiseService(serv_mode_pid);
  nh.advertiseService(serv_mode_kinematic);
  nh.advertiseService(serv_mode_neutral);
  nh.advertiseService(serv_calib_steering);
  /*****IMU Stuff***********/
  if(bno.begin()){
    delay(1000);//wait for imu
    bno.setExtCrystalUse(true);
    t_id.imu = threads.addThread(imu_read);  // asigns a thread to imu read and saves its id in t_id.imu field, from TeensyThreads.h.
    nh.loginfo("Connection to IMU established");
  }else{
    //if not detected, give error and continue
    nh.logerror("IMU not deteced, proceeding without IMU");
  } 
  ros_msgs.imu_dat.header.frame_id = "base_link";
  set_drive_mode(manual);
  //calibrate_steering();
}

unsigned long startMillis; //used for timing of ros rate
unsigned long currentMillis;
int delaytime;
unsigned long steering_pwm_copy;
unsigned long rec_esc_pwm_copy;
unsigned long rec_servo_pwm_copy;

void loop()
{
  startMillis=millis();
  // reading encoders
  ros_msgs.flFeedback.data = fl.read();            // reading encoders
  pub_fl_feedback.publish(&ros_msgs.flFeedback);   // publishing the encoder reading
  ros_msgs.frFeedback.data = fr.read();
  pub_fr_feedback.publish(&ros_msgs.frFeedback);
  ros_msgs.rlFeedback.data = rl.read();
  pub_rl_feedback.publish(&ros_msgs.rlFeedback);
  ros_msgs.rrFeedback.data = rr.read();
  pub_rr_feedback.publish(&ros_msgs.rrFeedback);
  
  noInterrupts();                                   // disable interrupts for reading ISR shared variables
  steering_pwm_copy = isr_vars.steering_pwm;        // current steering angle
  rec_esc_pwm_copy = isr_vars.reciever_esc_pwm;     // current throttle, from radio controller
  rec_servo_pwm_copy = isr_vars.reciever_servo_pwm; // current steering command, from radio controller
  interrupts();
  
  ros_msgs.steeringFeedback.data = mapf(steering_pwm_copy, steering_calab_params.MIN_pwm, steering_calab_params.MAX_pwm, steering_calab_params.MIN_angle, steering_calab_params.MAX_angle); //outputs angle in degrees of steering, center being 0
  ros_msgs.manualSteering.data = mapf(rec_servo_pwm_copy,_REC_SERVO_MIN,_REC_SERVO_MAX,-1,1);  // outputs steering percetange from -1 to 1
  ros_msgs.manualThrottle.data = mapf(rec_esc_pwm_copy,_REC_ESC_MIN,_REC_ESC_MAX,-1,1);        // outputs throttle percetange from -1 to 1
    
  pub_steering_feedback.publish(&ros_msgs.steeringFeedback);     // publish current steering reading
  pub_manual_steering_command.publish(&ros_msgs.manualSteering); // publish current radio control steering command
  pub_manual_throttle_command.publish(&ros_msgs.manualThrottle); // publish current radio control throttle command

  // imu data publication
  ros_msgs.imu_dat.orientation.x = orientationData.orientation.x;
  ros_msgs.imu_dat.orientation.y = orientationData.orientation.y;
  ros_msgs.imu_dat.orientation.z = orientationData.orientation.z;
  ros_msgs.imu_dat.angular_velocity.x = angVelocityData.gyro.x;
  ros_msgs.imu_dat.angular_velocity.y = angVelocityData.gyro.y;
  ros_msgs.imu_dat.angular_velocity.z = angVelocityData.gyro.z;
  ros_msgs.imu_dat.linear_acceleration.x = linearAccelData.acceleration.x;
  ros_msgs.imu_dat.linear_acceleration.y = linearAccelData.acceleration.y;
  ros_msgs.imu_dat.linear_acceleration.z = linearAccelData.acceleration.z;
  ros_msgs.imu_dat.header.stamp = nh.now();
  pub_imu.publish(&ros_msgs.imu_dat);                                      // slows down serial, comment for debugging
  
  nh.spinOnce(); // issue all communications
  
  currentMillis = millis();
  currentMillis = currentMillis - startMillis;
  delaytime = 10 - currentMillis - 1;
  if(delaytime<0){delaytime = 0;}
  delay(delaytime);
}

// function definitions
void moveCb(const ackermann_msgs::AckermannDriveStamped &msg){
  
}

// manual rc control mode
bool switchManualCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  set_drive_mode(manual);
  if(mode == manual){
    nh.loginfo("Set drive mode: Manual");
    return true;
  }
  return false;
}

// raw throttle and steering mode
bool switchActuatorCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  set_drive_mode(actuator);
  if(mode == actuator){
    nh.loginfo("Set drive mode: Actuator");
    return true;
  }
  return false;
}

//neutral mode
bool switchNeutralCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  set_drive_mode(neutral);
  if(mode == neutral){
    nh.loginfo("Set drive mode: Neutral");
    return true;
  }
  return false;
}

bool switchPidCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  set_drive_mode(pid);
  if(mode == pid){
    nh.loginfo("Set drive mode: PID");
    return true;
  }
  return false;
}

bool switchKinematicCb(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  set_drive_mode(kinematic);
  if(mode == kinematic){
    nh.loginfo("Set drive mode: Kinematic");
    return true;
  }
  return false;
}

// calibrating steering encoder
void calibrate_steering(const std_srvs::Empty::Request &req, const std_srvs::Empty::Response &res){
  enum drive_mode prev = mode;
  set_drive_mode(neutral); // nothing else sending commands to esc and servo
  threads.delay(1000);
  nh.loginfo("Steering Calibration in Progress");
  steer.write(0);                                        // commands steering servo to far clockwise
  threads.delay(1000);
  noInterrupts();
  steering_calab_params.MIN_pwm = isr_vars.steering_pwm; // get min pwm reading
  interrupts();
  steer.write(180);                                      // command servo to far counter clockwise
  threads.delay(1000);
  noInterrupts();
  steering_calab_params.MAX_pwm = isr_vars.steering_pwm; //get max pwm reading
  interrupts();

  float rangeAngle = abs(steering_calab_params.MIN_pwm - steering_calab_params.MAX_pwm)*0.351564; // 0.3515625 = 360/1024
  steering_calab_params.MAX_angle = rangeAngle/2;
  steering_calab_params.MIN_angle = -rangeAngle/2;
  set_drive_mode(prev);
  nh.loginfo("Steering Calibration complete.");
  }
  
void manual_controller(){
  unsigned long esc_copy;
  unsigned long servo_copy;
  while(1){
    
    noInterrupts();
    servo_copy = isr_vars.reciever_servo_pwm;
    esc_copy = isr_vars.reciever_esc_pwm;
    interrupts();
    steer.writeMicroseconds(servo_copy);
    esc.writeMicroseconds(esc_copy);
    
    threads.delay(10); //100 hz loop
  }
}

void actuator_controller(){
  while(1){
      threads.delay(10); //100 hz loop
  }
}

void pid_controller(){
  while(1){
      threads.delay(10); //100 hz loop
  }
}

void kinematic_controller(){
  while(1){
      threads.delay(10); //100 hz loop
  }
}

// Sets current driving mode of the vehicle
// Input:  desired mode
// Output: None
void set_drive_mode(enum drive_mode desired_mode){
  // Terminates the current driving mode
  switch(mode){
    case manual:
      threads.kill(t_id.manual_controller);
      break;
    case actuator:
      threads.kill(t_id.actuator_controller);
      break;
    case pid:
      threads.kill(t_id.pid_controller);
    case kinematic:
      threads.kill(t_id.kinematic_controller);
    case neutral:
      break;
  }
  // Command neutral value to steering servo and throttle.
  esc.writeMicroseconds(_ESC_NEUTRAL);
  steer.writeMicroseconds(_SERVO_NEUTRAL);
  // Set driving mode to the current desired one.
  switch(desired_mode){
   case manual:
     mode = manual;
     t_id.manual_controller = threads.addThread(manual_controller);
     break;
   case actuator:
     mode = actuator;
     t_id.actuator_controller = threads.addThread(actuator_controller);
     break;
   case pid:
     mode = pid;
     t_id.pid_controller = threads.addThread(pid_controller);
     break;
   case kinematic:
     mode = kinematic;
     t_id.kinematic_controller = threads.addThread(kinematic_controller);
     break;
   case neutral:
     mode = neutral;
     break;
  }
}
// Sets current driving mode of the vehicle
// Input:  None
// Output: None
void imu_read(){
  while(1){
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);       // Reading imu data, refer to imu github page for API.
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    threads.delay(10); //100 hz loop
  }
}

void rec_esc_isr(){
  if(digitalRead(_REC_ESC) == HIGH){
    isr_vars.esc_micros = micros();
  }else{
    if(isr_vars.esc_micros > 0){
      if(micros() > isr_vars.esc_micros && 1000 < (micros() - isr_vars.esc_micros) && 2010 > (micros() - isr_vars.esc_micros)){
        isr_vars.reciever_esc_pwm = micros() - isr_vars.esc_micros;
      }
    }
  }
}

void rec_servo_isr(){
  if(digitalRead(_REC_SERVO) == HIGH){
    isr_vars.servo_micros = micros();
  }else{
    if(isr_vars.servo_micros > 0){
      if(micros() > isr_vars.servo_micros && 1000 < (micros() - isr_vars.servo_micros) && 2010 > (micros() - isr_vars.servo_micros)){
        isr_vars.reciever_servo_pwm = micros() - isr_vars.servo_micros;
      }
    }
  }
}

void steering_isr(){
  if(digitalRead(_STEER) == HIGH){
    isr_vars.steering_micros = micros();
  }else{
    if(isr_vars.steering_micros > 0){
      if(micros() > isr_vars.steering_micros){
        isr_vars.steering_pwm = micros() - isr_vars.steering_micros;
      }
    }
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
