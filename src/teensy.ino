/*===========================================================================*/
/*  This program publishes sensor data and gets high level command from the ROS 
 *  host pc.
 *  May 2021
 *  Farshid Asadi
 */
/*========== Header files ===================================================*/
//#define USE_USBCON
#include <ros.h>                   // ROS.
#include <sensor_msgs/Imu.h>       // ROS IMU message type.
#include <std_msgs/Int32.h>        // ROS int 64
#include <std_msgs/Float32.h>      // ROS float32 type
// Sensor related libraries
#include <Encoder.h>          // Encoder library, up to 100 kHz.
#include <Servo.h>            // Servo library, for throttle and steering.
#include <Wire.h>             // I2C library.
#include <Adafruit_Sensor.h>  // Read/convert sensor data to std description.
#include <Adafruit_BNO055.h>  // Adafruit IMU library.
#include <utility/imumaths.h> // Adafruit labrary for imu calculations.
// Teensy related headers
#include<TeensyThreads.h>     // Multithreading for Teensy.
#define USE_TEENSY_HW_SERIAL  // Keyword to use teensy hardware serial.
/*========== Pin connections ================================================*/
#define _SERVO 1 // Steering servo PWM
#define _ESC 3   // Speed controller PWM
#define _FL_A 5  // Wheel encoder, _Front or Rear""Left or Right"_"A or B pin
#define _FL_B 6
#define _FR_A 7
#define _FR_B 8
#define _RL_A 9
#define _RL_B 10
#define _RR_A 11
#define _RR_B 12
// Other readings
#define _REC_SERVO 14 // Steering PWM from radio control
#define _REC_ESC   15 // Throttle PWM from radio control
#define _STEER 23     // Absolute encoder for steering 10 bit PWM
/*========== Some constants ================================================*/
int DT = 8;                     // Sampling period: 8 for 50Hz ROS loop
#define _REC_ESC_MIN 1065       // Radio control PWM range
#define _REC_ESC_MAX 2006       //
#define _REC_SERVO_MIN 1065     //
#define _REC_SERVO_MAX 2006     //
const int _ESC_NEUTRAL = 1514;      //
const int _SERVO_NEUTRAL = 1610;    //
const int _ESC_MIN = 1000;          // min ESC_PWM_micros
const int _ESC_MAX = 2000;          // max ESC_PWM_micros 
const float _M_ESC_RAD_S = 5.228; // ESC_PWM_micros = M_ESC* {Rad/s} + B_ESC
const float _B_ESC_RAD_S = 1514;    //
const float _M_ESC_RPM = 0.59299;   // ESC_PWM_micros = M_ESC* {RPM} + B_ESC
const float _B_ESC_RPM = 1514;      //
/*========== Struct and enum variable defs =================================*/
struct isr_variables{
  volatile int steer_pwm;     // PWM in micros, 0.351564 deg/micros.
  volatile int rec_esc_pwm;   // PWM in micros, 1512 neutral.
  volatile int rec_servo_pwm; // PWM in micros, neutral 1523.
  volatile int steer_micros;
  volatile int rec_esc_micros;
  volatile int rec_servo_micros;
  volatile float steering_angle;
};
// Steering_Calibration_parameter
struct s_c_p{
  // Range of PWM is about 114 microseconds
  int MIN_pwm;   // 530
  int MAX_pwm;   // 644
  int CENT_pwm;  // 587
  float MIN_angle; // -20.039 Deg or -57*2*PI/1024 Rad
  float MAX_angle; // 20.039 Deg or 57*2*PI/1024 Rad
}; 
// Encoder increments
struct wheel_inc_struct{
  int FL;
  int FR;
  int RL;
  int RR;
};
// thread id's to use for killing threads
struct thread_ids{
  int imu;
  int manual;
  int computer;
  int neutral;
};
//ROS related
struct ros_message{
  std_msgs::Float32 fl;          // front left wheel velocity
  std_msgs::Float32 fr;          // front right
  std_msgs::Float32 rl;          // rear left
  std_msgs::Float32 rr;          // rear right
  std_msgs::Float32 steerFB;     // FB: Feedback
  std_msgs::Float32 recSteer;    // rec: receiver
  std_msgs::Float32 recThrottle;
  std_msgs::Int32 cmdMode;      
  std_msgs::Float32 cmdSteer;    // cmd: command
  std_msgs::Float32 cmdThrottle;
  sensor_msgs::Imu imu;
  std_msgs::Int32 ltime;
};
/*========== Function prototypes ===========================================*/
// mapf: maps x from [in_min, in_max] to [out_min, out_max]
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void wheel_inc_sens();  // Calculates wheels encoder increment.
void imu_read();        // Reads IMU.
void rec_esc_isr();     // Read receiver throtle PWM "on time".
void rec_servo_isr();   // Read receiver steering servo PWM "on time".
void steer_isr();       // Read steering encoder PWM "on time".
void read_pwm_signal(); // Read current values of volatile variables of ISRs.
void act_steer(float desired_rad);   // Converts desired steer angle in Rad
void act_steer_p(float desired_rad); // Converts desired steer angle in Rad
void act_esc(float des_vel);         // Converts throttle command in Rad/s 
void gen_command(int current_mode);  // Converts command to PWM appropriately.
/*========== Global vars and objects ======================================*/
// User defined
int steer_pwm_copy;
float steer_ang_copy;
int rec_esc_pwm_copy;
int rec_servo_pwm_copy;
// Control status and commands
int drive_mode = 0; // 1 Radio control, 2 Computer, otherwise Neutral
int steer_cmd = _SERVO_NEUTRAL;
int esc_cmd = _ESC_NEUTRAL;

s_c_p steer_calab_params;          // vehicle calibration parameters

struct thread_ids t_id;            // keeping record of thread ids
struct isr_variables isr_vars;     // interrupt variables
struct wheel_inc_struct wheel_inc; // wheel encoder increments

// Sensor readings and actuation
float speed_ratio = 2*PI;     // Gives Rad/s, must be divided by period (ms)
float speed_ratio_rpm = 60.0; // Gives RPM, must be divided by period (ms)
float deg2rad = PI/180;
float rad2enc = 1024/(2*PI);
Encoder fl(_FL_A, _FL_B);     // encoders
Encoder fr(_FR_A, _FR_B);
Encoder rl(_RL_A, _RL_B);
Encoder rr(_RR_A, _RR_B);
Servo esc;                    // speed controller
Servo steer;                  // steering servo
// IMU related
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28); // 0x28 is I2C address
sensors_event_t orientationData, angVelocityData, linearAccelData;
// ROS related
struct ros_message ros_msgs;
ros::NodeHandle nh; // ROS node
// Publishers
ros::Publisher pub_fl("truck/fb/wheel/fl", &ros_msgs.fl);
ros::Publisher pub_fr("truck/fb/wheel/fr", &ros_msgs.fr);
ros::Publisher pub_rl("truck/fb/wheel/rl", &ros_msgs.rl);
ros::Publisher pub_rr("truck/fb/wheel/rr", &ros_msgs.rr);
ros::Publisher pub_steerFB("truck/fb/steer", &ros_msgs.steerFB);
ros::Publisher pub_cmdSteerfb("truck/fb/cmdsteer", &ros_msgs.cmdSteer);
ros::Publisher pub_recSteer("truck/fb/rec/steer", &ros_msgs.recSteer);
ros::Publisher pub_recThrottle("truck/fb/rec/throttle",&ros_msgs.recThrottle);
ros::Publisher pub_imu("truck/fb/imu", &ros_msgs.imu);
ros::Publisher pub_ltime("truck/fb/ltime", &ros_msgs.ltime);
// Subscribers
void cmdModeCb(const std_msgs::Int32 &msg){ros_msgs.cmdMode = msg;}
void cmdSteerCb(const std_msgs::Float32 &msg){ros_msgs.cmdSteer = msg;}
void cmdThrottleCb(const std_msgs::Float32 &msg){ros_msgs.cmdThrottle=msg;}
ros::Subscriber<std_msgs::Int32> sub_cmdMode("truck/cmd/mode",
                                                 &cmdModeCb);
ros::Subscriber<std_msgs::Float32> sub_cmdSteer("truck/cmd/steer",
                                                 &cmdSteerCb);
ros::Subscriber<std_msgs::Float32> sub_cmdThrottle("truck/cmd/throttle",
                                                    &cmdThrottleCb);
/*========== Setup ========================================================*/
void setup() {
  // Steering calibration, be carefull about these values.
  steer_calab_params.MIN_pwm = 530;
  steer_calab_params.MAX_pwm = 644;
  steer_calab_params.CENT_pwm = 587;
  steer_calab_params.MIN_angle = -57*2*PI/1024;
  steer_calab_params.MAX_angle = 57*2*PI/1024;
  pinMode(17, OUTPUT);    // sets the digital pin 13 as output
  //Serial.begin(57600);
  steer.attach(_SERVO);
  esc.attach(_ESC);
  // Neutraling Steering and throttle.
  steer.writeMicroseconds(_SERVO_NEUTRAL);
  esc.writeMicroseconds(_ESC_NEUTRAL);
  // setting up PWM readers for different parts
  attachInterrupt(_REC_ESC,rec_esc_isr,CHANGE);     // reciever esc pwm
  attachInterrupt(_REC_SERVO,rec_servo_isr,CHANGE); // reciever servo pwm
  attachInterrupt(_STEER,steer_isr,CHANGE); // steering encoder pwm
  // ROS node initialization
  nh.initNode();
  // Advertise published topics
  nh.advertise(pub_fl);
  nh.advertise(pub_fr);
  nh.advertise(pub_rl);
  nh.advertise(pub_rr);
  nh.advertise(pub_steerFB);
  nh.advertise(pub_cmdSteerfb);
  nh.advertise(pub_recSteer);
  nh.advertise(pub_recThrottle);
  //nh.advertise(pub_imu);
  nh.advertise(pub_ltime);
  // Subscribe to topics
  nh.subscribe(sub_cmdMode);
  nh.subscribe(sub_cmdSteer);
  nh.subscribe(sub_cmdThrottle);
  // IMU initialization
  /*
  if(bno.begin()){
    delay(1000);//wait for imu
    bno.setExtCrystalUse(true);
    t_id.imu = threads.addThread(imu_read);
    nh.loginfo("Connection to IMU established");
  }else{
    //if not detected, give error and continue
    nh.logerror("IMU not deteced, proceeding without IMU");
  } 
  */
  ros_msgs.imu.header.frame_id = "base_link";
  drive_mode = 2;
}

long newMillis;
long oldMillis = millis();
int period = 20;
float input = 0;//_ESC_NEUTRAL;
void loop() {
  // Reading subscribers messages
  gen_command(ros_msgs.cmdMode.data);
  // Timing
  newMillis = millis();
  period = max(newMillis - oldMillis,1);
  oldMillis = newMillis;
  // Reading encoders
  wheel_inc_sens();
  ros_msgs.fl.data = wheel_inc.FL*speed_ratio/period; // reading velocities
  ros_msgs.fr.data = wheel_inc.FR*speed_ratio/period;
  ros_msgs.rl.data = wheel_inc.RL*speed_ratio/period;
  ros_msgs.rr.data = wheel_inc.RR*speed_ratio/period;
  ros_msgs.ltime.data = newMillis;
  // publishing the encoder reading
  pub_fl.publish(&ros_msgs.fl);
  pub_fr.publish(&ros_msgs.fr);
  pub_rl.publish(&ros_msgs.rl);
  pub_rr.publish(&ros_msgs.rr);
  pub_ltime.publish(&ros_msgs.ltime);

  // Reading IMU
  ros_msgs.imu.orientation.x = orientationData.orientation.x*deg2rad;
  ros_msgs.imu.orientation.y = -orientationData.orientation.y*deg2rad;
  ros_msgs.imu.orientation.z = orientationData.orientation.z*deg2rad;
  ros_msgs.imu.angular_velocity.x = -deg2rad*angVelocityData.gyro.x;
  ros_msgs.imu.angular_velocity.y = deg2rad*angVelocityData.gyro.y;
  ros_msgs.imu.angular_velocity.z = -deg2rad*angVelocityData.gyro.z;
  ros_msgs.imu.linear_acceleration.x = linearAccelData.acceleration.x;
  ros_msgs.imu.linear_acceleration.y = linearAccelData.acceleration.y;
  ros_msgs.imu.linear_acceleration.z = linearAccelData.acceleration.z;
  ros_msgs.imu.header.stamp = nh.now();
  // Publishing IMU
  //pub_imu.publish(&ros_msgs.imu);
  
  // Reading PWM signals: receiver commands and steer encoder
  read_pwm_signal();
  ros_msgs.steerFB.data = -steer_ang_copy; // Comply with ROS program
  ros_msgs.recSteer.data = rec_servo_pwm_copy;
  ros_msgs.recThrottle.data = rec_esc_pwm_copy;
  // Publishing the PWM sensory feedback
  pub_steerFB.publish(&ros_msgs.steerFB);
  pub_recSteer.publish(&ros_msgs.recSteer);
  pub_recThrottle.publish(&ros_msgs.recThrottle);
  pub_cmdSteerfb.publish(&ros_msgs.cmdSteer);
 
  /*
  // Receiver
  if (Serial.available()>0){
    input = Serial.parseFloat('\n');
  }*/
  /*
  char str[80];
  sprintf(str, "%+06.2f %+06.3f %+06.2f",
          ros_msgs.imu.orientation.x,
          ros_msgs.imu.orientation.y,
          ros_msgs.imu.orientation.z);
          */
  
  /*sprintf(str, "%+06.2f %+06.2f %+06.2f",
          ros_msgs.imu.angular_velocity.x,
          ros_msgs.imu.angular_velocity.y,
          ros_msgs.imu.angular_velocity.z);*/
  /*
  sprintf(str, "%+06.2f %+06.2f %+06.2f",
          ros_msgs.imu.linear_acceleration.x,
          ros_msgs.imu.linear_acceleration.y,
          ros_msgs.imu.linear_acceleration.z);*/
  //sprintf(str, "%+06.2f", ros_msgs.cmdThrottle.data);
  //sprintf(str, "%+07.2f", wheel_inc.RL*speed_ratio/period);
  //sprintf(str, "%+06.2f, %+06.2f", input, steer_ang_copy);
  //sprintf(str, "%+07.2f, %+07.2f", wheel_inc.RL*speed_ratio/period, input);
  //Serial.println(rec_esc_pwm_copy);
  //Serial.println(rec_servo_pwm_copy);
  //Serial.println(steer_pwm_copy);
 
  digitalWrite(17, !digitalRead(17));
  //Serial.print(str);
  //Serial.print("  ");
  /*sprintf(str, "%+06.2f %+06.2f %+06.2f",
          ros_msgs.imu.linear_acceleration.x,
          ros_msgs.imu.linear_acceleration.y,
          ros_msgs.imu.linear_acceleration.z);*/
  //Serial.println(str);
  esc.writeMicroseconds(esc_cmd);
  steer.writeMicroseconds(steer_cmd);
  nh.spinOnce(); // issue services and subscribes
  delay(DT);
}
/*========== function defs ================================================*/
// Maps x from [in_min, in_max] to [out_min, out_max]
// Input: float x
//        float in_min, in_max
//        float out_min, out_max
// Output: float normalized
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Converts commands to appropriate PWM in each deriving mode
// Input:  current mode
// Output: None
void gen_command(int current_mode){
  static int prev_mode = 0; // storing previous mode
  switch(current_mode){
    case 2:
      // Control by computer
      //nh.loginfo("Computer mode.");
      act_steer_p(-ros_msgs.cmdSteer.data); // To comply with ROS program
      act_esc(ros_msgs.cmdThrottle.data);
      prev_mode = 2;
      break;
    case 1:
      // Control by radio controller
      //nh.loginfo("Radio control mode.");
      steer_cmd = rec_servo_pwm_copy; // To comply with ROS program
      esc_cmd = rec_esc_pwm_copy;
      prev_mode = 1;
      break;
    case 0:
      //nh.loginfo("Neutral mode.");
      steer_cmd = _SERVO_NEUTRAL;
      if(prev_mode){ // Brakes before going to neutral
      act_esc(0);
      if(!(wheel_inc.RR + wheel_inc.RL)){prev_mode = 0;} 
      }else{
      esc_cmd = _ESC_NEUTRAL;
      prev_mode = 0;
      }
      break;
    default:
      steer_cmd = _SERVO_NEUTRAL;
      if(prev_mode){ // Brakes before going to neutral
      act_esc(0);
      if(!(wheel_inc.RR + wheel_inc.RL)){prev_mode = 0;} 
      }else{
      esc_cmd = _ESC_NEUTRAL;
      prev_mode = 0;
      }
      break;
  }
}
// Reading encoder increments with main loop frequency
// Input: None
// Output: None
void wheel_inc_sens(){
  static int fl_old = fl.read();
  static int fr_old = fr.read();
  static int rl_old = rl.read();
  static int rr_old = rr.read();
  static int fl_new, fr_new, rl_new, rr_new;
  fl_new = fl.read();
  fr_new = fr.read();
  rl_new = rl.read();
  rr_new = rr.read();
  wheel_inc.FL = -fl_new + fl_old;
  wheel_inc.FR = fr_new - fr_old;
  wheel_inc.RL = -rl_new + rl_old;
  wheel_inc.RR = rr_new - rr_old;
  fl_old = fl_new;
  fr_old = fr_new;
  rl_old = rl_new;
  rr_old = rr_new;
}
// Reading IMU
// Input:  None
// Output: None
void imu_read(){
  while(1){
    // Reading imu data, refer to imu github page for API.
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    threads.delay(DT); //83 hz loop
  }
}
// Reads current values of volatile variables of ISRs
// Input: None
// Output: none
void read_pwm_signal(){
  noInterrupts();
  steer_pwm_copy = isr_vars.steer_pwm;
  rec_esc_pwm_copy = isr_vars.rec_esc_pwm;
  rec_servo_pwm_copy = isr_vars.rec_servo_pwm;
  interrupts();
  steer_ang_copy = (steer_calab_params.CENT_pwm - steer_pwm_copy)/rad2enc;
}
// ISRs
// Reading receiver CH1 PWM signal High time in micros.
// Input:  None
// Output: None
void rec_servo_isr(){
  if(digitalRead(_REC_SERVO) == HIGH){ // Starts counting
    isr_vars.rec_servo_micros = micros();
  }else{                             // finish counting
    isr_vars.rec_servo_micros = micros() - isr_vars.rec_servo_micros;
    if(isr_vars.rec_servo_micros<400 && isr_vars.rec_servo_micros>2600){
      // If not out of range or lost.
      isr_vars.rec_servo_pwm = isr_vars.rec_servo_micros;
    }else{
      // If out of range.
      isr_vars.rec_servo_pwm = _SERVO_NEUTRAL;
    }
  }
}
// Reading receiver CH2 PWM signal High time in micros.
// Input:  None
// Output: None
void rec_esc_isr(){
  if(digitalRead(_REC_ESC) == HIGH){ // Starts counting
    isr_vars.rec_esc_micros = micros();
  }else{                             // finish counting
    isr_vars.rec_esc_micros = micros() - isr_vars.rec_esc_micros;
    if(isr_vars.rec_esc_micros<1000 && isr_vars.rec_esc_micros>2010){
      // If not out of range or lost.
      isr_vars.rec_esc_pwm = isr_vars.rec_esc_micros;
    }else{
      // If out of range.
      isr_vars.rec_esc_pwm = _ESC_NEUTRAL;
    }
  }
}
// Reading steering encoder PWM signal high time in micros
// Input:  None
// Output: None
void steer_isr(){
  if(digitalRead(_STEER) == HIGH){ // Starts counting
    isr_vars.steer_micros = micros();
  }else{                             // finish counting
    isr_vars.steer_micros = micros() - isr_vars.steer_micros - 1;
    isr_vars.steer_pwm = isr_vars.steer_micros;
  }
}

// Steer set
// Input: Desired steering angle in Rad
// Output: None
void act_steer(float desired_rad){
  //static int error_sum = 0;
  int desired_enc = int(rad2enc*desired_rad);
  steer_cmd = _SERVO_NEUTRAL+int(desired_enc*5.218);
}
// Steer PID
// Input: Desired steering angle in Rad
// Output: None
void act_steer_p(float desired_rad){
  static int errorp = 0;
  float kp = 200.0/57; // Setting KP = 0, removes the P control
  float kd = 1;       // Setting KD = 0, removed D control
  int desired_enc = steer_calab_params.CENT_pwm - int(desired_rad*rad2enc);
  int constant_cmd = _SERVO_NEUTRAL + int(desired_rad*rad2enc*5.218);
  int error = steer_pwm_copy - desired_enc;
  float derror = error - errorp;
  steer_cmd = max(1050, min(constant_cmd + int(kp*error) + int(kd*derror),2150));
}
// ESC PID
// Input: Desired main shaft velocity Rad/s
// Output: None
void act_esc(float des_vel){
  static bool nonzeroflag = true;
  float vel = 0.5*(wheel_inc.RL + wheel_inc.RR)*speed_ratio/period;
  des_vel = des_vel + max(-6,min(50*(des_vel-vel),6));
  if(vel!=0){
    if(vel*des_vel<=0){
      // Breaking
      if(vel>4){
        des_vel = -70;
      }else if(vel>0){
        des_vel = -vel;
      }else if(vel>-4){
        des_vel = -vel;
      }else{
        des_vel = 70;
      }
    }
    nonzeroflag = true;
  }else{
    if(nonzeroflag){
      // If just breaked, first command neutral, then proceed.
      des_vel = 0;
      nonzeroflag = false;
    }
  }
  // int cmd_final = _ESC_NEUTRAL + int(des_vel*_M_ESC_RAD_S);
  // cubic fitting
  int cmd_final = int(1.943e-4*des_vel*des_vel*des_vel
                      - 1.895e-3*des_vel*des_vel+4.238*des_vel+_ESC_NEUTRAL);
  esc_cmd = max(_ESC_MIN,min(cmd_final,_ESC_MAX));
}
