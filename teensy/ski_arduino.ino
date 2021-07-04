/******************* Truck low level control: ski-stunt ***********************/
/*  This program publishes sensor data and gets high level command from the ROS 
 *  host pc.
 *  May 2021
 *  Farshid Asadi
 */
/******************* Header files *********************************************/
//#define USE_USBCON
#include <ros.h>                   // ROS.
#include <std_msgs/Int32.h>      // ROS Int32 type
#include <std_msgs/Float32.h>      // ROS Float32 type
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float32MultiArray.h>
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
/******************* Pin connections ******************************************/
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
/******************* Some constants *******************************************/
int DT = 6;                     // Sampling period: 14 for 50Hz ROS loop
#define _REC_ESC_MIN 1065       // Radio control PWM range
#define _REC_ESC_MAX 2006       //
#define _REC_SERVO_MIN 1065     //
#define _REC_SERVO_MAX 2006     //
const int _ESC_NEUTRAL = 1514;      //
const int _SERVO_NEUTRAL = 1640;    //
const int _ESC_MIN = 1000;          // min ESC_PWM_micros
const int _ESC_MAX = 2000;
// max ESC_PWM_micros 
const float _M_ESC_RAD_S = 5.228; // ESC_PWM_micros = M_ESC* {Rad/s} + B_ESC
const float _B_ESC_RAD_S = 1514;    //
const float _M_ESC_RPM = 0.59299;   // ESC_PWM_micros = M_ESC* {RPM} + B_ESC
const float _B_ESC_RPM = 1514;      //
/******************* Function prototypes **************************************/
/******************* FStruct and enum variable defs ***************************/
//ROS related
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
struct s_c_p{// precision of the encoder is ~ 2p/1024 = 0.006 rad = 0.35 deg
  // Range of PWM is about 114 microseconds
  int MIN_pwm;   // 699
  int MAX_pwm;   // 815
  int CENT_pwm;  // 757
  float MIN_angle; // -20.7422 Deg or -58*2*PI/1024 Rad or -0.362
  float MAX_angle; // 20.7422 Deg or 58*2*PI/1024 Rad ot 0.362
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
#define fb_size 13 // feedback message size
struct ros_message{
  std_msgs::Float32MultiArray fb;
  //  0: fl, front left wheel angular velocity, Rad/s
  //  1: fr, front right wheel angular velocity
  //  2: rl, rear left wheel angular velocity
  //  3: rr, rear right wheel angular velocity
  //  4: steerFB, steering wheel angle, Rad
  //  5: recSteer, receiver steering PWM signal
  //  6: recThrottle, receiver throttle PWM signal
  //  7: imuAngx, Angle of rotation in xyz order in bodyframe, Rad
  //  8: imuAngz
  //  9: imuWx, d/dt{imuAngx}, Rad/s
  // 10: imuWz, d/dt{imuAngz}
  // 11: imuAccx, linear acceleration x axis, m/s2
  // 12: imuAccy
  std_msgs::Int32 cmdMode;       // Command mode
                                 // [1, 2, other]->[Radio,Computer, neutral]
  std_msgs::Float32 cmdSteer;    // Steering command, Rad
  std_msgs::Float32 cmdThrottle; // Throttle command, Rad/s
};
/******************* Function prototypes **************************************/
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
/******************* Global vars and objects **********************************/
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
float fb[fb_size];
std_msgs::MultiArrayDimension dim[1];
std_msgs::MultiArrayLayout layout;
ros::NodeHandle nh; // ROS node
// Publishers
ros::Publisher pub_fb("truck/fb", &ros_msgs.fb);
// Subscribers
void cmdModeCb(const std_msgs::Int32 &msg){ros_msgs.cmdMode = msg;}
void cmdSteerCb(const std_msgs::Float32 &msg){ros_msgs.cmdSteer = msg;}
void cmdThrottleCb(const std_msgs::Float32 &msg){ros_msgs.cmdThrottle=msg;}
ros::Subscriber<std_msgs::Int32> sub_cmdMode("truck/cmd/mode",&cmdModeCb);
ros::Subscriber<std_msgs::Float32> sub_cmdSteer("truck/cmd/steer",&cmdSteerCb);
ros::Subscriber<std_msgs::Float32> sub_cmdThrottle("truck/cmd/throttle",
                                                    &cmdThrottleCb);
/******************* Setup ****************************************************/
void setup() {
  // Steering calibration, be carefull about these values.
  steer_calab_params.MIN_pwm = 699;
  steer_calab_params.MAX_pwm = 815;
  steer_calab_params.CENT_pwm = 757;
  steer_calab_params.MIN_angle = -58*2*PI/1024;
  steer_calab_params.MAX_angle = 58*2*PI/1024;
  steer.attach(_SERVO);
  esc.attach(_ESC);
  // Neutraling Steering and throttle.
  steer.writeMicroseconds(_SERVO_NEUTRAL);
  esc.writeMicroseconds(_ESC_NEUTRAL);
  // setting up PWM readers for different parts
  attachInterrupt(_STEER,steer_isr,CHANGE); // steering encoder pwm
  attachInterrupt(_REC_SERVO,rec_servo_isr,CHANGE); // reciever servo pwm
  attachInterrupt(_REC_ESC,rec_esc_isr,CHANGE);     // reciever esc pwm
  // ROS node initialization
  dim[0].size = fb_size;
  dim[0].stride = fb_size;
  layout.dim = dim;
  layout.data_offset = 0;
  ros_msgs.fb.layout = layout;
  ros_msgs.fb.data_length = fb_size;
  ros_msgs.fb.layout.dim_length = 1;
  nh.initNode();
  // Advertise published topics
  nh.advertise(pub_fb);
  // Subscribe to topics
  nh.subscribe(sub_cmdMode);
  nh.subscribe(sub_cmdSteer);
  nh.subscribe(sub_cmdThrottle);
  // IMU initialization
  if(bno.begin()){
    delay(1000);//wait for imu
    bno.setExtCrystalUse(true);
    t_id.imu = threads.addThread(imu_read);
  }
  drive_mode = 0; // setting driving mode to neutral
}
/******************* Main loop ************************************************/
// Timing of the loop
long newMillis;
long oldMillis = millis();
int period = 20;
void loop() {
  // Reading subscribers messages
  gen_command(ros_msgs.cmdMode.data);
  esc.writeMicroseconds(esc_cmd);
  steer.writeMicroseconds(steer_cmd);
  // Timing
  newMillis = millis();
  period = max(newMillis - oldMillis,1);
  oldMillis = newMillis;
  // Reading encoders
  wheel_inc_sens();
  fb[0] = wheel_inc.FL*speed_ratio/period; // reading velocities
  fb[1] = wheel_inc.FR*speed_ratio/period;
  fb[2] = wheel_inc.RL*speed_ratio/period;
  fb[3] = wheel_inc.RR*speed_ratio/period;
  // Reading PWM signals: receiver commands and steer encoder
  read_pwm_signal();
  fb[4] = steer_ang_copy;
  fb[5] = rec_servo_pwm_copy;
  fb[6] = rec_esc_pwm_copy;
  // Reading IMU
  //  7: imuAngx, Angle of rotation in xyz order in bodyframe, Rad
  //  8: imuAngz
  //  9: imuWx, d/dt{imuAngx}, Rad/s
  // 10: imuWz, d/dt{imuAngz}
  // 11: imuAccx, linear acceleration x axis, m/s2
  // 12: imuAccy
  // IMU axes are right hand rule, X to ground, Y to rear, Z to left
  // Car xyz: x to forward, y to left, z to up
  fb[7] = -orientationData.orientation.y*deg2rad;     // alpha
  fb[8] = 2*PI-orientationData.orientation.x*deg2rad; // psi
  // Gyro and lin acc axes are different: X to right, Y to forward, Z to up
  fb[9] = deg2rad*angVelocityData.gyro.y;   // alpha_dot
  fb[10] = deg2rad*angVelocityData.gyro.z;  // psi_dot
  fb[11] = linearAccelData.acceleration.y;  // u_dot
  fb[12] = -linearAccelData.acceleration.z; // v_dot
  fb[12] = ros_msgs.cmdSteer.data;
  ros_msgs.fb.data = fb;
  // Publishing messages
  pub_fb.publish(&ros_msgs.fb);
  delay(DT);
  nh.spinOnce(); // issue services and subscribes
}
/******************* Utility function definitions *****************************/
// Converts commands to appropriate PWM in each deriving mode
// Input:  current mode
// Output: None
void gen_command(int current_mode){
  static int prev_mode = 0; // storing previous mode
  switch(current_mode){
    case 2:
      // Control by computer
      //nh.loginfo("Computer mode.");
      act_steer_p(ros_msgs.cmdSteer.data);
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
// Steer set
// Input: Desired steering angle in Rad
// Output: None
void act_steer(float desired_rad){
  //static int error_sum = 0;
  int desired_enc = int(rad2enc*desired_rad);
  steer_cmd = _SERVO_NEUTRAL+int(desired_enc*5.218);
  steer_cmd = max(1200,min(steer_cmd,2000)); // for savox
}
// Steer PID
// Input: Desired steering angle in Rad
// Output: None
void act_steer_p(float desired_rad){
  static float errorsum = 0;
  float kp = 400.0; // Setting KP = 0, removes the P control
  float ki = 75;       // Setting Kp = 0, removed I control
  int constant_cmd = _SERVO_NEUTRAL + int(desired_rad*rad2enc*5.218);
  float error = desired_rad - steer_ang_copy;
  errorsum += error;
  errorsum = max(-.35,min(errorsum,.35)); // clamping integral error sum
  int var_cmd = max(-300,min(int(kp*error + ki*errorsum),300));
  //steer_cmd = max(1050, min(constant_cmd + var_cmd,2150));   // shitty servo
  steer_cmd = max(1200, min(constant_cmd + var_cmd,2200));   // savox
}
// ESC PID
// Input: Desired main shaft velocity Rad/s
// Output: None
void act_esc(float des_vel){
  static bool nonzeroflag = true;
  float vel = 0.5*(wheel_inc.RL + wheel_inc.RR)*speed_ratio/period;
  des_vel = des_vel + max(-10,min(75*(des_vel-vel),10));
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
/******************* Hardware function definitions ****************************/
// ISRs
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
// Reads current values of volatile variables of ISRs
// Input: None
// Output: none
void read_pwm_signal(){
  noInterrupts();
  steer_pwm_copy = isr_vars.steer_pwm;
  rec_esc_pwm_copy = isr_vars.rec_esc_pwm;
  rec_servo_pwm_copy = isr_vars.rec_servo_pwm;
  interrupts();
  steer_ang_copy = (steer_pwm_copy - steer_calab_params.CENT_pwm )/rad2enc;
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
    threads.delay(20); //50 hz loop
  }
}
