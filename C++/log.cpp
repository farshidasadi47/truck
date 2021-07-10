/******************************************************************************/
// This program tests data logging of teensy node from USB.
// The functions are written for the truck.
// Farshid Asadi
/********** Header files ******************************************************/
#include <ros/ros.h>                        // ROS.
#include <rosbag/bag.h>                     // rosbag
#include <std_msgs/Float32.h>               // ROS Float32 type
#include <std_msgs/Int32.h>                 // ROS Int32 type
#include <geometry_msgs/Point32.h>          // Point 32 message
#include <geometry_msgs/TransformStamped.h> // For receiving Vicon data
#include <std_msgs/MultiArrayDimension.h>   // ROS MultiArray
#include <std_msgs/MultiArrayLayout.h>      // ROS MultiArray
#include <std_msgs/Float32MultiArray.h>     // ROS MultiArray
#include <tf2/LinearMath/Quaternion.h>      // For quaternion transformation
#include <tf2/LinearMath/Matrix3x3.h>       // For getting rotation angles
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <truck/state_msgs.h>              // truck costum message
#include <cmath>
#include <algorithm>
#include <vector>
#include "path/path.h"
#include "kdtree/kdtree.h"
/*========= Struct defs =====================================================*/
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
  geometry_msgs::TransformStamped vicon; // for vicon
  std_msgs::Float32 time;
  // states
  // positions and orientation
  geometry_msgs::Point32 gPos;    // Global position of CoM
  geometry_msgs::Point32 oPos;    // Global position of O
  geometry_msgs::Point32 gOrient; // Orientation of O frame [\alpha, ~, psi]
  geometry_msgs::Point32 t1Pos;   // Track point global position: [X,Y,psir]
  geometry_msgs::Point32 t2Pos;   // Track point global position: [k, s, ey]
  // Velocities
  geometry_msgs::Point32 gVel;    // Global velocity of CoM
  geometry_msgs::Point32 oVel;    // Global velocity of O
  geometry_msgs::Point32 gVelo;   // Velocity of CoM in body frame O
  geometry_msgs::Point32 oVelo;   // Velocity of O in body frame O
  geometry_msgs::Point32 gW;      // Angular velocity of body frame:[dalpha,~,dpsi]
  // Accelerations
  geometry_msgs::Point32 gAcc;    // Global acceleration of CoM
  geometry_msgs::Point32 oAcc;    // Global acceleration of O
  geometry_msgs::Point32 gAcco;   // Acceleration of CoM in O frame
  geometry_msgs::Point32 oAcco;   // Acceleration of O in O frame
  geometry_msgs::Point32 gDW;     // Angular acceleration of CoM in O frame
  
  truck::state_msgs state; // state message
};
struct car_param{
  float r;       // CoM radius
  float m;       // vehicle mass
  float alpha_0; // angle of CoM
  float l;       // Half length of vehicle
  float l_w;     // Half width of the vehicle
  float R;       // Wheel radius
  float h_m;     // height of top marker
  float del_m;   // max steering range
  float w_m;     // motor combined angular velocity max
  float g;       // gravity
  float ix;      // roll moment of inertia
  float iz;      // yaw moment of inertia
  float c_r;     // rear wheel lateral cornering stifness
  float c_s;     // real wheel longitudinal stifness
  float c_f;     // front wheel lingitudinal stifness
};
/********** Global vars *******************************************************/
struct ros_message ros_msgs;
rosbag::Bag bag;
ros::Time ros_time;
float fb[fb_size];
// modes of driving
enum drive_mode{manual, computer, neutral};
// car specs
const struct car_param params{
  .r = 0.309,        // CoM radius
  .m = 9.0,          // vehicle mass
  .alpha_0 = 0.531,  // angle of CoM
  .l = 0.24,         // Half length of vehicle
  .l_w = 0.26625,    // Half width of the vehicle
  .R = 0.101,        // Wheel radius
  .h_m = 0.386,      // height of top marker
  .del_m = 0.349,    // max steering range
  .w_m = 78.0,       // motor combined angular velocity max
  .g = 9.81,         // gravity
  .ix = 0.1,         // roll moment of inertia
  .iz = 0.5,         // yaw moment of inertia
  .c_r = 1.7*9*9.81, // rear wheel lateral cornering stifness
  .c_s = .5*9*9.81,  // real wheel longitudinal stifness
  .c_f = .6*9*9.81   // front wheel lingitudinal stifness  
  };

// Time keeping
double time_past; // previous time
double time_now;  // current time
double duration;  // duration
/****************** Subscriber callback functions *****************************/
// Teensy
void fbCb(const std_msgs::Float32MultiArray &msg)
{for(int i=0;i<fb_size;i++){fb[i] = msg.data[i];}ros_msgs.fb.data = std::vector<float> (fb, fb + fb_size);}
// Set command 
void setcmdCb(const std_msgs::Int32 &msg){ros_msgs.cmdMode = msg;}
// vicon
void viconCb(const geometry_msgs::TransformStamped &msg){ros_msgs.vicon = msg;}
/****************** function declerations *************************************/
float wrap(float angle); // wrap angle to [0, 2*PI]
void update_states();  // updates states calculated from vicon data
void update_desired(); // updates desired states
void record();         // recording rosbag
void open_loop();         // openloop control for testing stuff
// kinematic controller for ground phase
void ground_kinematic_control(std::vector<std::vector<float>> points);
// dynamic controller for ground phase
void ground_dynamic_control(std::vector<std::vector<float>> points);
void control_choice(std::vector<std::vector<float>> points); // Choose controller
/****************** Cllass definitions ****************************************/
class Filter{
  // This is a 1st order RC filter
  public:
    float fc; // Cut off frequency
    float dt; // Sampling time
    Filter(float input_fc, float input_dt){ // Constructor
      fc = input_fc;
      dt = input_dt;
    }
  float filter(float measured){
    // Receives the measured and returns the filterred.
    // Should be run concurrently to perform appropriately.
    static float filterred = measured;
    filterred = filterred/(1+2*M_PI*fc*dt) + measured*2*M_PI*fc*dt/(1+2*M_PI*fc*dt);
    return filterred;
  }
};
// ROS related classes
class Subscribers{// This class holds all subscribers
  public:
    ros::NodeHandle nh;
    Subscribers(ros::NodeHandle &node){nh = node;}
    // Teensy feedbacks
    ros::Subscriber sub_fb = nh.subscribe("truck/fb", 1,&fbCb);
    // Set command mode
    ros::Subscriber sub_setcmd = nh.subscribe("truck/cmd/setcmd", 1,&setcmdCb);
    // vicon subscription
    ros::Subscriber sub_vicon = nh.subscribe("vicon/truck/truck", 1, &viconCb);
    //ros::Subscriber sub_vicon = nh.subscribe("/vicon/CRACK_ROBOT/CRACK_ROBOT", 1, &viconCb);
};
class Publishers{// This class holds all publisherss
  public:
    ros::NodeHandle nh;
    Publishers(ros::NodeHandle &node){nh = node;}
    ros::Publisher pub_time=nh.advertise<std_msgs::Float32> ("truck/time",1);
    // Commanding mode
    ros::Publisher pub_cmdMode=nh.advertise<std_msgs::Int32>
                                            ("truck/cmd/mode",1);
    // Steering command
    ros::Publisher pub_cmdSteer=nh.advertise<std_msgs::Float32>
                                            ("truck/cmd/steer",1);
    // Throttle command
    ros::Publisher pub_cmdThrottle = nh.advertise<std_msgs::Float32>
                                                 ("truck/cmd/throttle", 1);
    // States
    // positions and orientation
    ros::Publisher pub_gPos=nh.advertise<geometry_msgs::Point32>("/state/gpos",1);
    ros::Publisher pub_oPos=nh.advertise<geometry_msgs::Point32>("/state/opos",1);
    ros::Publisher pub_gOrient=nh.advertise<geometry_msgs::Point32>("/state/gorient",1);
    ros::Publisher pub_t1Pos=nh.advertise<geometry_msgs::Point32>("/state/t1pos",1);
    ros::Publisher pub_t2Pos=nh.advertise<geometry_msgs::Point32>("/state/t2pos",1);
    // Velocities
    ros::Publisher pub_gVel=nh.advertise<geometry_msgs::Point32>("/state/gvel",1);
    ros::Publisher pub_gVelo=nh.advertise<geometry_msgs::Point32>("/state/gvelo",1);
    ros::Publisher pub_oVel=nh.advertise<geometry_msgs::Point32>("/state/ovel",1);
    ros::Publisher pub_oVelo=nh.advertise<geometry_msgs::Point32>("/state/ovelo",1);
    ros::Publisher pub_gW=nh.advertise<geometry_msgs::Point32>("/state/gw",1);
    // Accelerations
    ros::Publisher pub_gAcc=nh.advertise<geometry_msgs::Point32>("/state/gacc",1);
    ros::Publisher pub_gAcco=nh.advertise<geometry_msgs::Point32>("/state/gacco",1);
    ros::Publisher pub_oAcc=nh.advertise<geometry_msgs::Point32>("/state/oacc",1);
    ros::Publisher pub_oAcco=nh.advertise<geometry_msgs::Point32>("/state/oacco",1);
    ros::Publisher pub_gDW=nh.advertise<geometry_msgs::Point32>("/state/gdw",1);
   // main publisher
   void publish(){
     // To Teensy
     pub_cmdMode.publish(ros_msgs.cmdMode);
     pub_cmdSteer.publish(ros_msgs.cmdSteer);
     pub_cmdThrottle.publish(ros_msgs.cmdThrottle);
     // Time
     pub_time.publish(ros_msgs.time);
     // State topics
     // positions and orientation
     pub_gPos.publish(ros_msgs.gPos);
     pub_oPos.publish(ros_msgs.oPos);
     pub_gOrient.publish(ros_msgs.gOrient);
     pub_t1Pos.publish(ros_msgs.t1Pos);
     pub_t2Pos.publish(ros_msgs.t2Pos);
     // Velocities
     pub_gVel.publish(ros_msgs.gVel);
     pub_gVelo.publish(ros_msgs.gVelo);
     pub_oVel.publish(ros_msgs.oVel);
     pub_oVelo.publish(ros_msgs.oVelo);
     pub_gW.publish(ros_msgs.gW);
     // Accelerations
     pub_gAcc.publish(ros_msgs.gAcc);
     pub_gAcco.publish(ros_msgs.gAcco);
     pub_oAcc.publish(ros_msgs.oAcc);
     pub_oAcco.publish(ros_msgs.oAcco);
     pub_gDW.publish(ros_msgs.gDW);
   }
};
/****************** Global classes *********************************************/
/****************** Main loop *************************************************/
int main(int argc, char **argv){
  // Initializing ROS node.
  ros::init(argc, argv, "log"); // Name of the node is "log".
  ros::NodeHandle nh;
  ros::Rate rate(50);          // Rate of execution 50Hz.
  // Opening bag file
  bag.open("test.bag", rosbag::bagmode::Write);
  // Initializing subscribers and publishers
  Subscribers subscribe(nh);
  Publishers publish(nh);
  // Setting command mode to neutral
  ros_msgs.cmdMode.data = 0;  //[1 or 2 or other] = [radio or computer or neutral]
  publish.pub_cmdMode.publish(ros_msgs.cmdMode);
  ros::spinOnce();
  // Initialize path points
   // path.points: [X, Y, psir, s]
  PathOval path(0.01,0,0,0);
  // Get the total path tree
  tnode* node_total = get_kd_tree(path.points, 0);
  // Update states
  update_states();
  // Other varibles
  std::vector<float> X = {0,0,0};         // [X,Y,psi]
  std::vector<float> L = {0,0,0,0,0,0};   // [X,Y, psir, k, s, ey], path.points
  float distance;
  // Main while loop of the contro program
  int cmdMode_p = ros_msgs.cmdMode.data;
  int  i = 0;
  while(ros::ok()){
    // Timing
    ros_time = ros::Time::now();
    time_now = round(ros_time.toSec()*1000.0)/1000.0; // Get current time
    duration = time_now - time_past;                   // Get duration
    time_past = time_now;                              // Update previous time
    ros_msgs.time.data = time_now;
    // For latency test
    //ros_msgs.cmdSteer.data = i/5;    i = i+1;
    //publish.pub_cmdSteer.publish(ros_msgs.cmdSteer);
    // Update states
    update_states();
    // Control generation
    //open_loop();
    //ground_kinematic_control(path.points);
    ground_dynamic_control(path.points);
    // ROS_INFO("%+9ld",path.points[1100].size());
    //X = {ros_msgs.gPos.x, ros_msgs.gPos.y, ros_msgs.gOrient.z};   // [X, Y, psi]
    //L = get_nn(path.points,X); // [X, Y, psir, k, s, ey]: path.points
    //ros_msgs.t1Pos.x = L[0]; ros_msgs.t1Pos.y = L[1]; ros_msgs.t1Pos.z = L[2];
    //ros_msgs.t2Pos.x = L[3]; ros_msgs.t2Pos.y = L[4]; ros_msgs.t2Pos.z = L[5];
//    ROS_INFO("************************************************");
   /*
    ROS_INFO("%+07.3f,%+07.3f,%+07.3f", ros_msgs.gPos.x,ros_msgs.gPos.y,ros_msgs.gOrient.z);
    ROS_INFO("%+07.3f,%+07.3f,%+07.3f,%+07.3f,%+07.3f,%+07.3f",
               ros_msgs.t1Pos.x,ros_msgs.t1Pos.y,ros_msgs.t1Pos.z,
               ros_msgs.t2Pos.x,ros_msgs.t2Pos.y,ros_msgs.t2Pos.z);
    */
    //ROS_INFO("I heard: [%20lu]", period);
    
    //ROS_INFO("%+7.3f", ros_msgs.gOrient.z);
    //ROS_INFO("rl: [%+7.3f]", fb[4]);
    /*
    ROS_INFO("%+06.2f %+06.2f",
              path.length,
              (float)path.num_points);
    */
    //ROS_INFO("%+07.3f,%+07.3f,%+07.3f",ros_msgs.cmdSteer.data,fb[12],fb[2]);
    publish.publish();
    record();
    rate.sleep();
    ros::spinOnce();
  }
   
}

/****************** Function definitions **************************************/
// Wrap angle to [0, 2PI]
float wrap(float angle){
  // wraps the angle between [0, 2*pi]
  float w_angle;
  w_angle = remainder(angle, 2*M_PI);
  if (w_angle<0){w_angle += 2*M_PI;}
  return w_angle;
}
// updates states calculated from vicon data
void update_states(){
  // Define local vars
  tf2::Quaternion vicon_quat;
  static double rolld=0, pitchd=0, yawd=0;      // rotations
  static float roll=0, pitch=0, yaw=0;          // rotations
  static float roll_p=0, pitch_p=0, yaw_p=0;    // past rotations
  static float droll=0, dpitch=0, dyaw=0;       // rotation derivatives
  static float droll_p=0, dpitch_p=0, dyaw_p=0; // rotation derivatives
  static float ddroll=0, ddpitch=0, ddyaw=0;    // rotation derivatives
  static float XG=0, YG=0, ZG=0;                // CoM position global frame
  static float XG_p=0, YG_p=0, ZG_p=0;          // CoM past position global frame
  static float XO=0, YO=0, ZO=0;                // O position global frame
  static float XO_p=0, YO_p=0, ZO_p=0;          // O past position global frame
  static float DXG=0, DYG=0, DZG=0;             // CoM velocity global frame
  static float DXG_p=0, DYG_p=0, DZG_p=0;       // CoM past velocity global frame
  static float DDXG = 0, DDYG=0, DDZG=0;        // CoM acceleration global frame
  static float DXO=0, DYO=0, DZO=0;             // O velocity global frame
  static float DXO_p=0, DYO_p=0, DZO_p=0;       // O past velocity global frame
  static float DDXO=0, DDYO=0, DDZO=0;          // O aceleration global frame
  static float dxg=0, dyg=0, dzg=0;             // CoM velocity O frame
  static float ddxg=0, ddyg=0, ddzg=0;          // CoM acceleration O frame
  static float dxo=0, dyo=0, dzo=0;             // O velocity O frame
  static float ddxo=0, ddyo=0, ddzo=0;          // O acceleration O frame
  // Convert geometry_msgs/Quaternios gotten from vicon to tf2::Quaternion
  tf2::convert(ros_msgs.vicon.transform.rotation, vicon_quat);
  // Update orientation angles and store in global vars
  tf2::Matrix3x3(vicon_quat).getRPY(rolld,pitchd,yawd); // alpha, ~, yaw
  roll = (float)rolld;
  pitch = wrap((float)pitchd);
  yaw = wrap((float)yawd);
  ros_msgs.gOrient.x = roll; ros_msgs.gOrient.y = pitch; ros_msgs.gOrient.z = yaw;
  // Calculate angular derivatives and store in global vars
  droll = ros_msgs.fb.data[9];//(roll-roll_p)/duration;
  dpitch = (pitch-pitch_p)/duration;
  dyaw = ros_msgs.fb.data[10];//(yaw-yaw_p)/duration;
  ros_msgs.gW.x = droll; ros_msgs.gW.y = dpitch; ros_msgs.gW.z = dyaw;
  // Calculate angular acceleration
  ddroll = (droll-droll_p)/duration;
  ddpitch = (dpitch-dpitch_p)/duration;
  ddyaw = (dyaw-dyaw_p)/duration;
  ros_msgs.gDW.x = ddroll; ros_msgs.gDW.y = ddpitch; ros_msgs.gDW.z = ddyaw;
  // Update past rotations and its derivatives
  roll_p = roll;
  pitch_p = pitch;
  yaw_p = yaw;
  droll_p = droll;
  dpitch_p = dpitch;
  dyaw_p = dyaw;
  // Update CoM global position
  XG = ros_msgs.vicon.transform.translation.x;
  YG = ros_msgs.vicon.transform.translation.y;
  ZG = ros_msgs.vicon.transform.translation.z;
  ros_msgs.gPos.x = XG; ros_msgs.gPos.y = YG; ros_msgs.gPos.z = ZG;
  // Update O global position, PO = PG - r_G
  XO = (XG + params.r*cos(roll+params.alpha_0)*sin(yaw));
  YO = (YG - params.r*cos(roll+params.alpha_0)*cos(yaw));
  ZO = (-params.r*sin(roll+params.alpha_0));
  ros_msgs.oPos.x = XO; ros_msgs.oPos.y = YO; ros_msgs.oPos.z = ZO;
  // Calculate and update CoM velocities global 
  DXG = (XG-XG_p)/duration;
  DYG = (YG-YG_p)/duration;
  DZG = (ZG-ZG_p)/duration;
  // Filter values for control purposes
  //DXG = filter_DXG.filter(DXG);
  //DYG = filter_DYG.filter(DYG);
  ros_msgs.gVel.x = DXG; ros_msgs.gVel.y = DYG; ros_msgs.gVel.z = DZG;
  // Calculate and update CoM accelerations global 
  DDXG = (DXG-DXG_p)/duration;
  DDYG = (DYG-DYG_p)/duration;
  DDZG = (DZG-DZG_p)/duration;
  ros_msgs.gAcc.x = DDXG; ros_msgs.gAcc.y = DDYG; ros_msgs.gAcc.z = DDZG;
  // Calculate and update O velocities global 
  DXO = (XO-XO_p)/duration;
  DYO = (YO-YO_p)/duration;
  DZO = (ZO-ZO_p)/duration;
  // Filter values for control purposes
  //DXO = filter_DXO.filter(DXO);
  //DYO = filter_DYO.filter(DYO);
  ros_msgs.oVel.x = DXO; ros_msgs.oVel.y = DYO; ros_msgs.oVel.z = DZO;
  // Calculate and update O accelerations global 
  DDXO = (DXO-DXO_p)/duration;
  DDYO = (DYO-DYO_p)/duration;
  DDZO = (DZO-DZO_p)/duration;
  ros_msgs.oAcc.x = DDXO; ros_msgs.oAcc.y = DDYO; ros_msgs.oAcc.z = DDZO;
  // Update CoM and O past positions and its derivatives
  XG_p = XG;
  YG_p = YG;
  ZG_p = ZG;
  XO_p = XO;
  YO_p = YO;
  ZO_p = ZO;
  DXG_p = DXG;
  DYG_p = DYG;
  DZG_p = DZG;
  DXO_p = DXO;
  DYO_p = DYO;
  DZO_p = DZO;
  // Calculate CoM and update velocities and accelerations O frame
  dxg = (DXG*cos(yaw) + DYG*sin(yaw));
  dyg = (-DXG*sin(yaw) + DYG*cos(yaw));
  dzg = DZG;
  ros_msgs.gVelo.x = dxg; ros_msgs.gVelo.y = dyg; ros_msgs.gVelo.z = dzg;
  ddxg = (DDXG*cos(yaw) + DDYG*sin(yaw));
  ddyg = (-DDXG*sin(yaw) + DDYG*cos(yaw));
  ddzg = DDZG;
  ros_msgs.gAcco.x = ddxg; ros_msgs.gAcco.y = ddyg; ros_msgs.gAcco.z = ddzg;
  // Calculate O and update velocities and accelerations in O frame
  dxo = (DXO*cos(yaw) + DYO*sin(yaw));
  dyo = (-DXO*sin(yaw) + DYO*cos(yaw));
  dzo = 0;
  ros_msgs.oVelo.x = dxo; ros_msgs.oVelo.y = dyo; ros_msgs.oVelo.z = dzo;
  ddxo = (DDXO*cos(yaw) + DDYO*sin(yaw));
  ddyo = (-DDXO*sin(yaw) + DDYO*cos(yaw));
  ddzo = 0;
  ros_msgs.oAcco.x = ddxo; ros_msgs.oAcco.y = ddyo; ros_msgs.oAcco.z = ddzo;
}
// openloop control for testing stuff
void open_loop(){
  static float elapsed = 0;
  switch(ros_msgs.cmdMode.data){
  case 2:
    
    ros_msgs.cmdThrottle.data = 0*2/params.R;
    if(fmod(elapsed,2)<1){ros_msgs.cmdSteer.data = 0.3436;}
    else{ros_msgs.cmdSteer.data = -0.346;}
    ROS_INFO("%+07.3f,%+07.3f", ros_msgs.cmdSteer.data, fb[12]);
    
    /*
    ros_msgs.cmdSteer.data = 0.37;
    ros_msgs.cmdThrottle.data = (2.0 + floor(elapsed/5))/params.R;
    ROS_INFO("%+07.3f,%+07.3f", elapsed, ros_msgs.cmdThrottle.data);
    */
    /*
    ros_msgs.cmdThrottle.data = 4/params.w_r;
    ros_msgs.cmdSteer.data = 0.346;
    ROS_INFO("%+07.3f,%+07.3f", elapsed, fmod(elapsed,2));
    */
    elapsed += 0.02;
    if (elapsed>20){ros_msgs.cmdMode.data = 0;}
  break;
  default:
    elapsed = 0;
  break;  
  }
}
// kinematic controller for ground phase
void ground_kinematic_control(std::vector<std::vector<float>> points){
  // Time deactivation
  static float elapsed = 0;
  if(elapsed>20){
    ros_msgs.cmdMode.data = 0;
    elapsed = 0;
  }
  if(ros_msgs.cmdMode.data ==2){elapsed +=.02;}
  
  // vehicle parameters
  float l = params.l;         // vehicle half length
  float R = params.R;         // wheel radius
  
  // velocity control variables
  float u = ros_msgs.gVelo.x; // current longitudinal velocity
  float u0 = u ? u : 0.1;     // current longitudinal velocity modification to avoid zero division
  float ud = 2;               // desired longitudinal velocity
  float erru = u-ud;          // current longitudinal velocity error
  static float err_sum_u = 0; // sum of lingitudinal velocity error
  err_sum_u += err_sum_u;     // updating error sum
  // clamping error sum for anti windup
  err_sum_u = std::max((float) -2,std::min(err_sum_u,(float)2));
  float ku = 10;              // P gain, longitudinal velocity
  float kiu = 0;              // I gain longitudinal velocity
  // Longitudinal velocity control
  float throttle = ud/R - ku*erru - kiu*err_sum_u;
  // clamping the control
  throttle = std::max((float)-80,std::min(throttle,(float)80));
  // Updating ros message value
  ros_msgs.cmdThrottle.data = throttle;
  
  // path tracking and steering vars
  // global position and orientation [X,Y,psi]
  std::vector<float> XG = {ros_msgs.gPos.x, ros_msgs.gPos.y,ros_msgs.gOrient.z};
  // local position with respect to track
  std::vector<float> LG = get_nn(points,XG); // [X, Y, psir, k, s, ey]: path.points
  // Updating corresponding ros message
  ros_msgs.t1Pos.x = LG[0]; ros_msgs.t1Pos.y = LG[1]; ros_msgs.t1Pos.z = LG[2];
  ros_msgs.t2Pos.x = LG[3]; ros_msgs.t2Pos.y = LG[4]; ros_msgs.t2Pos.z = LG[5];
  float e = LG[5];               // current lateral error
  static float ep = e;           // past lateral error
  float de = (e-ep)/duration;    // lateral error rate of change
  ep = e;                  // updating past lateral error
  float psi = XG[2];             // current vehicle body heading
  float psir = LG[2];            // current path heading
  float k = LG[3];               // current path curvature
  float dpsir = u*k/(1-k*e);     // current path heading rate
  float dpsirk = k/(1-k*e);      // current path heading rate divided by u
  float d = 0.8;                 // look ahead distance
  float el = e+d*sin(psi-psir);  // current look ahead error
  static float elp = el;         // past look ahead lateral error
  float del = (el-elp)/duration; // look ahead error rate of change
  elp = el;                      // updating past look ahead error
  float ks = 10;                  // lateral control P gain
  float kds = 0.75;             // lateral error control d gain
  // Lateral error control
  float steer = 2*l/(ud*ud)*(- ks*el - kds*del) + 2*l*dpsirk; // feedback linearization
  //float steer = 2*l*dpsirk - ks*el - kds*del;               // feedforward plus feedforward
  // Clamping steering values
  steer = std::max((float)-0.3436,std::min(steer,(float)0.3436));
  // Update ros message
  ros_msgs.cmdSteer.data = steer;
  ROS_INFO("%+06.2f,%+06.2f", 2*l*dpsirk, 2*l/(ud*ud)*(-ks*el - kds*del));
}

// dynamic controller for ground phase
void ground_dynamic_control(std::vector<std::vector<float>> points){
  // timing of activation
  static float elapsed = 0;
  if(elapsed>20){
    ros_msgs.cmdMode.data = 0;
    elapsed = 0;
  }
  if(ros_msgs.cmdMode.data ==2){elapsed +=.02;}
  // parameters of car
  float R = params.R;
  float m = params.m;
  float l = params.l;
  float iz = params.iz;
  float c_r = params.c_r;
  float c_s = params.c_s;
  float c_f = params.c_f;
  // velocity control variables
  float u = ros_msgs.gVelo.x;         // current longitudinal velocity
  float v = ros_msgs.gVelo.y;         // current lateral velocity
  float dpsi = ros_msgs.gW.z;    // current vehicle body heading
  float u0 = u<1? 1:u;                // modified longitudinal velocity to avoid zero division
  float ud = 1.8;                     // desired velocity
  float eu = u - ud;                  // velocity error
  static float eus = 0;               // sum of velocity error
  eus += eu;                          // Updating error sum
  eus = std::max((float)-2,std::min(eus,(float)2)); // Clamping error summation, anti wind up action
  // longitudinal velocity control
  float ku = 0;                          // P gain  
  float kiu = 0;                         // I gain
  float throttle;                        // command
  throttle = 1.12*ud/R - ku*eu -kiu*eus; 
  // Clamping command
  throttle = std::max((float)-80,std::min(throttle,(float)80));
  // Update ros message
  ros_msgs.cmdThrottle.data = throttle;
  // Lateral control
  // global position and orientation [X,Y,psi]
  std::vector<float> XG = {ros_msgs.gPos.x, ros_msgs.gPos.y,ros_msgs.gOrient.z};
  // local position with respect to track
  std::vector<float> LG = get_nn(points,XG); // [X, Y, psir, k, s, ey]: path.points
  // Updating corresponding ros message
  ros_msgs.t1Pos.x = LG[0]; ros_msgs.t1Pos.y = LG[1]; ros_msgs.t1Pos.z = LG[2];
  ros_msgs.t2Pos.x = LG[3]; ros_msgs.t2Pos.y = LG[4]; ros_msgs.t2Pos.z = LG[5];
  // states
  float d = 0.8;                    // look ahead error
  float e = LG[5];                  // current lateral error
  static float ep = e;              // past lateral error
  float de = (e - ep)/duration;     // current lateral error rate
  ep = e;                           // updating past lateral error
  float psi = XG[2];                // current vehicle body heading
  float psir = LG[2];               // current path heading
  float el = e + d*sin(psi - psir); // current look ahead error
  static float elp = el;            // past look ahead error
  float del = (el-elp)/duration;    // look ahead error rate
  elp = el;                         // Updating past look ahead error
  float k = LG[3];                  // path curvature
  float dpsir = u*k/(1-k*e);        // path heading rate
  // Lateral control
  float ks =2;
  float kds = 0;
  float steer;
  steer = -ks*el - kds*del + (m*iz/(c_f*(iz+m*d*l)))*
         ( c_f*(1/m+d*l/iz)*((v+l*dpsi)/u0)
          +c_r*(1/m-d*l/iz)*((v-l*dpsi)/u0) + u*dpsir);
  //steer = -ks*el - kds*del;
  // Clamping steering values
  steer = std::max((float)-0.3436,std::min(steer,(float)0.3436));
  // Update ros message
  ros_msgs.cmdSteer.data = steer;
  ROS_INFO("%+07.3f,%+07.3f", -ks*el - kds*del, (m*iz/(c_f*(iz+m*d*l)))*
         ( c_f*(1/m+d*l/iz)*((v+l*dpsi)/u0)
          +c_r*(1/m-d*l/iz)*((v-l*dpsi)/u0) + u*dpsir));
  
}

// geometric controller for flight phase
void flight_geometric_control(std::vector<std::vector<float>> points){
  // activation flag
  static bool flag = true;
  // timing of activation
  static float elapsed = 0;
  if(elapsed>20){
    ros_msgs.cmdMode.data = 0;
    elapsed = 0;
    flag = true;
  }
  // parameters of car
  float g = params.g;
  float R = params.R;
  float m = params.m;
  float l = params.l;
  float iz = params.iz;
  float ix = params.ix;
  float alpha_0 = params.alpha_0;
  float r = params.r;
  float c_r = params.c_r;
  float c_s = params.c_s;
  float c_f = params.c_f;
  // states for point o
  float steerfb = ros_msgs.fb.data[4]; // current steering angle
  float u = ros_msgs.oVelo.x; // current longitudinal velocity
  float us = u;               // velocity to be fed into steering control
  float v = ros_msgs.gVelo.y; // current lateral velocity
  float dpsi = ros_msgs.gW.z; // current yaw rate
  float alpha = ros_msgs.gOrient.x; // current roll
  float dalpha = ros_msgs.gW.x;     // current roll rate
  float e_alpha;                    // roll error
  // desired 
  static float ud = 2;     // desired velocity
  float ue = u - ud;       // velocity error
  static float ue_sum = 0; // sum of velocity error
  ue_sum += ue;            // updating sum of error
  ue_sum = std::max((float) 2,std::min(ue_sum,(float)2));
  // path tracking states
  // getting point O data with respect to path
  std::vector<float> X = {0,0,0};         // [X,Y,psi]
  std::vector<float> XP = {0,0,0};        // look ahead point
  std::vector<float> L = {0,0,0,0,0,0};   // [X,Y, psir, k, s, ey], path.points
  static  std::vector<float> L_p = {0,0,0,0,0,0}; // past [X,Y, psir, k, s, ey], path.points
  X = {ros_msgs.oPos.x, ros_msgs.oPos.y, ros_msgs.gOrient.z};   // [X, Y, psi]
  L = get_nn(points,X); // [X, Y, psir, k, s, ey]: path.points
  ros_msgs.t1Pos.x = L[0]; ros_msgs.t1Pos.y = L[1]; ros_msgs.t1Pos.z = L[2];
  ros_msgs.t2Pos.x = L[3]; ros_msgs.t2Pos.y = L[4]; ros_msgs.t2Pos.z = L[5];
  // path vars
  float ey = L[5];                      // current lateral error
  static float ey_p = ey;               // past lateral error
  float d = 0.6;                        // look ahead distance (c_f + c_r)/(2/k)
  float el = ey + d*sin(X[2] - L[2]);   // look ahead error
  static float el_p = el;               // past look ahead error
  float psir = L[2];                    // path heading
  static float psir_p = psir;           // past path heading
  float k = L[3];                       // current path curvature
  float dpsir = 1-k*ey ? k*u/(1-k*ey): 0; // current path heading rate
  float dey = (ey - ey_p)/duration;     // current lateral error rate
  float del = (el-el_p)/duration;       // current lateral error of look ahead point rate
  ey_p = ey;
  el_p = el;
  L_p = L;
  // roll desired
  float alpha_e;
  // commands
  float throttle;
  float steer;
  // quasi command
  float steer_e = 0;
  // longitudinal velocity control parameters
  float ku = 5;
  // lateral control parameters
  float ke = 2;
  float kde = 0;
  // roll control parameters
  float ka = 2;
  float kda = 0;
  // roll and lateral control
  if(us<.5){us = .5;}
  // Simple geometric model
  // quasi lateral control
  steer_e = -ke*el-kde*del + u*dpsir;
  steer_e = std::max((float)-0.3436*u*u/(2*l*cos(alpha)),
                     std::min(steer_e,(float)0.3436*u*u/(2*l*cos(alpha)))
                    );
  // bem calculation
  alpha_e = steer_e ? atan(g/steer_e) - alpha_0: M_PI/2-alpha_0;
  e_alpha = alpha - alpha_e;
  // main control
  steer = 2*l*cos(alpha)*(ix + m*r*r)/(m*r*sin(alpha + alpha_0)*us*us)*(
    m*r*g*cos(alpha + alpha_0)-ka*e_alpha-kda*dalpha);
  steer = std::max((float)-0.3436,std::min(steer,(float)0.3436));
  // Update ros message
  ros_msgs.cmdSteer.data = steer;
  
  // velocity control
  // simple P control based on linear dynamics
  throttle = ud/R - ku*(u-ud);// - kd_u*du;
  // saturate values
  throttle = std::max((float)-80,std::min(throttle,(float)80));
  // Update ros message
  ros_msgs.cmdThrottle.data = throttle;
  if(ros_msgs.cmdMode.data ==2){elapsed +=.02;}
  // activation flag
  if(flag && ros_msgs.cmdMode.data ==2){
    if(L[4] >0.5 && L[4]<1){flag = false;}
  }
  if(flag){ros_msgs.cmdThrottle.data = 5;}
}

// rosbag record
void record(){
  static  int cmdMode_p = 0;
  switch(ros_msgs.cmdMode.data){
  case 2:
    // Feedback from Teensy
    bag.write("truck/fb", ros_time, ros_msgs.fb);
    // Commands to Teensy
    bag.write("truck/cmd/mode", ros_time, ros_msgs.cmdMode);
    bag.write("truck/cmd/steer", ros_time, ros_msgs.cmdSteer);
    bag.write("truck/cmd/throttle", ros_time, ros_msgs.cmdThrottle);
    // Vicon data
    bag.write("truck/vicon", ros_time, ros_msgs.vicon);
    
    // States
    // positions and orientation
    bag.write("state/gpos", ros_time, ros_msgs.gPos);
    bag.write("state/opos", ros_time, ros_msgs.oPos);
    bag.write("state/gorient", ros_time, ros_msgs.gOrient);
    bag.write("state/t1pos", ros_time, ros_msgs.t1Pos);
    bag.write("state/t2pos", ros_time, ros_msgs.t2Pos);
    // Velocities
    bag.write("state/gvel", ros_time, ros_msgs.gVel);
    bag.write("state/gvelo", ros_time, ros_msgs.gVelo);
    bag.write("state/ovel", ros_time, ros_msgs.oVel);
    bag.write("state/ovelo", ros_time, ros_msgs.oVelo);
    bag.write("state/gw", ros_time, ros_msgs.gW);
    // Accelerations
    bag.write("state/gacc", ros_time, ros_msgs.gAcc);
    bag.write("state/gacco", ros_time, ros_msgs.gAcco);
    bag.write("state/oacc", ros_time, ros_msgs.oAcc);
    bag.write("state/oacco", ros_time, ros_msgs.oAcco);
    bag.write("state/gdw", ros_time, ros_msgs.gDW);
  break;
  
  }
}
