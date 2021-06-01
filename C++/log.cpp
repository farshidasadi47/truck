/*===========================================================================*/
// This program tests data logging of teensy node from USB.
// Farshid Asadi
/*========= Header files ====================================================*/
#include <ros/ros.h>          // ROS.
#include <sensor_msgs/Imu.h>  // ROS IMU message type.
#include <std_msgs/Float32.h> // ROS Float32 type
#include <std_msgs/Int32.h> // ROS Int32 type
/*========= Struct defs =====================================================*/
struct ros_message{
  std_msgs::Float32 fl;          // front left wheel velocity
  std_msgs::Float32 fr;          // front right
  std_msgs::Float32 rl;          // rear left
  std_msgs::Float32 rr;          // rear right
  std_msgs::Float32 steerFB;     // FB: Feedback
  std_msgs::Float32 recSteer;    // rec: receiver
  std_msgs::Float32 recThrottle;
  std_msgs::Float32 cmdSteer;    // cmd: command
  std_msgs::Float32 cmdThrottle;
  sensor_msgs::Imu imu;
  std_msgs::Int32 ltime;
};
/*========= Global vars =====================================================*/
struct ros_message mcu_msgs;
// modes of driving
enum drive_mode{manual, computer, neutral};
/*========= Functions =======================================================*/
// Wheel velocity subscribers
void flCb(const std_msgs::Float32 &msg){mcu_msgs.fl = msg;}
void frCb(const std_msgs::Float32 &msg){mcu_msgs.fr = msg;}
void rlCb(const std_msgs::Float32 &msg){mcu_msgs.rl = msg;}
void rrCb(const std_msgs::Float32 &msg){mcu_msgs.rr = msg;}
void steerFBCb(const std_msgs::Float32 &msg){mcu_msgs.steerFB = msg;}
void recSteerCb(const std_msgs::Float32 &msg){mcu_msgs.recSteer = msg;}
void recThrottleCb(const std_msgs::Float32 &msg){mcu_msgs.recThrottle = msg;}
void imuCb(const sensor_msgs::Imu &msg){mcu_msgs.imu = msg;}
void ltimeCb(const std_msgs::Int32 &msg){mcu_msgs.ltime = msg;}
/*========= Classes =========================================================*/
class Subscribers{// This class holds all subscribers
  public:
    ros::NodeHandle nh;
    Subscribers(ros::NodeHandle &node){nh = node;}
    // Wheel velocities
    ros::Subscriber sub_fl = nh.subscribe("truck/fb/wheel/fl", 1,&flCb);
    ros::Subscriber sub_fr = nh.subscribe("truck/fb/wheel/fr", 1,&frCb);
    ros::Subscriber sub_rl = nh.subscribe("truck/fb/wheel/rl", 1,&rlCb);
    ros::Subscriber sub_rr = nh.subscribe("truck/fb/wheel/rr", 1,&rrCb);
    // PWM sensory feedback
    ros::Subscriber sub_steerFB=nh.subscribe("truck/fb/steer",1,&steerFBCb);
    ros::Subscriber sub_recSteer=nh.subscribe("truck/fb/rec/steer",
                                               1,&recSteerCb);
    ros::Subscriber sub_recThrottle = nh.subscribe("truck/fb/rec/steer",
                                                    1,&recThrottleCb);  
    // IMU data
    ros::Subscriber sub_imu=nh.subscribe("truck/fb/imu",1,&imuCb);
    // Timing data
    ros::Subscriber sub_ltime = nh.subscribe("truck/fb/ltime", 1,&ltimeCb);
};

class Publishers{// This class holds all publisherss
  public:
    ros::NodeHandle nh;
    Publishers(ros::NodeHandle &node){nh = node;}
    // Steering command
    ros::Publisher pub_cmdSteer=nh.advertise<std_msgs::Float32>
                                            ("truck/cmd/steer",1);
    // Throttle command
    ros::Publisher pub_cmdThrottle = nh.advertise<std_msgs::Float32>
                                                 ("truck/cmd/throttle", 1);
   // main publisher
   void publish(){
     pub_cmdSteer.publish(mcu_msgs.cmdSteer);
     pub_cmdThrottle.publish(mcu_msgs.cmdThrottle);
   }
    

};


int main(int argc, char **argv){
  // Initializing ROS node.
  ros::init(argc, argv, "log"); // Name of the node is "log".
  ros::NodeHandle nh;
  ros::Rate rate(100);          // Rate of execution 100Hz.
  // Initializing subscribers and publishers
  Subscribers subscribe(nh);
  Publishers publish(nh);
  ros::spinOnce();
  long told = mcu_msgs.ltime.data; // car_sens.imu.header.stamp.nsec/1000000;
  long tnew;
  long period;
  int i  = 0;
  while(ros::ok()){
    // Read topics
    tnew = mcu_msgs.ltime.data; // car_sens.imu.header.stamp.nsec/1000000;
    period = tnew - told;
    told = tnew;
//    ROS_INFO("I heard: [%20lu]", period);
    /*ROS_INFO("%+06.2f %+06.2f %+06.2f",
              mcu_msgs.imu.orientation.x,
              mcu_msgs.imu.orientation.y,
              mcu_msgs.imu.orientation.z);*/
    
    ROS_INFO("%+06.2f %+06.2f %+06.2f",
              mcu_msgs.imu.angular_velocity.x,
              mcu_msgs.imu.angular_velocity.y,
              mcu_msgs.imu.angular_velocity.z);
    ros::spinOnce();
    // Publish topics
    //mcu_msgs.cmdSteer.data = 1.5 + i;
    //publish.publish();
    i++;
    rate.sleep();
  }
  
  
  
}
