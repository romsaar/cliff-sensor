/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */

#ifndef TURTLEBOT3_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h> // RAN
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include "PayloadConfig.h"

#include <TurtleBot3.h>
#include "turtlebot3_burger.h"
#include <Servo.h>


#include <math.h>

#define FIRMWARE_VER "1.2.3"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define PAYLOAD_SPEED_FREQUENCY                30   //hz
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2

// define PWM pins
#define PWM_PIN_PUMP                     3  // paint pump pin
#define PWM_PIN_BUFFER                   6  // buffer pin
#define BUFFER_OFF_FILTER                3  // [seconds]
#define PUMP_IGNITION_TIME               5  // [seconds]
#define PUMP_CYCLE_ON_TIME               2  // [seconds]
#define PUMP_CYCLE_OFF_TIME              3  // [seconds] 

// define servo pin
#define HVLP1_SERVO_PIN                   9 // spray servo pin tessa
#define HVLP1_CLOSE_POS                   100 // closed position angle (deg)
#define HVLP1_OPEN_POS                    40 // open position angle (deg)
#define HVLP2_SERVO_PIN                   10 // spray servo pin no tessa
#define HVLP2_CLOSE_POS                   120 // closed position angle (deg)
#define HVLP2_OPEN_POS                    70 // open position angle (deg)

// define cliff IR sensor
#define sensorR A0 // Sharp IR Right
#define sensorL A1 // sharp IR Left


// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void soundCallback(const turtlebot3_msgs::Sound& sound_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);
//void pwm3Callback(const std_msgs::Int32& pwm_rqt_msg);
//void pumpPWMCallback(const std_msgs::Int32& pump_pwm_msg);
void servoCallback(const std_msgs::Int32& servo_angle_msg);
void bufferPWMCallback(const std_msgs::Int32& buffer_pwm_msg);
void payloadModeCallback(const std_msgs::Bool& pld_auto_msg);  // 0 = manual, 1 = auto
void payloadPowerCallback(const std_msgs::Bool& pld_power_msg); // 0 = OFF, 1 = ON
void payloadConfigCallback(const turtlebot3_msgs::PayloadConfig& pld_config_msg); 
void payloadFreqCallback(const std_msgs::Int32& pld_freq_msg);  // 1 - 1000Hz

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void payloadHandler(void);
//void setPumpPWM(int32_t pumpPWM);
void setServo1Angle(int32_t servo1Angle);
void setServo2Angle(int32_t servo2Angle);
void setBufferPWM(int32_t bufferPWM);
void CliffIRSensor(void);

void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<turtlebot3_msgs::Sound> sound_sub("sound", soundCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

//ros::Subscriber<std_msgs::Int32> pump_request_sub("pump_pwm_request", pumpPWMCallback);

ros::Subscriber<std_msgs::Int32> servo_request_sub("servo_angle_request", servoCallback);

ros::Subscriber<std_msgs::Int32> buffer_request_sub("buffer_pwm_request", bufferPWMCallback); 

ros::Subscriber<std_msgs::Bool> payload_power_sub("payload_power", payloadPowerCallback);

ros::Subscriber<std_msgs::Bool> payload_auto_sub("payload_auto", payloadModeCallback);

ros::Subscriber<std_msgs::Int32> payload_freq_sub("payload_freq", payloadFreqCallback);

ros::Subscriber<turtlebot3_msgs::PayloadConfig> payload_config_sub("payload_config", payloadConfigCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

// RAN - My voltage 
std_msgs::Float32 voltage_msg;
ros::Publisher pub_voltage("voltage", &voltage_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
Turtlebot3Controller controllers;
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

/*******************************************************************************
* Declaration for payload
*******************************************************************************/
bool payload_power      = true;  // Power = OFF
bool payload_auto_mode  = true;   // Mode = Auto
int32_t goal_pump_pwm = 220;      // ?? 
int32_t goal_buffer_pwm = 250;    // 8.4v
int32_t current_pump_pwm = 0;
int32_t current_buffer_pwm = 0;
int8_t buffer_off_filter = 0;
int32_t goal_payload_freq = 20;         // buffer PWM frequency [Hz]
int32_t current_payload_freq = 20;      // buffer PWM frequency [Hz]
int32_t buffer_filter_th = 30;          // init th with 1 second
int32_t buffer_filter_counter = 0;
int32_t pump_cycle_counter = 0;
uint8_t pump_ignition_time = PUMP_IGNITION_TIME;
uint8_t pump_cycle_on_time = PUMP_CYCLE_ON_TIME;
uint8_t pump_cycle_off_time = PUMP_CYCLE_OFF_TIME;

/*******************************************************************************
* Declaration for HVLP servo
*******************************************************************************/
Servo HVLP1_servo;  // create servo object to control the HVLP servo
Servo HVLP2_servo;  // create servo object to control the HVLP servo
uint8_t servo1_angle = 0;
int32_t goal_servo1_angle = HVLP1_CLOSE_POS; 
int32_t current_servo1_angle = HVLP1_CLOSE_POS;  
uint8_t servo2_angle = 0;
int32_t goal_servo2_angle = HVLP2_CLOSE_POS; 
int32_t current_servo2_angle = HVLP2_CLOSE_POS;   
/*******************************************************************************
* IR cliff sensor
*******************************************************************************/
float ARR = 0.0;// value from right sensor 
bool right_IR = false;
float ARL = 0.0;// value from left sensor 
bool left_IR = false; 

#endif // TURTLEBOT3_CORE_CONFIG_H_
