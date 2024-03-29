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

#include "turtlebot3_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);
  //nh.subscribe(pump_request_sub); 
  //nh.subscribe(buffer_request_sub);
  nh.subscribe(servo_request_sub);  
  nh.subscribe(payload_power_sub); 
  nh.subscribe(payload_auto_sub); 
  nh.subscribe(payload_freq_sub);   
  nh.subscribe(payload_config_sub);

  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  nh.advertise(pub_voltage);  // RAN

  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  motor_driver.init(NAME);

  // Setting for IMU
  sensors.init();

  // Init diagnosis
  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);
    
  // Initialize Aduino PWM pins - RAN
  // analogWrite(3, 0);
  
  // Setting buffer pin (6) to 400Hz and set to zero
  drv_pwm_set_freq(PWM_PIN_BUFFER, current_payload_freq); //was 400
  drv_pwm_setup(PWM_PIN_BUFFER);  
  drv_pwm_set_duty(PWM_PIN_BUFFER, 10, 0);

  // Setting pump pin (3) to 400Hz and set to zero
  //drv_pwm_set_freq(PWM_PIN_PUMP, current_payload_freq); //was 400
  //drv_pwm_setup(PWM_PIN_PUMP);  
  //drv_pwm_set_duty(PWM_PIN_PUMP, 10, 0);

  buffer_filter_th = PAYLOAD_SPEED_FREQUENCY * BUFFER_OFF_FILTER;

  // Setting the HVLP1 servo
  HVLP1_servo.attach(HVLP1_SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  HVLP1_servo.write(HVLP1_CLOSE_POS);
  // Setting the HVLP2 servo
  HVLP2_servo.attach(HVLP2_SERVO_PIN);  // attaches the servo on pin 8 to the servo object
  HVLP2_servo.write(HVLP2_CLOSE_POS);
  // temporary debug code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Serial.begin(9600);  
  Serial.println("--- Start Serial Monitor SEND_RCVE ---"); 
  
  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    tTime[0] = t;
  }

  if ((t-tTime[6]) >= (1000 / PAYLOAD_SPEED_FREQUENCY)) // RAN - payload handler
  {
    //updateGoalVelocity();
    payloadHandler();    
    tTime[6] = t;
  }
  

  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
  {
    publishCmdVelFromRC100Msg();
    tTime[1] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // Send log message after ROS connection
  sendLogMsg();

  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100);

  // Check push button pressed for simple test drive
  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  //goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[LINEAR]  = -cmd_vel_msg.linear.x;    // RAN (11-05-19) - temporary fix to manual controller !!!!
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

/*******************************************************************************
* Callback function for sound msg
*******************************************************************************/
void soundCallback(const turtlebot3_msgs::Sound& sound_msg)
{
  sensors.makeSound(sound_msg.value);
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

  motor_driver.setTorque(dxl_power);
}



/*******************************************************************************
* Callback function for payload configuration msg - RAN
*******************************************************************************/
void payloadConfigCallback(const turtlebot3_msgs::PayloadConfig& pld_config_msg)
{

  char log_msg[50];
  char log_msg1[50];   
  
  payload_power = false;    
  sprintf(log_msg, "payload power has changed to OFF");
  nh.loginfo(log_msg); 
    
  goal_pump_pwm = pld_config_msg.pump_pwm;
  goal_buffer_pwm = pld_config_msg.buffer_pwm;  
  pump_ignition_time = pld_config_msg.pump_priming_time;
  pump_cycle_on_time = pld_config_msg.pump_on_time;
  pump_cycle_off_time = pld_config_msg.pump_off_time;
  
  sprintf(log_msg1, "Setting PWM: pump goal %i, buffer goal %i", goal_pump_pwm, goal_buffer_pwm);
  nh.loginfo(log_msg1);
}

/*******************************************************************************
* Callback function for PUMP pwm_request msg - RAN
*******************************************************************************/
/*
void pumpPWMCallback(const std_msgs::Int32& pump_pwm_msg)
{
  int32_t pwm_request = pump_pwm_msg.data;
  char log_msg[50];

  if (pwm_request<0)
    pwm_request = 0; 

  goal_pump_pwm = pwm_request;  
  sprintf(log_msg, "Setting pump PWM goal to %i", goal_pump_pwm);
  nh.loginfo(log_msg);
}

/*******************************************************************************
* Callback function for servo angle_request msg - RAN
*******************************************************************************/
void servoCallback(const std_msgs::Int32& servo_angle_msg)
{
  int32_t angle_request = servo_angle_msg.data;
  char log_msg[50];

  goal_servo1_angle = angle_request % 180;
  goal_servo2_angle = angle_request % 180;  
  sprintf(log_msg, "servoCallback: setting goal to %i", goal_servo1_angle);
  nh.loginfo(log_msg);
}


/*******************************************************************************
* Callback function for payloadfrequencyt msg - RAN
*******************************************************************************/

void payloadFreqCallback(const std_msgs::Int32& pld_freq_msg)
{
  int32_t freq_request = pld_freq_msg.data;
  char log_msg[50];

  goal_payload_freq = freq_request;
  sprintf(log_msg, "Setting payload frequency goal to %i", goal_payload_freq);
  nh.loginfo(log_msg);
}


/*******************************************************************************
* Callback function for payload auto mode request msg
*******************************************************************************/

void payloadModeCallback(const std_msgs::Bool& pld_auto_msg)
{
  bool pld_auto_mode = pld_auto_msg.data;
  char log_msg[50];

  if (pld_auto_mode)
  {
    // switch payload auto mode to ON
    payload_auto_mode = true;     
    sprintf(log_msg, "Payload state = auto");
  }
  else
  {
    // switch payload auto mode to ON
    payload_auto_mode = false;    
    sprintf(log_msg, "Payload state = manual, HVLP shut OFF");
    
  }
  goal_servo1_angle = HVLP1_CLOSE_POS; 
  goal_servo2_angle = HVLP2_CLOSE_POS; 
  nh.loginfo(log_msg); 
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelFromRC100Msg(void)
{
  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  else
    return;

  sensor_state_msg.bumper = sensors.checkPushBumper();

  sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  sensor_state_msg.illumination = sensors.getIlluminationData();
  
  sensor_state_msg.button = sensors.checkPushButton();

  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
* Turtlebot3 test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  motor_driver.readEncoder(current_tick[LEFT], current_tick[RIGHT]);

  if (buttons & (1<<0))  
  {
    move[LINEAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
  }

  if (move[LINEAR])
  {    
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {   
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= -0.7;
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro - RAN!!!!!!");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End - RAN!!!!!");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
   
  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}
/*******************************************************************************
* Cliff IR Sensor
*******************************************************************************/
 void CliffIRSensor(void)
 {
 ARR = analogRead(sensorR);// value from right sensor 
 ARL = analogRead(sensorL);// value from left sensor 

    //Serial.print("right analog read ");
    //Serial.println(ARR);
    //Serial.print("left analog read ");
    //Serial.println(ARL);
   
   if (ARR >= 400) // 7cm value
    right_IR=false; // right ir sensor find deck
    
    else
    right_IR=true; // right ir sensor find cliff
    
    if (ARL >= 400)  // 7cm value
    left_IR=false; // left ir sensor find deck
    
    else
    left_IR=true; // left ir sensor find cliff
 }
/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  if (right_IR && left_IR)
  {
    
    goal_velocity[ANGULAR] = 0.0;
    if (goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR] >= 0.0)
     goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
    else
     goal_velocity[LINEAR]  = 0.0;
  }
  else
  {
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}
}

/*******************************************************************************
* Set paint pump PWM 
*******************************************************************************/

/*
void setPumpPWM(int32_t pumpPWM)
{
  char log_msg[50];
  drv_pwm_set_duty(PWM_PIN_PUMP, 10, pumpPWM);
  current_pump_pwm = pumpPWM;  
  sprintf(log_msg, "Setting pump PWM to %i", pumpPWM);
  nh.loginfo(log_msg);
}
*/

/*******************************************************************************
* Set HVLP servo angle 
*******************************************************************************/

void setServo1Angle(int32_t servo1Angle)
{
  char log_msg[50];  
  HVLP1_servo.write(servo1Angle);
  current_servo1_angle = servo1Angle;  
  sprintf(log_msg, "Setting HVLP1 servo angle to %i", servo1Angle);  
  nh.loginfo(log_msg);
}

void setServo2Angle(int32_t servo2Angle)
{
  char log_msg[50];  
  HVLP2_servo.write(servo2Angle);
  current_servo2_angle = servo2Angle;  
  sprintf(log_msg, "Setting HVLP2 servo angle to %i", servo2Angle);  
  nh.loginfo(log_msg);
}
/*******************************************************************************
* Set buffer PWM 
*******************************************************************************/

void setBufferPWM(int32_t bufferPWM)
{
  char log_msg[50];
  drv_pwm_set_duty(PWM_PIN_BUFFER, 10, bufferPWM);
  current_buffer_pwm = bufferPWM;
  sprintf(log_msg, "Setting buffer PWM to %i", bufferPWM);
  nh.loginfo(log_msg); 
}

/*******************************************************************************
* Payload handler
*******************************************************************************/

void payloadHandler(void)
{
  char log_msg[50];  
  int32_t actual_angle1_goal = HVLP1_CLOSE_POS;
  int32_t actual_angle2_goal = HVLP2_CLOSE_POS;
  
  sprintf(log_msg, "Payload handler:");

  // Control servo angle
  //Serial.print("Set servo to: ");
  //Serial.println(servo_angle);
  //servo_angle = (servo_angle + 1) % 180;
  //HVLP_servo.write(servo_angle);
         
  if (payload_auto_mode)
  {
    // Handle Auto mode
    // ----------------
    if (goal_velocity[LINEAR] < 0)
    {        
      // Set servo angle goal to OPEN
      actual_angle1_goal = HVLP1_OPEN_POS;   
      actual_angle2_goal = HVLP2_OPEN_POS; 
      sprintf(log_msg,"auto mode - open");
    }
    else
    {
      // No linear velocity -> shut down HVLP sparyer       
      actual_angle1_goal = HVLP1_CLOSE_POS;
      actual_angle2_goal = HVLP2_CLOSE_POS;
      sprintf(log_msg,"auto mode - close");
    }     
  }
  else
  {
    // Handle Manual mode 
    // ------------------      
    actual_angle1_goal = goal_servo1_angle;  
    actual_angle2_goal = goal_servo2_angle;
    sprintf(log_msg,"manual mode");      
  }
  

  //actual_angle_goal = HVLP_OPEN_POS;
   
  // set servo angle to goal   
  if (current_servo1_angle != actual_angle1_goal)
     setServo1Angle(actual_angle1_goal);
  if (current_servo2_angle != actual_angle2_goal)
     setServo2Angle(actual_angle2_goal);      
}


/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);
  
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  int32_t encoder[WHEEL_NUM] = {0, 0};
  motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);
  
  DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("TurtleBot3");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");   
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}
