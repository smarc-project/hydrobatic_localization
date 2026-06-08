#include "hydrobatic_localization/state_estimator.h"


StateEstimator::StateEstimator()
  : Node("state_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    tf_broadcast_(this), number_of_imu_measurements(0), is_graph_initialized_(false),
    new_dvl_measurement_(false), new_gps_measurement_(false), map_initialized_(false),
    first_barometer_measurement_(0.0), new_barometer_measurement_received_(false), atmospheric_pressure_(100800.0),
    dt_(0.02)
{

  // Declare parameters
  this->declare_parameter<bool>("use_motion_model", true);
  this->get_parameter("use_motion_model", using_motion_model_);

  this->declare_parameter<std::string>("inference_strategy","FixedLagSmoothing");
  this->get_parameter("inference_strategy", inference_strategy_);

  this->declare_parameter<bool>("init_from_ground_truth", true);
  this->get_parameter("init_from_ground_truth", init_from_ground_truth_);

  this->declare_parameter<std::string>("config_file", "sam.yaml");
  this->get_parameter("config_file", config_file_);

  this->declare_parameter<int>("kf_interval_hz", 10);
  this->get_parameter("kf_interval_hz", kf_interval_hz_);
  bool use_sim_time_;
  this->get_parameter("use_sim_time", use_sim_time_);
  this->declare_parameter<bool>("use_sensor_covariance", false);
  this->get_parameter("use_sensor_covariance", use_sensor_covariance_);



  std::string config_file;
  if (std::filesystem::path(config_file_).is_absolute()) {
    config_file = config_file_;
  } else {
    auto pkg_share = ament_index_cpp::get_package_share_directory("hydrobatic_localization");
    config_file = pkg_share + "/config/" + config_file_;
  }

  RCLCPP_INFO(this->get_logger(), "Loading config from %s", config_file.c_str());
  //logg the ros parameters
  RCLCPP_INFO(this->get_logger(), "Using motion model: %s", using_motion_model_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Init from ground truth: %s", init_from_ground_truth_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Use sensor covariance: %s", use_sensor_covariance_ ? "true" : "false");
  name_space_ = this->get_namespace();
  //remove leading slashes from namespace
  if (name_space_.front() == '/') {
    name_space_.erase(0, 1);
  }
  std::cout << "Namespace: " << name_space_ << std::endl;
  InferenceStrategy inference_strategy;
  if(inference_strategy_ == "ISAM2"){
    inference_strategy = InferenceStrategy::ISAM2;
  }
  else if(inference_strategy_ == "FixedLagSmoothing"){
    inference_strategy = InferenceStrategy::FixedLagSmoothing;
  }
  else if (inference_strategy_ == "EKF") {
    inference_strategy = InferenceStrategy::EKF;
  }
  else if (inference_strategy_ == "FullSmoothing") {
    inference_strategy = InferenceStrategy::FullSmoothing;
  }
  else {
    throw std::invalid_argument("Invalid inference strategy, choose between ISAM2, FixedLagSmoothing, EKF or FullSmoothing");
  }
  auto imu_qos = rclcpp::SensorDataQoS()                   
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                   .keep_last(250);
  //Reentrant callback groups for the IMU and SBG sensors
  rclcpp::SubscriptionOptions imu_options;
  imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  imu_options.callback_group = imu_callback_group_;
  // Timer callback group for the keyframe timer
  keyframe_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  // Subscriptions for sensors using callback groups
  stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, imu_qos,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1),
      imu_options);
    
  sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::SBG_IMU_TOPIC, 100,
      std::bind(&StateEstimator::sbg_callback, this, std::placeholders::_1),
      imu_options);

  dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
      sam_msgs::msg::Topics::DVL_TOPIC, 10, 
    std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));

  barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "core/depth20_pressure", 10,     /*If sim: use depth20 on real sam use depth300, "core/vbs_tank_pressure"*/
      //"core/vbs_tank_pressure", 10,     /*If sim: use depth20 on real sam use depth300, "core/vbs_tank_pressure"*/
      std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
  smarc_msgs::msg::Topics::GPS_TOPIC, 10,
  std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));


  //Subscribe to gt odometry if init_from_ground_truth_ is true
  if(init_from_ground_truth_)
  {
    gt_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mocap/sam_mocap/odom", 10,
      std::bind(&StateEstimator::gt_odom_callback, this, std::placeholders::_1));
  }


  if(using_motion_model_)
  {
    thruster_vector_sub_ = this->create_subscription<sam_msgs::msg::ThrusterAngles>(
      sam_msgs::msg::Topics::THRUST_VECTOR_CMD_TOPIC, 10,
      std::bind(&StateEstimator::ThrusterVectorCallback, this, std::placeholders::_1));

    thruster_rpms_sub_ = this->create_subscription<sam_msgs::msg::ThrusterRPMs>(
      "core/thruster_rpms_cmd", 10,
      std::bind(&StateEstimator::thruster_callback, this, std::placeholders::_1));

    lcg_sub_.subscribe(this, sam_msgs::msg::Topics::LCG_FB_TOPIC);
    vbs_sub_.subscribe(this, sam_msgs::msg::Topics::VBS_FB_TOPIC);

    lcg_vbs_sync_ = std::make_shared<LcgVbsSync>(
      LcgVbsSyncPolicy(10), lcg_sub_, vbs_sub_);

    lcg_vbs_sync_->registerCallback(
      std::bind(&StateEstimator::lcg_vbs_callback, this, std::placeholders::_1, std::placeholders::_2) );
  }

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);  

  // Publishers
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      dead_reckoning_msgs::msg::Topics::DR_ODOM_TOPIC, 10);
      
  utm_publisher_ = this->create_publisher<std_msgs::msg::String>(
      sam_msgs::msg::Topics::UTM_ZONE_BAND, 10);

  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(1000 / kf_interval_hz_),
      std::bind(&StateEstimator::KeyframeTimerCallback, this),
      keyframe_callback_group_);

  RCLCPP_INFO(this->get_logger(), "Keyframe timer set to %d Hz", kf_interval_hz_);

  // Initialize the GtsamGraph with the chosen inference strategy
  gtsam_graph_ = std::make_unique<GtsamGraph>(inference_strategy, config_file);

  // Initialize the PreintegratedMotionModel
  pmm = std::make_shared<PreintegratedMotionModel>(dt_);

  // utm timer for publishing UTM zone band
  utm_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&StateEstimator::utm_timer_publisher, this));

  //this is used for the final GPS injection
  // double bag_duration  = 220; // seconds, adjust as needed
  // double trigger_time  = bag_duration;

  // final_gps_timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(int(trigger_time * 1000.0)),
  //   std::bind(&StateEstimator::start_final_gps_publishing, this));

}

void StateEstimator::start_final_gps_publishing()
{
  // cancel the one-shot
  final_gps_timer_->cancel();

  // reset counter
  gps_fix_count_ = 0;

  // restart as a 1 Hz timer
  final_gps_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&StateEstimator::publish_final_gps, this));
}

void StateEstimator::publish_final_gps()
{

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
      name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,
      "sam_mocap/gps_link",
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
      "Final-GPS timer: TF lookup failed: %s", ex.what());
    return;
  }

  // your variance, point, and flag logic…
  const auto &t = tf.transform.translation;
  latest_gps_point_    = gtsam::Point3(t.x, t.y, t.z);
  new_gps_measurement_ = true;

  RCLCPP_INFO(this->get_logger(),
    "Injected final MoCap GPS factor at (%.3f, %.3f, %.3f)",
    t.x, t.y, t.z);
    
}


void StateEstimator::utm_timer_publisher()
{
  if (!map_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Map not initialized, skipping UTM zone publication");
    return; 
  }
  utm_publisher_->publish(utm_zone_band_);
}



void StateEstimator::gt_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  geometry_msgs::msg::VelocityStamped vel_mocap;
  vel_mocap.header = msg->header;
  vel_mocap.velocity = msg->twist;    

  geometry_msgs::msg::TransformStamped T_odom_from_base;
  try {
    T_odom_from_base = tf_buffer_.lookupTransform(
      "sam_mocap/base_link",                     
      vel_mocap.header.frame_id, 
      rclcpp::Time(0),           
      tf2::durationFromSec(0.1)  
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Quaternion q_odom_from_base;
  tf2::fromMsg(T_odom_from_base.transform.rotation, q_odom_from_base);
  tf2::Matrix3x3 R_odom_from_mocap(q_odom_from_base);

  tf2::Vector3 v_base(
    vel_mocap.velocity.linear.x,
    vel_mocap.velocity.linear.y,
    vel_mocap.velocity.linear.z
  );

  tf2::Vector3 v_odom = R_odom_from_mocap * v_base;

  gt_velocity_ = gtsam::Vector3(v_odom.x(), -v_odom.y(), -v_odom.z());
}


// Callback for the ground truth odometry in order to align the initial odom frame with gt
void StateEstimator::gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::VelocityStamped vel_in;
  vel_in.header  = msg->header;             
  vel_in.velocity = msg->twist.twist;       
  if (!map_initialized_)
  {
    geometry_msgs::msg::TransformStamped ned_to_enu;
    ned_to_enu.header.stamp    = this->get_clock()->now();
    ned_to_enu.header.frame_id = "mocap";           
    ned_to_enu.child_frame_id  = "map";             

    // 180° rotation about X to go from NED to ENU
    ned_to_enu.transform.rotation.x = 0.70710678;
    ned_to_enu.transform.rotation.y = 0.70710678;
    ned_to_enu.transform.rotation.z = 0.0;
    ned_to_enu.transform.rotation.w = 0.0;

    tf_static_broadcaster_->sendTransform(ned_to_enu);
    RCLCPP_INFO(this->get_logger(), "NED→ENU static transform published");

    geometry_msgs::msg::TransformStamped map_to_blgt;
    try {
      map_to_blgt = tf_buffer_.lookupTransform(
        "map",                     
        "sam_mocap/base_link",
        tf2::TimePointZero,     
        tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;                                 
    }
    map_to_blgt.header.frame_id = "map";
    map_to_blgt.child_frame_id  = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ;
  // tf2::Quaternion q = tf2::Quaternion(map_to_blgt.transform.rotation.x,
  //                                    map_to_blgt.transform.rotation.y,
  //                                    map_to_blgt.transform.rotation.z,
  //                                    map_to_blgt.transform.rotation.w);
  
    tf2::Quaternion q_full;
    tf2::fromMsg(map_to_blgt.transform.rotation, q_full);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_full).getRPY(roll, pitch, yaw);
    tf2::Quaternion q_yaw_only;
    q_yaw_only.setRPY(0.0, 0.0, yaw);
    q_yaw_only.normalize();

    map_to_blgt.transform.rotation = tf2::toMsg(q_yaw_only);
    // tf2::Quaternion q_ned_to_enu; 

    // q_ned_to_enu.setRPY(M_PI, 0.0, 0.0);     
    // tf2::Quaternion q_enu =  q * q_ned_to_enu ;
    // q_enu.normalize();
    // map_to_blgt.transform.rotation.x = q_enu.x();
    // map_to_blgt.transform.rotation.y = q_enu.y();
    // map_to_blgt.transform.rotation.z = q_enu.z();
    // map_to_blgt.transform.rotation.w = q_enu.w();
    
    
    gt_init_quat_ = gtsam::Quaternion(
      q_yaw_only.w(),
      q_yaw_only.x(),
      q_yaw_only.y(),
      q_yaw_only.z()
    );
    tf_static_broadcaster_->sendTransform(map_to_blgt);
    RCLCPP_INFO(this->get_logger(), "Map to base_link_gt static transform published");

    geometry_msgs::msg::TransformStamped body_to_odom_init;
    try {
      body_to_odom_init = tf_buffer_.lookupTransform(
          name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,                     
          vel_in.header.frame_id, 
          rclcpp::Time(0),            
          tf2::durationFromSec(0.1)   
        );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    tf2::Quaternion q_body_to_odom_init;
    tf2::fromMsg(body_to_odom_init.transform.rotation, q_body_to_odom_init);
    tf2::Matrix3x3 R_body_to_odom_init(q_body_to_odom_init);

    // Rotate linear velocity:
    const auto & lin_in_init = vel_in.velocity.linear;
    tf2::Vector3 v_body_init(lin_in_init.x, lin_in_init.y, lin_in_init.z);
    tf2::Vector3 v_odom_init = R_body_to_odom_init * v_body_init;

    // Rotate angular velocity (if desired):
    const auto & ang_in_init = vel_in.velocity.angular;
    tf2::Vector3 w_body_init(ang_in_init.x, ang_in_init.y, ang_in_init.z);
    tf2::Vector3 w_odom_init = R_body_to_odom_init * w_body_init;

    init_vel_odom_.header.stamp    = vel_in.header.stamp;
    init_vel_odom_.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
    init_vel_odom_.velocity.linear.x  = v_odom_init.x();
    init_vel_odom_.velocity.linear.y  = v_odom_init.y();
    init_vel_odom_.velocity.linear.z  = v_odom_init.z();
    init_vel_odom_.velocity.angular.x = w_odom_init.x();
    init_vel_odom_.velocity.angular.y = w_odom_init.y();
    init_vel_odom_.velocity.angular.z = w_odom_init.z();



    map_initialized_ = true;
    // gt_pose_sub_.reset(); // Reset the subscription to avoid further callbacks
    return;
  }  

  //
  geometry_msgs::msg::TransformStamped body_to_odom;
  try {
    body_to_odom = tf_buffer_.lookupTransform(
      name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,                   
      msg->child_frame_id,          
      tf2::TimePointZero,           
      tf2::durationFromSec(0.1)     
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF lookup (odom←base_link) failed: %s", ex.what());
    return;
  }
  tf2::Quaternion q_body_to_odom;
  // Construct rotation matrix from quaternion
  tf2::fromMsg(body_to_odom.transform.rotation, q_body_to_odom);
  tf2::Matrix3x3 R_body_to_odom(q_body_to_odom);

  // Rotate linear velocity:
  const auto & lin = vel_in.velocity.linear;
  tf2::Vector3 v_body(lin.x, lin.y, lin.z);
  tf2::Vector3 v_odom = R_body_to_odom * v_body;

  // Rotate angular velocity if needed:
  const auto & ang = vel_in.velocity.angular;
  tf2::Vector3 w_body(ang.x, ang.y, ang.z);
  tf2::Vector3 w_odom = R_body_to_odom * w_body;

  const auto & T = body_to_odom.transform.translation;
  const auto & R_msg = body_to_odom.transform.rotation;
  gtsam::Pose3 pose_in_odom(
    gtsam::Rot3::Quaternion(R_msg.w, R_msg.x, R_msg.y, R_msg.z),
    gtsam::Point3(T.x, T.y, T.z)
  );
  gtsam::Vector3 vel_vec(v_odom.x(), v_odom.y(), v_odom.z());
  // gt_navstate_ = gtsam::NavState(pose_in_odom, vel_vec);
  //rotate with 180 roll
  gtsam::Rot3 R_enu_to_ned = gtsam::Rot3::RzRyRx(M_PI, 0.0, 0.0); // 180° roll to go from NED to ENU in body
  gtsam::Rot3 R_ned = pose_in_odom.rotation().compose(R_enu_to_ned);
  gtsam::Point3 T_ned(pose_in_odom.translation().x(),
                      pose_in_odom.translation().y(),
                      pose_in_odom.translation().z());
  gtsam::Pose3 enu_pose(R_ned, T_ned);
  gt_pose_ = enu_pose; // Store the pose in ENU  odom frame from mocap

}


void StateEstimator::ThrusterVectorCallback(const sam_msgs::msg::ThrusterAngles::SharedPtr msg)
{
  if(is_graph_initialized_)
  {
    Eigen::VectorXd u(2);
    u << msg->thruster_vertical_radians,
        msg->thruster_horizontal_radians;
        
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    {
      std::lock_guard<std::mutex> lk(control_list_mutex_);
      pmm->controlToList(u, timestamp, true);
    }
  }
}

// thrusters-only
void StateEstimator::thruster_callback(const sam_msgs::msg::ThrusterRPMs::SharedPtr msg)
{
  if(is_graph_initialized_)
  {
    last_thr1_rpm_ = msg->thruster_1_rpm;
    last_thr2_rpm_ = msg->thruster_2_rpm;

    Eigen::Vector4d u_fb;
    u_fb << last_lcg_,      
            last_vbs_,
            last_thr1_rpm_,
            last_thr2_rpm_;
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    {
      std::lock_guard<std::mutex> lk(control_list_mutex_);
      pmm->controlToList(u_fb, timestamp, false);
    }
  }
}

// LCG/VBS-only
void StateEstimator::lcg_vbs_callback(
  const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
  const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs)
{
  if(is_graph_initialized_)
  {
    last_lcg_ = lcg->value;
    last_vbs_ = vbs->value;

    Eigen::Vector4d u_fb;
    u_fb << last_lcg_,last_vbs_, last_thr1_rpm_, last_thr2_rpm_;
   double timestamp = rclcpp::Time(lcg->header.stamp).seconds();
   {
     std::lock_guard<std::mutex> lk(control_list_mutex_);
     pmm->controlToList(u_fb, timestamp, false);
   }
  } 
}


// callback for the IMU preintegator
void StateEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  acc = Vector3(acc.x(), acc.y(), acc.z());
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z()); // Adjusted gyro measurements to right-hand rule.
  if(is_graph_initialized_)
  {
    gtsam_graph_->integrateImuMeasurement(acc, gyro, gtsam_graph_->getImuRate());
  }
}


// callback for the SBG IMU, this is currently not used
void StateEstimator::sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);

  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);  

  Vector3 sbg_gyro = Vector3(gyro_raw.x(), gyro_raw.y(), gyro_raw.z());//remove minus sign if using sim
  
  sbg_orientation_ = gtsam::Rot3::Quaternion(
  msg->orientation.w,
  msg->orientation.x,
  msg->orientation.y,
  msg->orientation.z);

  acc = Vector3(acc.x(), acc.y(), acc.z());
  if(number_of_imu_measurements< 6)
  {
    estimated_rotations_.push_back(sbg_orientation_);
    number_of_imu_measurements++;
  }
  // if(is_graph_initialized_)
  // {
  //   gtsam_graph_->integrateSbgMeasurement(acc, sbg_gyro, gtsam_graph_->getSbgRate());
  // }
}


void StateEstimator::dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg)
{  
    // the DVL is in NED, we need to convert to ENU
    Vector3 vel_dvl(msg->velocity.x, -msg->velocity.y, -msg->velocity.z); 
    latest_dvl_measurement_ = vel_dvl;
    covariance_dvl_ << 
          msg->velocity_covariance[0],  
          msg->velocity_covariance[4],  
          msg->velocity_covariance[8];  
    dvl_gyro = gyro;
    new_dvl_measurement_ = true;
}


void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double measured_pressure = msg->fluid_pressure;
  double water_density  = gtsam_graph_->getWaterDensity();
  double depth = -(measured_pressure - atmospheric_pressure_) / (water_density * 9.818); //Down negative 
  if(map_initialized_ && is_graph_initialized_){
    if (!baro_calibrated) {
      auto ext = gtsam_graph_->getExtrinsics();
      gtsam::Vector3 base_to_pressure_offset = ext.baro_sensor_offset;
      gtsam::Vector3 sensor_offset = previous_state_.rotation().rotate(base_to_pressure_offset);
      static_offset_ =  depth-sensor_offset.z(); // this is the offset to the static frame
      baro_calibrated = true;
    }
  latest_depth_measurement_ =  depth - static_offset_; // depth in the odom frame
  new_barometer_measurement_received_ = true;
  }

}


void StateEstimator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if(init_from_ground_truth_) return;
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) 
  {
    RCLCPP_WARN(this->get_logger(), "Received GPS message without valid fix (status: %d)", msg->status.status);
    return;
  }

  double utm_x, utm_y, utm_z;
  if(!map_initialized_ )
  {
  // if sim time is used, take the ground truth as gps reading
    if(this->get_parameter("use_sim_time").as_bool())
    {
      try
      {
        transformStamped = tf_buffer_.lookupTransform("utm_34_V", "sam_auv_v1/gps_link_gt",
                                                        tf2::TimePointZero, std::chrono::seconds(1));
        utm_x = transformStamped.transform.translation.x;
        utm_y = transformStamped.transform.translation.y;
        utm_z = transformStamped.transform.translation.z;
        geometry_msgs::msg::TransformStamped map_transform;
        map_transform.header.stamp = this->get_clock()->now();
        map_transform.header.frame_id = "utm_34_V";     // Frmae name from sim
        map_transform.child_frame_id = "map";        
        map_transform.transform.translation.x = utm_x;
        map_transform.transform.translation.y = utm_y;
        map_transform.transform.translation.z = utm_z;
        // Use an identity rotation for the map frame.
        map_transform.transform.rotation.x = 0.0;
        map_transform.transform.rotation.y = 0.0;
        map_transform.transform.rotation.z = 0.0;
        map_transform.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(map_transform);
        // first utm coordinates of the base_link
        first_utm_x = utm_x;
        first_utm_y = utm_y;
        first_utm_z = utm_z;
        RCLCPP_INFO(this->get_logger(), 
                    "Broadcasted static map transform at local x: %f, y: %f, z: %f", 
                    utm_x, utm_y, utm_z);
        map_initialized_ = true;
      }
      
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
      }
    }

    // if not using the sim, take the real gps reading
    else
    {
      // RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix to initialize map frame");
      RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix to initialize map frame: %f ", number_of_gps_measurements_);
      double var = msg->position_covariance[0];
      if (var > cov_threshold_*cov_threshold_)
      {
        RCLCPP_WARN(get_logger(),
        "GPS covariance too high (sigma=%.1f m), dropping fix", std::sqrt(var));
        return;
      }
        sum_lat_ += msg->latitude;
        sum_lon_ += msg->longitude;
        sum_alt_ += msg->altitude;
        number_of_gps_measurements_++;
      
      if (number_of_gps_measurements_ >= number_of_gps_measurements_for_map_init_) 
      {
        double avg_lat = sum_lat_  / number_of_gps_measurements_;
        double avg_lon = sum_lon_  / number_of_gps_measurements_;
        double avg_alt = sum_alt_  / number_of_gps_measurements_;
        int utm_zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(avg_lat, avg_lon, utm_zone, northp, utm_x, utm_y);
        utm_z = avg_alt;
        std::string gridZone;
        GeographicLib::MGRS::Forward(
            utm_zone, northp, utm_x, utm_y,
            -1,
            gridZone
        );
        char bandLetter = gridZone.back();
        RCLCPP_INFO(this->get_logger(), "Band letter: %c", bandLetter);
        std::string utm = "utm_" + std::to_string(utm_zone) + "_" + bandLetter;
        // Create a static transform from "utm" to "map" using the UTM coordinates.
        utm_zone_band_ = std_msgs::msg::String();
        utm_zone_band_.data = utm;
        geometry_msgs::msg::TransformStamped map_transform;
        map_transform.header.stamp = this->get_clock()->now();
        map_transform.header.frame_id = utm ;
        map_transform.child_frame_id = "map";
        map_transform.transform.translation.x = utm_x;
        map_transform.transform.translation.y = utm_y;
        map_transform.transform.translation.z = 0;
        // Use an identity rotation for the map frame.
        map_transform.transform.rotation.x = 0.0;
        map_transform.transform.rotation.y = 0.0;
        map_transform.transform.rotation.z = 0.0;
        map_transform.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(map_transform);
        // first utm coordinates of the base_link
        first_utm_x = utm_x;
        first_utm_y = utm_y;
        first_utm_z = utm_z;
        RCLCPP_INFO(this->get_logger(), 
                    "Broadcasted static map transform at local x: %f, y: %f, z: %f", 
                    utm_x, utm_y, utm_z); 
        map_initialized_ = true;

        return;
      } 
      return;
    }
    return;
  }
  // Convert the GPS coordinates to UTM coordinates
  int utm_zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone, northp, utm_x, utm_y);
  utm_z = msg->altitude;
  // Compare the new gps message with the first one to get the offset, but we need it in the odom frame
  // if(is_graph_initialized_){
  //   Point3 map_to_odom_offset;
  //   Rot3 map_to_odom_rotation;
  //   try{
  //     transformStamped = tf_buffer_.lookupTransform("map", name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,
  //                                                     tf2::TimePointZero, std::chrono::seconds(1));
  //     map_to_odom_offset = Point3(transformStamped.transform.translation.x,
  //                                 transformStamped.transform.translation.y,
  //                                 transformStamped.transform.translation.z);
  //     map_to_odom_rotation = Rot3(transformStamped.transform.rotation.w,
  //                                 transformStamped.transform.rotation.x,
  //                                 transformStamped.transform.rotation.y,
  //                                 transformStamped.transform.rotation.z);

  //   }
  //   catch (tf2::TransformException &ex) {
  //     RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  //     return;
  //   }
  //   Point3 gps_in_map(utm_x - first_utm_x, utm_y - first_utm_y, utm_z - first_utm_z);
  //   // Apply rotation from map to odom
  //   Point3 gps_in_odom = map_to_odom_rotation.inverse().rotate(gps_in_map - map_to_odom_offset);
  //   latest_gps_point_ = gps_in_odom;
  //   position_variances << 
  //       msg->position_covariance[0],  
  //       msg->position_covariance[4],  
  //       msg->position_covariance[8];  
  //   new_gps_measurement_ = true;
  //   // Logg off the gps point of the gps in the odom frame
  //   RCLCPP_DEBUG(this->get_logger(), "GPS Point: [%f, %f, %f]", latest_gps_point_.x(), latest_gps_point_.y(), latest_gps_point_.z());
  // }
 
}  



Rot3 StateEstimator::averageRotations(const std::vector<Rot3>& rotations) {
  Vector3 sumLog = Vector3::Zero();
  for (const auto& rot : rotations) {
    sumLog += Rot3::Logmap(rot);
  }
  Vector3 avgLog = sumLog / static_cast<double>(rotations.size());
  return Rot3::Expmap(avgLog);
}


void StateEstimator::start_without_mocap()
{
  geometry_msgs::msg::TransformStamped mocap_2_map;
  mocap_2_map.header.stamp    = this->get_clock()->now();
  mocap_2_map.header.frame_id = "mocap";           
  mocap_2_map.child_frame_id  = "map";   
  
  tf2::Quaternion q_mocap_map;
  tf2::fromMsg(mocap_2_map.transform.rotation, q_mocap_map);

  // 180° rotation about X to go from FRD to FLU
  tf2::Quaternion q_frd_to_flu;
  q_frd_to_flu.setRPY(M_PI, 0.0, 0.0);     
  tf2::Quaternion q_flu =  q_mocap_map * q_frd_to_flu ;
  q_flu.normalize();
  mocap_2_map.transform.rotation = tf2::toMsg(q_flu);

  tf_static_broadcaster_->sendTransform(mocap_2_map);
  RCLCPP_INFO(this->get_logger(), "Mocap->map static transform published");

  // Odom is one meter away from map in x
  geometry_msgs::msg::TransformStamped map_to_blgt;
  map_to_blgt.transform.translation.x = 1.;
  map_to_blgt.transform.translation.y = 0.;
  map_to_blgt.transform.translation.z = 0.;

  map_to_blgt.header.frame_id = "map";
  map_to_blgt.child_frame_id  = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ;
  // tf2::Quaternion q = tf2::Quaternion(map_to_blgt.transform.rotation.x,
  //                                    map_to_blgt.transform.rotation.y,
  //                                    map_to_blgt.transform.rotation.z,
  //                                    map_to_blgt.transform.rotation.w);

  tf2::Quaternion q_full;
  tf2::fromMsg(map_to_blgt.transform.rotation, q_full);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_full).getRPY(roll, pitch, yaw);
  tf2::Quaternion q_yaw_only;
  q_yaw_only.setRPY(0.0, 0.0, yaw);
  q_yaw_only.normalize();
  map_to_blgt.transform.rotation = tf2::toMsg(q_yaw_only);
  // tf2::Quaternion q_ned_to_enu; 

  // q_ned_to_enu.setRPY(M_PI, 0.0, 0.0);     
  // tf2::Quaternion q_enu =  q * q_ned_to_enu ;
  // q_enu.normalize();
  // map_to_blgt.transform.rotation.x = q_enu.x();
  // map_to_blgt.transform.rotation.y = q_enu.y();
  // map_to_blgt.transform.rotation.z = q_enu.z();
  // map_to_blgt.transform.rotation.w = q_enu.w();

  gt_init_quat_ = gtsam::Quaternion(
    q_yaw_only.w(),
    q_yaw_only.x(),
    q_yaw_only.y(),
    q_yaw_only.z()
  );

  tf_static_broadcaster_->sendTransform(map_to_blgt);
  map_initialized_ = true;
}


void StateEstimator::KeyframeTimerCallback()
{
  // need to have at least 6 imu measurements to initialize the graph with the current orientation
  std::lock_guard<std::mutex> lock(imu_mutex_);
  // if(number_of_imu_measurements < 6){
  //       RCLCPP_INFO(get_logger(),
  //   "  skipping: only %d IMUs (need ≥6)", number_of_imu_measurements);
  //   return;
  //   }

  if(!map_initialized_){
      RCLCPP_INFO(get_logger(), "  skipping: map_initialized_ == false");
      start_without_mocap(); // Nacho: to start at the surface without the mocap
    return;
  }

  if (!is_graph_initialized_)
  {
    Quaternion initial_quat;
    initial_quat = gtsam::Quaternion(1.0, 0.0, 0.0, 0.0); 
    Point3 initial_position = Point3(0.0, 0.0, 0.0);
    if(init_from_ground_truth_)
    {

      // Nacho: to start at the surface without the mocap, override this
      // //look up the base link to odom transform
      // geometry_msgs::msg::TransformStamped odom_transform;
      
      // try {
      //   odom_transform = tf_buffer_.lookupTransform(
      //   name_space_+"/"+sam_msgs::msg::Links::ODOM_LINK, "sam_mocap/base_link",
      //   tf2::TimePointZero, std::chrono::microseconds(10));
      // } 
      // catch (tf2::TransformException &ex) 
      // {
      //   RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      //   return;
      // }
      
      // Nacho: Surface testing without mocap
      // tf2::Quaternion q;
      // tf2::fromMsg(odom_transform.transform.rotation, q);
      // q.setRPY(M_PI, 0.0, M_PI/2.);
      // tf2::Quaternion q_ned_to_enu; 
      // q_ned_to_enu.setRPY(M_PI, 0.0, 0.0);     
      // tf2::Quaternion q_enu = q*q_ned_to_enu ;
      // q_enu.normalize();
      // //Extrect the orientation from the transform

      // initial_quat = gtsam::Quaternion(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
      // initial_position = gtsam::Point3(odom_transform.transform.translation.x,
      //                                   odom_transform.transform.translation.y,
      //                                   odom_transform.transform.translation.z);
                                        // Broadcast the initial pose.

      //// Nacho: Surface testing without mocap
      initial_position = gtsam::Point3(0.0,0.,0.);
      initial_quat = gtsam::Quaternion(1.0, 0.0, 0.0, 0.0); 
      ////

      geometry_msgs::msg::TransformStamped init_transform;
      init_transform.header.stamp = this->get_clock()->now();
      init_transform.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ; 
      init_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK ; 
      init_transform.transform.translation.x = initial_position.x();
      init_transform.transform.translation.y = initial_position.y();
      init_transform.transform.translation.z = initial_position.z();

      init_transform.transform.rotation.x = initial_quat.x();
      init_transform.transform.rotation.y = initial_quat.y();
      init_transform.transform.rotation.z = initial_quat.z();
      init_transform.transform.rotation.w = initial_quat.w();
      tf_broadcast_.sendTransform(init_transform);

      RCLCPP_INFO(this->get_logger(), "Initial base_link transform at [%f,%f,%f]", init_transform.transform.translation.x, init_transform.transform.translation.y,init_transform.transform.translation.z );

      RCLCPP_INFO(this->get_logger(), "Initial orientation, [%f,%f,%f,%f]",initial_quat.w(), initial_quat.x(), initial_quat.y(), initial_quat.z());
      RCLCPP_INFO(this->get_logger(), "Initial position, [%f,%f,%f]",initial_position.x(),initial_position.y(),initial_position.z());
    }
    else
    {
      Rot3 average_rotation = averageRotations(estimated_rotations_);
      Quaternion quat = average_rotation.toQuaternion();
      // Initialize the odom frame from map
      auto ext = gtsam_graph_->getExtrinsics();
      gtsam::Vector3 base_to_gps_offset = ext.gps_sensor_offset;
      tf2::Quaternion q_tf2( quat.x(), quat.y(), quat.z(), quat.w());

      tf2::Vector3 off_base(base_to_gps_offset.x(),base_to_gps_offset.y(),base_to_gps_offset.z()  );
      tf2::Vector3 off_map = tf2::quatRotate(q_tf2, off_base);
      geometry_msgs::msg::TransformStamped odom_transform;
      odom_transform.header.stamp = this->get_clock()->now();
      odom_transform.header.frame_id = "map";
      odom_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ;
      // translate the map -> odom with -base_to_gps_offset in x and y

      odom_transform.transform.translation.x = -off_map.x(); // x and y were swapped from the sim
      odom_transform.transform.translation.y = -off_map.y();
      odom_transform.transform.translation.z = -off_map.z();
      Rot3 map_to_odom_rot = average_rotation;
      Quaternion map_to_odom_quat = map_to_odom_rot.toQuaternion();
      odom_transform.transform.rotation.x = 0;
      odom_transform.transform.rotation.y = 0;
      odom_transform.transform.rotation.z = 0;
      odom_transform.transform.rotation.w = 1;
      //no roation for initial quat
      initial_quat = gtsam::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
      tf_static_broadcaster_->sendTransform(odom_transform);
      RCLCPP_INFO(this->get_logger(),
                  "Initialized odom at map (%.3f, %.3f, %.3f)",
                  odom_transform.transform.translation.x,
                  odom_transform.transform.translation.y,
                  odom_transform.transform.translation.z);
    }
    
    
    Vector3 initial_velocity = Vector3(init_vel_odom_.velocity.linear.x,
                                        init_vel_odom_.velocity.linear.y,
                                        init_vel_odom_.velocity.linear.z);
    initial_velocity = Vector3(0,0,0);
    // Initialize the GTSAM graph
    gtsam_graph_->initGraphAndState(initial_quat, initial_position,initial_velocity);
    gtsam_graph_->getImuPreintegrated()->resetIntegration();
    // gtsam_graph_->sbg_preintegrated_->resetIntegration();


    RCLCPP_INFO(this->get_logger(),"initial position: [%f, %f, %f]",
                initial_position.x(), initial_position.y(), initial_position.z());
    RCLCPP_INFO(this->get_logger(),"initial velocity: [%f, %f, %f]",
                initial_velocity.x(), initial_velocity.y(), initial_velocity.z());
    RCLCPP_INFO(this->get_logger(),"Graph initialized !!!");
    previous_state_ = gtsam_graph_->getCurrentState();
    is_graph_initialized_ = true;
    current_time = this->get_clock()->now().seconds();
    last_time_ = current_time;
    return;
    
  }   
  
  auto [imu_dt, sbg_dt] = gtsam_graph_->getTij();
  if (imu_dt <= 0.0/* || sbg_dt <= 0.0*/)
  {
    RCLCPP_INFO(get_logger(),"No new IMU/SBG data this cycle (imu_dt=%.6f, sbg_dt=%.6f), skipping factors + optimize",
      imu_dt, sbg_dt);
    return;
  }
  if(using_motion_model_)
  {
    double current_time = this->get_clock()->now().seconds();
    NavState state = NavState(previous_state_.pose(), previous_state_.velocity());
    NavState new_state;
    {
      std::lock_guard<std::mutex> lk(control_list_mutex_);
      
      new_state = pmm->predict(state, gyro, last_time_,
              current_time,gtsam_graph_->getCurrentCovariance(gtsam_graph_->getCurrentIndex()));
      gtsam_graph_->addMotionModelFactor(last_time_,current_time,pmm,gyro,new_state);
    }
    last_time_ = current_time;

  }
  
  // Predict the next state using the preintegrated measurements AND add the imu factor to the graph.
  NavState predictes_imu_state = gtsam_graph_->addImuFactor();


  // NavState predicted_sbg_state = gtsam_graph_->addSbgFactor();
  // RCLCPP_INFO(this->get_logger(), "SBG prediction state: [%f, %f, %f]",predicted_sbg_state.pose().translation().x(),
  //   predicted_sbg_state.pose().translation().y(),
  //   predicted_sbg_state.pose().translation().z());


  // if(init_from_ground_truth_ && gtsam_graph_->getCurrentIndex()<100) 
  // {
  //   // gtsam_graph_->addGtVelocityFactor(gt_velocity_);
  //   // RCLCPP_INFO(this->get_logger(), "Ground truth velocity factor added: [%f, %f, %f]",+
  //   //   gt_velocity_.x(), gt_velocity_.y(), gt_velocity_.z());
  //   gtsam_graph_->addGtPoseFactor(gt_pose_);
  // }
  // Add the DVL, GPS and Barometer factors to the graph.

  // gtsam_graph_->addSbgOrientationFactor(sbg_orientation_);

  if (new_dvl_measurement_) {  
    gtsam_graph_->addDvlFactor(latest_dvl_measurement_, dvl_gyro, covariance_dvl_, use_sensor_covariance_);
    // RCLCPP_INFO(this->get_logger(), "DVL Factor velocity [%f, %f, %f]", latest_dvl_measurement_.x(),latest_dvl_measurement_.y(), latest_dvl_measurement_.z());
    new_dvl_measurement_ = false;
  }

  if (new_gps_measurement_) {
    gtsam_graph_->addGpsFactor(latest_gps_point_, position_variances, use_sensor_covariance_);
    RCLCPP_INFO(this->get_logger(), "GPS Factor position [%f, %f, %f]", latest_gps_point_.x(), latest_gps_point_.y(), latest_gps_point_.z());
    new_gps_measurement_ = false;
  }
// 
  if (new_barometer_measurement_received_) {
    gtsam_graph_->addBarometerFactor(latest_depth_measurement_);
    new_barometer_measurement_received_ = false;
  }

  gtsam_graph_->optimize();

  if(using_motion_model_){
    std::lock_guard<std::mutex> lk(control_list_mutex_);
    pmm->resetIntegration();
  }
  current_imu_bias_ = gtsam_graph_->getCurrentImuBias();

  previous_state_ = gtsam_graph_->getCurrentState();

  // To get the same timestamp for message and tf
  rclcpp::Time stamp = this->get_clock()->now();
  
  // Publish the estimated pose.
  nav_msgs::msg::Odometry estimated_pose;
  estimated_pose.header.stamp = stamp;
  estimated_pose.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ; 
  estimated_pose.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK ; 
  estimated_pose.pose.pose.position.x = previous_state_.pose().translation().x();
  estimated_pose.pose.pose.position.y = previous_state_.pose().translation().y();
  estimated_pose.pose.pose.position.z = previous_state_.pose().translation().z();
  Quaternion quat = previous_state_.pose().rotation().toQuaternion();
  estimated_pose.pose.pose.orientation.x = quat.x();
  estimated_pose.pose.pose.orientation.y = quat.y();
  estimated_pose.pose.pose.orientation.z = quat.z();
  estimated_pose.pose.pose.orientation.w = quat.w();
  Eigen::Vector3d v_body(
    previous_state_.velocity().x(),
    previous_state_.velocity().y(),
    previous_state_.velocity().z()
  );
  // we are not estimateing the angular vels but the bias so take the current angular from stim, but substract the bias
  Eigen::Vector3d w_body(
    gyro.x()-current_imu_bias_.gyroscope().x(),
    gyro.y()-current_imu_bias_.gyroscope().y(),
    gyro.z()-current_imu_bias_.gyroscope().z()
  );

  // rotation matrix from odom to body frame
  Eigen::Matrix3d R = previous_state_.pose().rotation().transpose().matrix();
  Eigen::Vector3d v_odom = R * v_body;
  estimated_pose.twist.twist.linear.x = v_odom.x();
  estimated_pose.twist.twist.linear.y = v_odom.y();
  estimated_pose.twist.twist.linear.z = v_odom.z();
  estimated_pose.twist.twist.angular.x = w_body.x();
  estimated_pose.twist.twist.angular.y = w_body.y();
  estimated_pose.twist.twist.angular.z = w_body.z();
  pose_pub_->publish(estimated_pose);

  // Broadcast estimated pose.
  geometry_msgs::msg::TransformStamped out_transform;
  out_transform.header.stamp = stamp;
  out_transform.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK ;
  out_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK ;
  Point3 estimated_translation = previous_state_.pose().translation();
  Rot3 estimated_rotation = previous_state_.pose().rotation();
  out_transform.transform.translation.x = estimated_translation.x();
  out_transform.transform.translation.y = estimated_translation.y();
  out_transform.transform.translation.z = estimated_translation.z();
  Quaternion out_quat = estimated_rotation.toQuaternion();
  out_transform.transform.rotation.x = out_quat.x();
  out_transform.transform.rotation.y = out_quat.y();
  out_transform.transform.rotation.z = out_quat.z();
  out_transform.transform.rotation.w = out_quat.w();
  tf_broadcast_.sendTransform(out_transform);
}



int main(int argc, char **argv) {
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StateEstimator>();
  py::gil_scoped_release release;

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), /*num_threads=*/2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
