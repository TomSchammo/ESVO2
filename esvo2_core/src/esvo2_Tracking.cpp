#include <esvo2_core/esvo2_Tracking.h>
#include <esvo2_core/tools/TicToc.h>
#include <esvo2_core/tools/params_helper.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <sys/stat.h>

//#define ESVO2_CORE_TRACKING_DEBUG
//#define ESVO2_CORE_TRACKING_DEBUG
namespace esvo2_core
{
esvo2_Tracking::esvo2_Tracking()
  : rclcpp::Node("esvo2_tracking"),
  calibInfoDir_(tools::param(this, "calibInfoDir", std::string(""))),
  camSysPtr_(new CameraSystem(calibInfoDir_, false)),
  rpConfigPtr_(new RegProblemConfig(
    tools::param(this, "patch_size_X", 25),
    tools::param(this, "patch_size_Y", 25),
    tools::param(this, "kernelSize", 15),
    tools::param(this, "LSnorm", std::string("l2")),
    tools::param(this, "huber_threshold", 10.0),
    tools::param(this, "invDepth_min_range", 0.0),
    tools::param(this, "invDepth_max_range", 0.0),
    tools::param(this, "MIN_NUM_EVENTS", 1000),
    tools::param(this, "MAX_REGISTRATION_POINTS", 500),
    tools::param(this, "BATCH_SIZE", 200),
    tools::param(this, "MAX_ITERATION", 10))),
  rpType_((RegProblemType)((size_t)tools::param(this, "RegProblemType", 0))),
  rpSolver_(camSysPtr_, rpConfigPtr_, rpType_, NUM_THREAD_TRACKING),
  ESVO2_System_Status_(SystemStatus::INITIALIZATION),
  imu_data_(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), g_optimal),
  ets_(IDLE)
{
  // offline data
  dvs_frame_id_        = tools::param(this, "dvs_frame_id", std::string("dvs"));
  world_frame_id_      = tools::param(this, "world_frame_id", std::string("world"));

  /**** online parameters ***/
  tracking_rate_hz_    = tools::param(this, "tracking_rate_hz", 100);
  TS_HISTORY_LENGTH_   = tools::param(this, "TS_HISTORY_LENGTH", 100);
  REF_HISTORY_LENGTH_  = tools::param(this, "REF_HISTORY_LENGTH", 5);
  bSaveTrajectory_     = tools::param(this, "SAVE_TRAJECTORY", false);
  bVisualizeTrajectory_ = tools::param(this, "VISUALIZE_TRAJECTORY", true);
  bUseImu_ = tools::param(this, "USE_IMU", true);
  resultPath_          = tools::param(this, "PATH_TO_SAVE_TRAJECTORY", std::string());

  // system status - use shared topic for inter-node coordination (ROS2 parameters are node-local)
  system_status_pub_ = create_publisher<std_msgs::msg::String>("/ESVO2_SYSTEM_STATUS", rclcpp::QoS(1).transient_local());
  system_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/ESVO2_SYSTEM_STATUS", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (auto status = system_status_from_string(msg->data))
        ESVO2_System_Status_.store(*status);
    });

  // get extrinsic parameters
  R_b_c_ = camSysPtr_->cam_left_ptr_->T_b_c_.block<3, 3>(0, 0);

  imu_data_.initialization(ba_, bg_);
  initVsFlag = false;

  // message_filters subscribers
  TS_left_sub_.subscribe(this, "time_surface_left", rmw_qos_profile_default);
  TS_right_sub_.subscribe(this, "time_surface_right", rmw_qos_profile_default);
  TS_negative_sub_.subscribe(this, "time_surface_negative", rmw_qos_profile_default);
  TS_dx_sub_.subscribe(this, "time_surface_dx", rmw_qos_profile_default);
  TS_dy_sub_.subscribe(this, "time_surface_dy", rmw_qos_profile_default);

  // message_filters synchronizers
  TS_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
    ApproximateSyncPolicy(10), TS_left_sub_, TS_right_sub_);
  TS_negaTS_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy_negaTS>>(
    ApproximateSyncPolicy_negaTS(10), TS_left_sub_, TS_negative_sub_, TS_dx_sub_, TS_dy_sub_);
  TS_negaTS_sync_->registerCallback(std::bind(&esvo2_Tracking::timeSurface_NegaTS_Callback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/esvo2_tracking/pose_pub", 1);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/esvo2_tracking/trajectory", 1);

  // Subscribers
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", 10,
    std::bind(&esvo2_Tracking::refMapCallback, this, std::placeholders::_1));
  stampedPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "stamped_pose", 10,
    std::bind(&esvo2_Tracking::stampedPoseCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&esvo2_Tracking::refImuCallback, this, std::placeholders::_1));
  V_ba_bg_sub_ = create_subscription<events_repacking_tool::msg::VBaBg>(
    "/esvo2_mapping/V_ba_bg", 10,
    std::bind(&esvo2_Tracking::VBaBgCallback, this, std::placeholders::_1));

  /*** For Visualization and Test ***/
  reprojMap_pub_left_ = image_transport::create_publisher(this, "Reproj_Map_Left");
  rpSolver_.setRegPublisher(&reprojMap_pub_left_);

  // rename the old trajectory file
  renameOldTraj();

  /*** Tracker ***/
  T_world_cur_ = Eigen::Matrix<double,4,4>::Identity();
  t_world_cur_ = last_t_world_cur_ = last_t_ = Eigen::Vector3d::Zero();
  std::thread TrackingThread(&esvo2_Tracking::TrackingLoop, this);
  TrackingThread.detach();
}

esvo2_Tracking::~esvo2_Tracking()
{
  if(!resultPath_.empty())
  {
    LOG(INFO) << "pose size: " << lPose_.size();
    std::string path = std::string(resultPath_ + "result.txt");
    saveTrajectory(path);
  }
  // ROS2 handles cleanup automatically through shared_ptr
}

void esvo2_Tracking::TrackingLoop()
{
  rclcpp::Rate r(tracking_rate_hz_);
  while(rclcpp::ok())
  {
    // Keep Idling
    if(refPCMap_.size() < 1 || TS_history_.size() < 1)
    {
      r.sleep();
      continue;
    }

    // Reset - ESVO2_System_Status_ is updated via subscription callback
    if(ESVO2_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == WORKING)// This is true when the system is reset from dynamic reconfigure
    {
      reset();
      r.sleep();
      continue;
    }
    if(ESVO2_System_Status_.load() == SystemStatus::TERMINATE)
    {
      LOG(INFO) << "The tracking node is terminated manually...";
      break;
    }
    TicToc tt;
    double curData_time;
    // Data Transfer (If mapping node had published refPC.)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if(ref_.t_.seconds() < refPCMap_.rbegin()->first.seconds())// new reference map arrived
        refDataTransferring();
      if(cur_.t_.seconds() < TS_history_.rbegin()->first.seconds())// new observation arrived
      {
        if(ref_.t_.seconds() >= TS_history_.rbegin()->first.seconds())
        {
          LOG(INFO) << "The time_surface observation should be obtained after the reference frame";
          exit(-1);
        }
        if(!curDataTransferring())
        {
          continue;
        }
      }
      else
      {
        continue;
      }
    }
    curData_time = tt.toc();

    // create new regProblem
    double t_resetRegProblem, t_solve, t_pub_result;

    if(rpSolver_.resetRegProblem(&ref_, &cur_))
    {
      if(ets_ == IDLE)
        ets_ = WORKING;
      if(ESVO2_System_Status_.load() != SystemStatus::WORKING)
      {
        ESVO2_System_Status_.store(SystemStatus::WORKING);
        auto status_msg = std_msgs::msg::String();
        status_msg.data = to_string(ESVO2_System_Status_.load());
        system_status_pub_->publish(status_msg);
        LOG(INFO) << "ESVO2_SYSTEM_STATUS: WORKING";
      }

      // TicToc t_coarse;
      if(rpType_ == REG_NUMERICAL)
        rpSolver_.solve_numerical();
      if(rpType_ == REG_ANALYTICAL)
        rpSolver_.solve_analytical();

      T_world_cur_ = cur_.tr_.getTransformationMatrix();
      t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
      publishPose(cur_.t_, cur_.tr_);
      if(bVisualizeTrajectory_)
        publishPath(cur_.t_, cur_.tr_);

      // save result and gt if available.
      if(bSaveTrajectory_)
      {
        // save results to listPose and listPoseGt
        lTimestamp_.push_back(std::to_string(cur_.t_.seconds()));
        lPose_.push_back(cur_.tr_.getTransformationMatrix());
      }
    }
    else
    {
      ESVO2_System_Status_.store(SystemStatus::INITIALIZATION);
      auto status_msg = std_msgs::msg::String();
      status_msg.data = to_string(ESVO2_System_Status_.load());
      system_status_pub_->publish(status_msg);
      ets_ = IDLE;
    }
    std::ofstream f;

#ifdef  ESVO2_CORE_TRACKING_LOG
    double t_overall_count = 0;
    t_overall_count = t_resetRegProblem + t_solve + t_pub_result;
    LOG(INFO) << "\n";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "--------------------Tracking Computation Cost---------------";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "ResetRegProblem: " << t_resetRegProblem << " ms, (" << t_resetRegProblem / t_overall_count * 100 << "%).";
    LOG(INFO) << "Registration: " << t_solve << " ms, (" << t_solve / t_overall_count * 100 << "%).";
    LOG(INFO) << "pub result: " << t_pub_result << " ms, (" << t_pub_result / t_overall_count * 100 << "%).";
    LOG(INFO) << "Total Computation (" << rpSolver_.lmStatics_.nPoints_ << "): " << t_overall_count << " ms.";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "------------------------------------------------------------";
#endif
    r.sleep();
  }// while
}

bool
esvo2_Tracking::refDataTransferring()
{
  // load reference info
  ref_.t_ = rclcpp::Time(static_cast<int64_t>(refPCMap_.rbegin()->first.seconds() * 1e9));
  rclcpp::Time t(static_cast<int64_t>((refPCMap_.rbegin()->first.seconds() - 0.001) * 1e9));
  // ESVO2_System_Status_ is updated via subscription callback

  if(ESVO2_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == IDLE)
    ref_.tr_.setIdentity();
  auto status = ESVO2_System_Status_.load();
  if(status == SystemStatus::WORKING || (status == SystemStatus::INITIALIZATION && ets_ == WORKING))
  {
    if(!getPoseAt(t, ref_.tr_, dvs_frame_id_))
    {
      LOG(INFO) << "ESVO2_System_Status_: " << to_string(ESVO2_System_Status_.load()) << ", ref_.t_: " << ref_.t_.nanoseconds();
      LOG(INFO) << "Logic error ! There must be a pose for the given timestamp, because mapping has been finished.";
      // exit(-1);
      return false;
    }
  }

  // get the point cloud
  size_t numPoint = refPCMap_.rbegin()->second->size();
  ref_.vPointXYZPtr_.clear();
  ref_.vPointXYZPtr_.reserve(numPoint);
  auto PointXYZ_begin_it = refPCMap_.rbegin()->second->begin();
  auto PointXYZ_end_it   = refPCMap_.rbegin()->second->end();
  while(PointXYZ_begin_it != PointXYZ_end_it)
  {
    ref_.vPointXYZPtr_.push_back(PointXYZ_begin_it.base());// Copy the pointer of the pointXYZ
    PointXYZ_begin_it++;
  }
  return true;
}


bool
esvo2_Tracking::curDataTransferring()
{
  // load current observation
  auto ev_last_it = EventBuffer_lower_bound(events_left_, cur_.t_);
  auto TS_it = TS_history_.rbegin();

  // TS_history may not be updated before the tracking loop excutes the data transfering
  if(cur_.t_.nanoseconds() == TS_it->first.nanoseconds())
    return false;
  cur_.t_ = TS_it->first;
  cur_.pTsObs_ = &TS_it->second;

  // ESVO2_System_Status_ is updated via subscription callback
  if(ESVO2_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == IDLE)
  {
    cur_.tr_ = ref_.tr_;
  }
  auto status = ESVO2_System_Status_.load();
  if(status == SystemStatus::WORKING || (status == SystemStatus::INITIALIZATION && ets_ == WORKING))
  {
    curImuTransferring();
    // if use imu, the pose of the current frame is updated in the imu preintegration.
    if(bUseImu_)
    {
      Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);

      Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond q = q1.slerp(1, Imu_q);

      if(t_world_cur_ != Eigen::Vector3d::Zero() && last_t_world_cur_ != Eigen::Vector3d::Zero())
      {
        last_t_ = t_world_cur_ - last_t_world_cur_;
        last_t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
        qprevious_ts_.push_back(last_t_);
        int average_window_size = 5;
        if(qprevious_ts_.size() > average_window_size)
        {
          qprevious_ts_.erase(qprevious_ts_.begin(), qprevious_ts_.begin() + qprevious_ts_.size() - average_window_size);
        }
        if(qprevious_ts_.size() > 3)
        {
          last_t_.setZero();
          for(Eigen::Vector3d t: qprevious_ts_)
          {
            last_t_ += t;
          }
          last_t_ = last_t_ / qprevious_ts_.size();
        }

        // If the predicted position change is significantly different from the previous displacement due to potentially unstable velocity estimates,
        // use the previous displacement as the initial value for the next optimization.
        if(initVsFlag && (imu_data_.t_v_last_mapping.second * imu_data_.sum_dt - last_t_).norm()/last_t_.norm() < 0.1)
          T_world_cur_.block(0, 3, 3, 1) += R_b_c_.transpose() * Imu_t + (imu_data_.t_v_last_mapping.second * imu_data_.sum_dt);
        else
          T_world_cur_.block(0, 3, 3, 1) += R_b_c_.transpose() * Imu_t + last_t_;
      }
      else
      {
        last_t_world_cur_ = T_world_cur_.block(0, 3, 3, 1);
      }

      T_world_cur_.block(0, 0, 3, 3) =  R_w_c * R_b_c_* q.toRotationMatrix() * R_b_c_.inverse();
    }
    Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
    T_world_cur_.block(0, 0, 3, 3) = fixRotationMatrix(R_w_c);
    cur_.tr_ = Transformation(T_world_cur_);
  }
  return true;
}

bool esvo2_Tracking::curImuTransferring()
{
  auto TS_it = TS_history_.rbegin();
  double cur_TS_time = TS_it->first.seconds();
  double last_TS_time = (++TS_it)->first.seconds();

  if(bUseImu_)
  {
    imu_mutex_.lock();
    if(initVsFlag && (refPCMap_.rbegin()->first.seconds() - imu_data_.t_v_last_mapping.first > 0.001))
        imu_data_.update_v(refPCMap_.rbegin()->first.seconds(), last_TS_time);
    imu_data_.getPose(last_TS_time, cur_TS_time, true, ref_.t_.seconds());
    if(t_world_cur_ != Eigen::Vector3d::Zero() && last_t_world_cur_ != Eigen::Vector3d::Zero())
    {
      Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
      imu_data_.t_v_last_mapping.second += R_b_c_ * R_w_c.transpose() * imu_data_.delta_v;
    }
    imu_mutex_.unlock();
  }

  Eigen::Vector3d delta_p = imu_data_.delta_p;
  Eigen::Quaterniond delta_q = imu_data_.delta_q;

  Imu_q = imu_data_.delta_q ;
  Imu_t = imu_data_.delta_p;
  return true;
}


void esvo2_Tracking::reset()
{
  // clear all maintained data
  ets_ = IDLE;
  TS_id_ = 0;
  TS_history_.clear();
  refPCMap_.clear();
  events_left_.clear();
}


/********************** Callback functions *****************************/
void esvo2_Tracking::refImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  Eigen::Vector3d acc, gyr;
  if(imu_data_.dt_buf.size() == 0){

    acc[0] = msg->linear_acceleration.x;
    acc[1] = msg->linear_acceleration.y;
    acc[2] = msg->linear_acceleration.z;

    gyr[0] = msg->angular_velocity.x;
    gyr[1] = msg->angular_velocity.y;
    gyr[2] = msg->angular_velocity.z;
    if(imu_data_.last_time == 0)
    {
      imu_data_.begin_time = rclcpp::Time(msg->header.stamp).seconds();
      imu_data_.push_back(0.001, acc, gyr);
      imu_data_.last_time = imu_data_.begin_time;
    }
    else
    {
      double dt = rclcpp::Time(msg->header.stamp).seconds() - imu_data_.last_time;
      if(dt < 0)
        return;
      imu_data_.begin_time = rclcpp::Time(msg->header.stamp).seconds();
      imu_data_.push_back(dt, acc, gyr);
    }
  }
  else{
    double time = rclcpp::Time(msg->header.stamp).seconds();
    double dt = time - imu_data_.last_time;
    if(dt < 0)
      return;
    acc[0] = msg->linear_acceleration.x;
    acc[1] = msg->linear_acceleration.y;
    acc[2] = msg->linear_acceleration.z;

    gyr[0] = msg->angular_velocity.x;
    gyr[1] = msg->angular_velocity.y;
    gyr[2] = msg->angular_velocity.z;
    imu_data_.push_back(dt, acc, gyr);
    imu_data_.last_time = time;
  }
}

void esvo2_Tracking::VBaBgCallback(const events_repacking_tool::msg::VBaBg::SharedPtr msg)
{
  Eigen::Vector3d g_temp, ba_temp, bg_temp, V_temp;
  double t_temp = msg->head[0];
  for(int i = 0; i < 3; i++)
  {
    g_temp(i) = msg->g[i];
    ba_temp(i) = msg->ba[i];
    bg_temp(i) = msg->bg[i];
    V_temp(i) = msg->vs[i];
  }
  imu_mutex_.lock();
  imu_data_.G = g_temp;
  imu_data_.linearized_ba = ba_temp;
  imu_data_.linearized_bg = bg_temp;
  imu_data_.t_v_last_mapping = std::make_pair(t_temp, V_temp);
  imu_mutex_.unlock();
  initVsFlag = true;
}

void esvo2_Tracking::refMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr PC_ptr(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::fromPCLPointCloud2(pcl_pc, *PC_ptr);
  refPCMap_.emplace(msg->header.stamp, PC_ptr);
  while(refPCMap_.size() > REF_HISTORY_LENGTH_)
  {
    auto it = refPCMap_.begin();
    refPCMap_.erase(it);
  }
}

void esvo2_Tracking::eventsCallback(
  const dvs_msgs::msg::EventArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add new ones and remove old ones
  for(const dvs_msgs::msg::Event& e : msg->events)
  {
    events_left_.push_back(e);
    int i = events_left_.size() - 2;
    while(i >= 0 && rclcpp::Time(events_left_[i].ts).nanoseconds() > rclcpp::Time(e.ts).nanoseconds()) // we may have to sort the queue, just in case the raw event messages do not come in a chronological order.
    {
      events_left_[i+1] = events_left_[i];
      i--;
    }
    events_left_[i+1] = e;
  }
  clearEventQueue();
}

void esvo2_Tracking::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
  if (events_left_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_left_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_left_.erase(events_left_.begin(), events_left_.begin() + remove_events);
  }
}

void
esvo2_Tracking::timeSurface_NegaTS_Callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_left,
  const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_negative,
  const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_dx,
  const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_dy)
{

  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_negative, cv_ptr_dx, cv_ptr_dy;
  try
  {
    cv_ptr_left  = cv_bridge::toCvCopy(time_surface_left,  sensor_msgs::image_encodings::MONO8);
    cv_ptr_negative = cv_bridge::toCvCopy(time_surface_negative, sensor_msgs::image_encodings::MONO8);
    cv_ptr_dx = cv_bridge::toCvCopy(time_surface_dx, sensor_msgs::image_encodings::TYPE_16SC1);
    cv_ptr_dy = cv_bridge::toCvCopy(time_surface_dy, sensor_msgs::image_encodings::TYPE_16SC1);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  // push back the most current TS.
  rclcpp::Time t_new_ts = time_surface_left->header.stamp;
  TS_history_.emplace(t_new_ts, TimeSurfaceObservation(cv_ptr_left, cv_ptr_negative, cv_ptr_dx, cv_ptr_dy, TS_id_, false));
  TS_id_++;

  // keep TS_history_'s size constant
  while(TS_history_.size() > TS_HISTORY_LENGTH_)
  {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
}

void esvo2_Tracking::stampedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Create transform and add to buffer
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header = msg->header;
  transform_stamped.child_frame_id = dvs_frame_id_;
  transform_stamped.transform.translation.x = msg->pose.position.x;
  transform_stamped.transform.translation.y = msg->pose.position.y;
  transform_stamped.transform.translation.z = msg->pose.position.z;
  transform_stamped.transform.rotation = msg->pose.orientation;

  // Use a static transform broadcaster to publish the transform
  static tf2_ros::TransformBroadcaster br(this);
  br.sendTransform(transform_stamped);

  // Also set it in the buffer for lookups
  tf_buffer_->setTransform(transform_stamped, "esvo2_tracking", false);
}

bool
esvo2_Tracking::getPoseAt(
  const rclcpp::Time &t, esvo2_core::Transformation &Tr, const std::string &source_frame)
{
  try
  {
    if(!tf_buffer_->canTransform(world_frame_id_, source_frame, t, rclcpp::Duration::from_seconds(0.0)))
    {
      LOG(WARNING) << t.nanoseconds() << " : Cannot transform";
      return false;
    }
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(world_frame_id_, source_frame, t);
    // Convert geometry_msgs::msg::TransformStamped to tf2::Transform
    tf2::Transform tf2_transform;
    tf2_transform.setOrigin(tf2::Vector3(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z));
    tf2_transform.setRotation(tf2::Quaternion(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w));
    tf::transformTFToKindr(tf2_transform, &Tr);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    LOG(WARNING) << t.nanoseconds() << " : " << ex.what();
    return false;
  }
}

/************ publish results *******************/
void esvo2_Tracking::publishPose(const rclcpp::Time &t, Transformation &tr)
{
  geometry_msgs::msg::PoseStamped ps;

  ps.header.stamp = t;
  ps.header.frame_id = world_frame_id_;
  ps.pose.position.x = tr.getPosition()(0);
  ps.pose.position.y = tr.getPosition()(1);
  ps.pose.position.z = tr.getPosition()(2);
  ps.pose.orientation.x = tr.getRotation().x();
  ps.pose.orientation.y = tr.getRotation().y();
  ps.pose.orientation.z = tr.getRotation().z();
  ps.pose.orientation.w = tr.getRotation().w();
  pose_pub_->publish(ps);
  if(!resultPath_.empty())
  {
    std::ofstream f;
    f.open(resultPath_ + "stamped_traj_estimate_ours.txt", std::ofstream::app);
    f << std::fixed;
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    f.setf(std::ios::fixed, std::ios::floatfield);
    f.precision(9);
    f << t.seconds() << " ";
    f.precision(5);
    f << ps.pose.position.x << " "
    << ps.pose.position.y << " "
    << ps.pose.position.z << " "
    << ps.pose.orientation.x << " "
    << ps.pose.orientation.y << " "
    << ps.pose.orientation.z << " "
    << ps.pose.orientation.w << endl;
    f.close();
  }
}

void esvo2_Tracking::publishPath(const rclcpp::Time& t, Transformation& tr)
{
  geometry_msgs::msg::PoseStamped ps;

  ps.header.stamp = t;
  ps.header.frame_id = world_frame_id_;
  ps.pose.position.x = tr.getPosition()(0);
  ps.pose.position.y = tr.getPosition()(1);
  ps.pose.position.z = tr.getPosition()(2);
  ps.pose.orientation.x = tr.getRotation().x();
  ps.pose.orientation.y = tr.getRotation().y();
  ps.pose.orientation.z = tr.getRotation().z();
  ps.pose.orientation.w = tr.getRotation().w();
  path_.header.stamp = t;
  path_.header.frame_id = world_frame_id_;
  path_.poses.push_back(ps);
  path_pub_->publish(path_);
}

void
esvo2_Tracking::saveTrajectory(std::string &resultDir)
{
  LOG(INFO) << "Saving trajectory to " << resultPath_ + "stamped_traj_estimate.txt" << " ......";

  std::ofstream f;
  f.open(resultPath_ + "stamped_traj_estimate.txt", std::ofstream::app);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << resultPath_ + "stamped_traj_estimate.txt" << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;

  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_begin = lPose_.begin();
  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_end = lPose_.end();
  std::list<std::string>::iterator  ts_it_begin = lTimestamp_.begin();

  for(;result_it_begin != result_it_end; result_it_begin++, ts_it_begin++)
  {
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    Rwc_result = (*result_it_begin).block<3,3>(0,0);
    twc_result = (*result_it_begin).block<3,1>(0,3);
    Eigen::Quaterniond q(Rwc_result);
    f.setf(std::ios::fixed, std::ios::floatfield);
    f.precision(9);
    f << *ts_it_begin << " ";
    f.precision(5);
    f << twc_result.transpose().x() << " "
    << twc_result.transpose().y() << " "
    << twc_result.transpose().z() << " "
    << q.x() << " "
    << q.y() << " "
    << q.z() << " "
    << q.w() << endl;
  }
  f.close();
  LOG(INFO) << "Saving trajectory to " << resultPath_ + "stamped_traj_estimate.txt" << ". Done !!!!!!.";
}

void esvo2_Tracking::renameOldTraj()
{
  string ori_name = resultPath_ + "stamped_traj_estimate_ours.txt";
  string new_name = resultPath_ + "traj_ours_old.txt";
  if(std::rename(ori_name.c_str(), new_name.c_str()) == 0)
  {
    LOG(INFO) << "\33[32m" << "File renamed successfully." << "\33[0m";
  }
  else
  {
    LOG(INFO) << "\33[33m" << "Failed to rename the file." << "\33[0m";
  }
}

void esvo2_Tracking::groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::ofstream  f;
  f.open("/tmp/esvo2/stamped_groundtruth.txt", std::ofstream::app);
  f << std::fixed;
  f.setf(std::ios::fixed, std::ios::floatfield);
  f.precision(9);
  f << rclcpp::Time(msg->header.stamp).seconds() << " ";
  f.precision(5);
  f << msg->pose.position.x << " "
  << msg->pose.position.y << " "
  << msg->pose.position.z << " "
  << msg->pose.orientation.x << " "
  << msg->pose.orientation.y << " "
  << msg->pose.orientation.z << " "
  << msg->pose.orientation.w << endl;
  f.close();
}

Eigen::Matrix3d esvo2_Tracking::fixRotationMatrix(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    return U * V.transpose();
}

}// namespace esvo2_core
