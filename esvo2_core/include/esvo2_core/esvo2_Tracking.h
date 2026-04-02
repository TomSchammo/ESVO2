#ifndef ESVO2_CORE_TRACKING_H
#define ESVO2_CORE_TRACKING_H

#include <nav_msgs/msg/path.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <esvo2_core/container/CameraSystem.h>
#include <esvo2_core/core/RegProblemLM.h>
#include <esvo2_core/core/RegProblemSolverLM.h>
#include <esvo2_core/tools/utils.h>
#include <esvo2_core/tools/Visualization.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <map>
#include <deque>
#include <mutex>
#include <future>
#include <vector>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <esvo2_core/factor/imu_integration.h>
#include <events_repacking_tool/msg/v_ba_bg.hpp>

namespace esvo2_core
{
  using namespace core;
  using namespace factor;
  enum TrackingStatus
  {
    IDLE,
    WORKING
  };

  class esvo2_Tracking : public rclcpp::Node
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    esvo2_Tracking();
    virtual ~esvo2_Tracking();

    // functions regarding tracking
    void TrackingLoop();
    bool refDataTransferring();
    bool curDataTransferring(); // These two data transferring functions are decoupled because the data are not updated at the same frequency.
    bool curImuTransferring();

    // topic callback functions
    void refMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void refImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void VBaBgCallback(const events_repacking_tool::msg::VBaBg::SharedPtr msg);
    void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timeSurface_NegaTS_Callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_left,
        const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_negative,
        const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_dx,
        const sensor_msgs::msg::Image::ConstSharedPtr &time_surface_dy);
    void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg);

    // results
    void publishPose(const rclcpp::Time &t, Transformation &tr);
    void publishPath(const rclcpp::Time &t, Transformation &tr);
    void saveTrajectory(std::string &resultDir);

    // utils
    void reset();
    void clearEventQueue();
    void stampedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool getPoseAt(
        const rclcpp::Time &t,
        esvo2_core::Transformation &Tr, // T_world_something
        const std::string &source_frame);
    void renameOldTraj();
    Eigen::Matrix3d fixRotationMatrix(const Eigen::Matrix3d &R);

  private:
    // subscribers and publishers
    // rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr events_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_, map_sub_for_tracking_visualization_;
    rclcpp::Subscription<events_repacking_tool::msg::VBaBg>::SharedPtr V_ba_bg_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> TS_left_sub_, TS_right_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> TS_negative_sub_, TS_dx_sub_, TS_dy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stampedPose_sub_;
    image_transport::Publisher reprojMap_pub_left_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // results
    nav_msgs::msg::Path path_;
    std::list<Eigen::Matrix<double, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 4>>> lPose_;
    std::list<std::string> lTimestamp_;

    // Time Surface sync policy
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproximateSyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> TS_sync_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproximateSyncPolicy_negaTS;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy_negaTS>> TS_negaTS_sync_;

    // offline data
    std::string dvs_frame_id_;
    std::string world_frame_id_;
    std::string calibInfoDir_;
    CameraSystem::Ptr camSysPtr_;

    // inter-thread management
    std::mutex data_mutex_;
    std::mutex imu_mutex_;

    // imu data
    IntegrationBase imu_data_;
    Eigen::Vector3d g_optimal{0, 9.81, 0};
    Eigen::Matrix3d R_b_c_;
    Eigen::Vector3d ba_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d bg_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Quaterniond Imu_q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d Imu_t = Eigen::Vector3d::Zero();

    // online data
    EventQueue events_left_;
    TimeSurfaceHistory TS_history_;
    size_t TS_id_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::map<rclcpp::Time, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr, tools::RclcppTimeCmp> refPCMap_;
    RefFrame ref_;
    CurFrame cur_;

    /**** offline parameters ***/
    size_t tracking_rate_hz_;
    size_t TS_HISTORY_LENGTH_;
    size_t REF_HISTORY_LENGTH_;
    bool bSaveTrajectory_;
    bool bVisualizeTrajectory_;
    bool bUseImu_;
    std::string resultPath_;

    Eigen::Matrix<double, 4, 4> T_world_ref_;
    Eigen::Matrix<double, 4, 4> T_world_cur_;

    Eigen::Vector3d t_world_cur_;
    Eigen::Vector3d last_t_world_cur_;
    Eigen::Vector3d last_t_;

    /*** system objects ***/
    RegProblemType rpType_;
    TrackingStatus ets_;
    // system status communication (ROS2 uses topics instead of global parameters)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    std::string ESVO2_System_Status_;
    RegProblemConfig::Ptr rpConfigPtr_;
    RegProblemSolverLM rpSolver_;
    bool initVsFlag;

    /*** for test ***/

    std::vector<Eigen::Vector3d> qprevious_ts_;
  };
}

#endif // ESVO2_CORE_TRACKING_H
