#ifndef image_representation_H_
#define image_representation_H_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <image_representation/TicToc.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>

#include <deque>
#include <mutex>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <thread>

#include <yaml-cpp/yaml.h>

namespace image_representation
{
  using EventQueue = std::deque<dvs_msgs::msg::Event>;

  struct RclcppTimeCmp
  {
    bool operator()(const rclcpp::Time &a, const rclcpp::Time &b) const
    {
      return a.nanoseconds() < b.nanoseconds();
    }
  };
  using GlobalEventQueue = std::map<rclcpp::Time, dvs_msgs::msg::Event, RclcppTimeCmp>;

  inline static EventQueue::iterator EventBuffer_lower_bound(
      EventQueue &eb, rclcpp::Time &t)
  {
    return std::lower_bound(eb.begin(), eb.end(), t,
                            [](const dvs_msgs::msg::Event &e, const rclcpp::Time &t)
                            { return rclcpp::Time(e.ts).seconds() < t.seconds(); });
  }

  inline static EventQueue::iterator EventBuffer_upper_bound(
      EventQueue &eb, rclcpp::Time &t)
  {
    return std::upper_bound(eb.begin(), eb.end(), t,
                            [](const rclcpp::Time &t, const dvs_msgs::msg::Event &e)
                            { return t.seconds() < rclcpp::Time(e.ts).seconds(); });
  }

  inline static std::vector<dvs_msgs::msg::Event>::iterator EventVector_lower_bound(
      std::vector<dvs_msgs::msg::Event> &ev, double &t)
  {
    return std::lower_bound(ev.begin(), ev.end(), t,
                            [](const dvs_msgs::msg::Event &e, const double &t)
                            { return rclcpp::Time(e.ts).seconds() < t; });
  }

  class ImageRepresentation : public rclcpp::Node
  {
  public:
    ImageRepresentation();
    virtual ~ImageRepresentation();

    static bool compare_time(const dvs_msgs::msg::Event &e, const double reference_time)
    {
      return reference_time < rclcpp::Time(e.ts).seconds();
    }

  private:
    // core
    void init(int width, int height);
    // Support: TS, AA, negative_TS, negative_TS_dx, negative_TS_dy
    void createImageRepresentationAtTime(const rclcpp::Time &external_sync_time);
    void GenerationLoop();

    // callbacks
    void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg);

    // utils
    void clearEventQueue();
    bool loadCalibInfo(const std::string &cameraSystemDir, bool &is_left);
    void clearEvents(int distance, std::vector<dvs_msgs::msg::Event>::iterator ptr_e);

    void AA_thread(std::vector<dvs_msgs::msg::Event>::iterator &ptr_e, int distance, double external_t);
    void sobel(double external_t);
    bool fileExists(const std::string &filename);
    // tests

    // calibration parameters
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Mat rectification_matrix_, projection_matrix_;
    std::string distortion_model_;
    cv::Mat undistort_map1_, undistort_map2_;
    Eigen::Matrix2Xd precomputed_rectified_points_;

    // sub & pub
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    image_transport::Publisher dx_image_pub_, dy_image_pub_;
    image_transport::Publisher image_representation_pub_TS_;
    image_transport::Publisher image_representation_pub_negative_TS_;
    image_transport::Publisher image_representation_pub_AA_frequency_;
    image_transport::Publisher image_representation_pub_AA_mat_;

    bool left_;
    cv::Mat negative_TS_img;
    cv_bridge::CvImage cv_dx_image, cv_dy_image;
    std::thread thread_sobel;

    // online parameters
    bool bCamInfoAvailable_;
    bool bUse_Sim_Time_;
    cv::Size sensor_size_;
    rclcpp::Time sync_time_;
    bool bSensorInitialized_;

    // offline parameters TODO
    double decay_ms_;
    bool ignore_polarity_;
    int median_blur_kernel_size_;
    int blur_size_;
    int max_event_queue_length_;
    int events_maintained_size_;

    // containers
    EventQueue events_;

    std::vector<dvs_msgs::msg::Event> vEvents_;

    cv::Mat representation_TS_;
    cv::Mat representation_AA_;

    Eigen::MatrixXd TS_temp_map;

    // for rectify
    cv::Mat undistmap1_, undistmap2_;
    bool is_left_, bcreat_;

    // thread mutex
    std::mutex data_mutex_;

    enum RepresentationMode
    {
      Linear_TS, // 0
      AA2,       // 1
      Fast       // 2
    } representation_mode_;

    // parameters
    bool bUseStereoCam_;
    double decay_sec_; // TS param
    int generation_rate_hz_;
    int x_patches_, y_patches_;
    // std::vector<dvs_msgs::msg::Event>::iterator ptr_e_;

    // calib info
    std::string calibInfoDir_;
    std::vector<cv::Point> trapezoid_;
  };
} // namespace image_representation
#endif // image_representation_H_
