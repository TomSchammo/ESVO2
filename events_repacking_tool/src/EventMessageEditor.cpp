#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "sensor_msgs/Imu.h"
#include <chrono>
struct EventMessageEditor
{
  EventMessageEditor(
    double frequency,
    std::string & messageTopic)
    :bFirstMessage_(true)
  {
    eArray_ = dvs_msgs::msg::EventArray();
    duration_threshold_ = 1 / frequency;
    message_topic_ = messageTopic;
  }

  void resetBuffer(rclcpp::Time startTimeStamp)
  {
    start_time_ = startTimeStamp;
    end_time_ = start_time_ + rclcpp::Duration::from_seconds(duration_threshold_);
    eArray_.events.clear();
    eArray_.header.stamp = end_time_;
  }

  void resetArraySize(size_t width, size_t height)
  {
    eArray_.width  = width;
    eArray_.height = height;
  }

  void insertEvent(
    dvs_msgs::msg::Event & e,
    rosbag::Bag* bag)
  {
    if(bFirstMessage_)
    {
      resetBuffer(e.ts);
      bFirstMessage_ = false;
    }

    if(rclcpp::Time(e.ts).seconds() >= end_time_.seconds())
    {
      bag->write(message_topic_.c_str(), eArray_.header.stamp, eArray_);
      resetBuffer(end_time_);
    }
    eArray_.events.push_back(e);
  }

  // variables
  dvs_msgs::msg::EventArray eArray_;
  double duration_threshold_;
  rclcpp::Time start_time_, end_time_;
  bool bFirstMessage_;
  std::string message_topic_;
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "event_message_editor");
  auto start = std::chrono::system_clock::now();
  std::string src_bag_path(argv[1]);
  std::string src_bag_path2(argv[2]);
  std::string dst_bag_path(argv[3]);
  rosbag::Bag bag_src, bag_src2, bag_dst;
  bag_src.open(src_bag_path.c_str(), rosbag::bagmode::Read);
  bag_dst.open(dst_bag_path.c_str(), rosbag::bagmode::Write);

  sensor_msgs::msg::Imu imu_buff;

  if(!bag_src.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "No rosbag is found in the give path.");
    exit(-1);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "***********Input Bag File Name ***********");
    RCLCPP_INFO(this->get_logger(), argv[1]);
    RCLCPP_INFO(this->get_logger(), argv[2]);
    RCLCPP_INFO(this->get_logger(), "******************************************");
  }

  if(!bag_dst.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "The dst bag is not opened.");
    exit(-1);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "***********Output Bag File Name ***********");
    RCLCPP_INFO(this->get_logger(), argv[3]);
    RCLCPP_INFO(this->get_logger(), "***************************");
  }
  rclcpp::Time start_time;
  rclcpp::Time end_time;

  const double frequency = 1000;
  int nums = 0;
  // process events
  std::vector<std::string> topics;
  // the topic name in the input bag file
  topics.push_back(std::string("/davis/left/events"));
  topics.push_back(std::string("/davis/right/events"));
  std::vector<std::string> topics_rename;
  // the topic name in the output bag file
  topics_rename.push_back(std::string("/davis/left/events"));
  topics_rename.push_back(std::string("/davis/right/events"));
  for(size_t i = 0;i < topics.size(); i++)
  {
    rosbag::View view(bag_src, rosbag::TopicQuery(topics[i]));
    EventMessageEditor eArrayEditor(frequency, topics_rename[i]);
    // TODO getBeginTime is ros1?
    start_time = rclcpp::Time(static_cast<int64_t>(view.getBeginTime().toSec() * 1e9));
    end_time = rclcpp::Time(static_cast<int64_t>((view.getEndTime().toSec()+0.01) * 1e9));

    // topic loop
    for(rosbag::MessageInstance const m: view)
    {
      dvs_msgs::msg::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::msg::EventArray>();
      eArrayEditor.resetArraySize(msg->width, msg->height);
      // message loop
      for(dvs_msgs::msg::Event e : msg->events)
      {
        nums++;
        eArrayEditor.insertEvent(const_cast<dvs_msgs::msg::Event&>(e), &bag_dst);
      }
    }
  }

  bag_src.close();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  bag_src2.open(src_bag_path2.c_str(), rosbag::bagmode::Read);
  // the topic name in the input bag file
  rosbag::View view(bag_src2, rosbag::TopicQuery("/imu/data"));
  for(rosbag::MessageInstance const m: view)
  {
    sensor_msgs::msg::Imu::ConstPtr msg = m.instantiate<sensor_msgs::msg::Imu>();
    if(rclcpp::Time(msg->header.stamp).seconds() < start_time.seconds() || rclcpp::Time(msg->header.stamp).seconds() > end_time.seconds())
      continue;
    // the topic name in the output bag file
    bag_dst.write("/imu/data", msg->header.stamp, *msg);
  }


  bag_dst.close();
  return 0;
}


