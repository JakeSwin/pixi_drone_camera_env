#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    auto qos = rclcpp::QoS(1)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .history(rclcpp::HistoryPolicy::KeepLast)
        .durability(rclcpp::DurabilityPolicy::Volatile);
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("uav_image", qos);
    pipe_ = popen(
        "ffmpeg -i rtmp://localhost/live/stream1 -f rawvideo -pix_fmt bgr24 -", "r"
    );
    timer_ = create_wall_timer(
      std::chrono::milliseconds(40),
      [this]() {
          cv::Mat frame(720, 1280, CV_8UC3);
          fread(frame.data, 1, 1280*720*3, pipe_);
          auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
          publisher_->publish(*msg);
          cv::imshow("RTMP Stream", frame);
          cv::waitKey(1);
      }
    );
  }

  ~MyNode() {
    pclose(pipe_);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  FILE* pipe_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
