#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    auto qos = rclcpp::QoS(1)
                   .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                   .history(rclcpp::HistoryPolicy::KeepLast)
                   .durability(rclcpp::DurabilityPolicy::Volatile);
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("uav_image", qos);
    cam_publisher_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("uav_camera", 10);
    pipe_ = popen(
        "ffmpeg -i rtmp://localhost/live/stream1 -f rawvideo -pix_fmt bgr24 -",
        "r");
    timer_ = create_wall_timer(std::chrono::milliseconds(40), [this]() {
      cv::Mat frame(720, 1280, CV_8UC3);
      fread(frame.data, 1, 1280 * 720 * 3, pipe_);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                     .toImageMsg();
      publisher_->publish(*msg);

      auto message = sensor_msgs::msg::CameraInfo();

      message.header.stamp = this->now();
      message.header.frame_id = "camera_optical_frame";

      // Image dimensions
      message.height = 720;
      message.width = 1280;

      // Distortion model and coefficients
      message.distortion_model = "plumb_bob";
      message.d = {0.1, -0.01, 0.001, 0.002, 0.0}; // Example values

      float f_x = (6.4f * 1280.0f) / 9.6f;
      float f_y = (6.4f * 720.0f) / 7.2f;
      float c_x = 1280.0f / 2.0f;
      float c_y = 720.0f / 2.0f;

      // Intrinsic matrix (3x3 row-major)
      message.k = {
          f_x, 0.0,   c_x, // fx, 0, cx
          0.0,   f_y, c_y, // 0, fy, cy
          0.0,   0.0,   1.0    // Affine parameters
      };

      // Projection matrix (3x4)
      message.p = {
          f_x, 0.0, c_x, 0.0,
          0.0, f_y, c_y, 0.0,
          0.0,   0.0, 1.0, 0.0};

      cam_publisher_->publish(message);

      cv::imshow("RTMP Stream", frame);
      cv::waitKey(1);
    });
  }

  ~MyNode() { pclose(pipe_); }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  FILE *pipe_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
