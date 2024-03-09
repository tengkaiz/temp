#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"


//_______________Global Variables_______________
std::string TOPIC = "/image";
int FPS = 1;
int DEVICE_INDEX = 8;
cv::VideoCapture camera(DEVICE_INDEX);

//_______________Classes_______________
class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("image_publisher")
    {
        pub_ =
            this->create_publisher<sensor_msgs::msg::Image>(TOPIC, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / FPS), std::bind(&ImagePublisherNode::publisherCallback, this));
    }

private:
    void publisherCallback()
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat img;
        camera >> img;
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
                .toImageMsg();
        RCLCPP_INFO(this->get_logger(), "Publishing...");
        pub_->publish(*msg.get());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    
};

//_______________Main_______________
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}