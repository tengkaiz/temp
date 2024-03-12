#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber_node")
    {
        // 订阅/image话题
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV图像
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::);
            // cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::jpeg)->image; // 显示图像
            cv::imshow("Received Image", cv_image);
            cv::waitKey(0); // 等待1毫秒以更新窗口
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'jpeg'.", msg->encoding.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
