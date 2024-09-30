#include <chrono>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

// Constants
const int SCREEN_WIDTH = 640;

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node"), current_angle_(0.0)
    {
        // Initialize publishers and subscribers
        pub_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);
        
        // Image subscription with custom QoS
        sub_ = image_transport::create_subscription(
            this, "robotcam",
            std::bind(&MyNode::image_callback, this, std::placeholders::_1), 
            "raw");

        // Subscribe to the /current_angle topic
        current_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle", 10,
            std::bind(&MyNode::current_angle_callback, this, std::placeholders::_1));
    }

private:
    // Callback for receiving images
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing an image");
        process_image(msg); // Process the received image
    }

    // Callback to receive the current angle
    void current_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received current angle: %f", msg->data);
        current_angle_ = msg->data;
    }

    // Image processing logic
    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to HSV and apply color threshold for red
        cv::Mat hsv_image, mask;
        const cv::Scalar LowerBound(0, 120, 70);
        const cv::Scalar UpperBound(10, 255, 255);
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, LowerBound, UpperBound, mask);

        // Calculate moments and centroid
        cv::Moments m = cv::moments(mask, true);
        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

        // Calculate desired angle based on the centroid
        float diff_angle = -1.0f * ((center.x - (SCREEN_WIDTH / 2)) * (M_PI / (SCREEN_WIDTH * 2)));
        float desired_angle = current_angle_ + diff_angle;

        // Log calculated values
        RCLCPP_INFO(this->get_logger(), "Centroid: (%d, %d), diff_angle: %f, desired_angle: %f", center.x, center.y, diff_angle, desired_angle);

        // Publish desired angle or reset to zero if the box is lost
        auto angle_msg = std::make_shared<std_msgs::msg::Float32>();
        if (center.x == 0 && center.y == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Lost the box, resetting angle.");
            angle_msg->data = 0.0;
        }
        else
        {
            angle_msg->data = desired_angle;
        }
        
        pub_->publish(*angle_msg);
        RCLCPP_INFO(this->get_logger(), "Published desired angle: %f", angle_msg->data);
    }

    // Member variables
    image_transport::Subscriber sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_angle_sub_;
    double current_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
