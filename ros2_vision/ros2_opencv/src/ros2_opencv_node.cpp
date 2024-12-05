#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
using std::placeholders::_1;
//using namespace cv;

 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {


    // Soglia di colori, 0 indica nero, 255 indica bianco
    params_.minThreshold = 10;
    params_.maxThreshold = 200;
    
    // Filter by Area, intende l'area dei pixel
    params_.filterByArea = true;
    params_.minArea = 1500;
    params_.maxArea = 100000;
    
    // Filter by Circularity
    params_.filterByCircularity = true;
    params_.minCircularity = 0.1;
    
    // Filter by Convexity
    params_.filterByConvexity = true;
    params_.minConvexity = 0.87;
    
    // Filter by Inertia
    params_.filterByInertia = true;
    params_.minInertiaRatio = 0.01;
    
    subscriber_=
        this->create_subscription<sensor_msgs::msg::Image>(
          "/camera", 10, std::bind(&MinimalImagePublisher::subscriber, this, _1));
    publisher_=
        this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));

  }

 
private:
  
    void subscriber(const sensor_msgs::msg::Image & camera_msg) //Secondo me qua dentro va messo msg_, se vedete l'api mi pare ci sia un puntatore, verificate
    {
      cv_ptr_ = cv_bridge::toCvCopy(camera_msg,"bgr8");
      cv_img_ = *cv_ptr_;
    }


    void timer_callback() {
    // Create a new 640x480 image
    /*cv::Mat my_image(cv::Size(640, 480), CV_8UC3);
 
    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
 
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %l;d published", count_);
    count_++;*/



    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params_);


    detector->detect( cv_img_.image, keypoints);

    cv::Mat im_with_keypoints;
    cv::drawKeypoints( cv_img_.image, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
    // Show blobs
    //cv::imshow("keypoints", im_with_keypoints);
    //cv::waitKey(0);


//  msg_=cv_ptr_->toImageMsg();
    
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();

    publisher_->publish(*msg_.get());
    
  }



  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params_;
 
  

  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  cv_bridge::CvImagePtr cv_ptr_;
  cv_bridge::CvImage cv_img_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
