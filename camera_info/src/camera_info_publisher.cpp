#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class CameraInformation : public rclcpp::Node{
public:
    CameraInformation() : Node("camera_info_publisher"), cim_(this){
        RCLCPP_INFO(this->get_logger(),"Camera publisher is activated!");

        pub_ci_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

        param_descr_camera_info_url.description = "camera calibration info file url";
        param_descr_camera_info_url.read_only = true;

        const std::string &camera_info_url = declare_parameter<std::string>(
            "camera_info_url", {}, param_descr_camera_info_url);

        cim_.setCameraName(name_);

        // --ros-pars -p camera_info_url:="file:///home/q/test.yaml"
        // --ros-args -p camera_info_url:="package://camera_info/config/test.yaml"
        if (!cim_.loadCameraInfo(camera_info_url)){
            RCLCPP_INFO(this->get_logger(), "Load camera error");
        }
        
        timer_ = this->create_wall_timer(100ms, std::bind(&CameraInformation::ci_callback, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci_;
    rclcpp::TimerBase::SharedPtr timer_;
    camera_info_manager::CameraInfoManager cim_;
    const std::string &name_ = "test";
    
    rcl_interfaces::msg::ParameterDescriptor param_descr_camera_info_url;

    void ci_callback(){
        sensor_msgs::msg::CameraInfo ci = cim_.getCameraInfo();
        pub_ci_->publish(ci);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInformation>());
    rclcpp::shutdown();
    return 0;
}