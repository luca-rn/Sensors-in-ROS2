#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "Arena/ArenaApi.h"

using namespace std::chrono_literals;

class LucidCameraModelPublisher : public rclcpp::Node
{
public:
    LucidCameraModelPublisher()
    : Node("lucid_camera_model_publisher"), camera_initialized_(false)
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("camera_model", 10);
        
        // Initialize Arena SDK and get camera info
        try
        {
            initializeCamera();
            
            // Create timer to publish every 2 seconds
            timer_ = this->create_wall_timer(
                2s, std::bind(&LucidCameraModelPublisher::publishCameraModel, this));
            
            RCLCPP_INFO(this->get_logger(), "Lucid Camera Model Publisher initialized");
            RCLCPP_INFO(this->get_logger(), "Camera Model: %s", camera_model_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera: %s", e.what());
        }
    }
    
    ~LucidCameraModelPublisher()
    {
        // Clean up Arena SDK resources
        if (camera_initialized_)
        {
            try
            {
                if (pDevice_)
                {
                    pSystem_->DestroyDevice(pDevice_);
                }
                if (pSystem_)
                {
                    Arena::CloseSystem(pSystem_);
                }
                RCLCPP_INFO(this->get_logger(), "Arena SDK cleaned up successfully");
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error during cleanup: %s", e.what());
            }
        }
    }

private:
    void initializeCamera()
    {
        // Open Arena system
        pSystem_ = Arena::OpenSystem();
        
        // Update device list
        pSystem_->UpdateDevices(100); // 100ms timeout
        
        // Get available devices
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
        
        if (deviceInfos.size() == 0)
        {
            throw std::runtime_error("No Lucid camera connected");
        }
        
        // Use first available camera
        RCLCPP_INFO(this->get_logger(), "Found %zu camera(s)", deviceInfos.size());
        RCLCPP_INFO(this->get_logger(), "Using first camera: %s (Serial: %s, IP: %s)",
                    deviceInfos[0].ModelName().c_str(),
                    deviceInfos[0].SerialNumber().c_str(),
                    deviceInfos[0].IpAddressStr().c_str());
        
        // Store camera model name
        camera_model_ = deviceInfos[0].ModelName();
        
        // Create device (optional - only if you need to access the device later)
        pDevice_ = pSystem_->CreateDevice(deviceInfos[0]);
        
        camera_initialized_ = true;
    }
    
    void publishCameraModel()
    {
        if (!camera_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Camera not initialized, skipping publish");
            return;
        }
        
        auto message = std_msgs::msg::String();
        message.data = camera_model_;
        
        publisher_->publish(message);
        RCLCPP_DEBUG(this->get_logger(), "Published camera model: %s", camera_model_.c_str());
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    Arena::ISystem* pSystem_;
    Arena::IDevice* pDevice_;
    std::string camera_model_;
    bool camera_initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try
    {
        auto node = std::make_shared<LucidCameraModelPublisher>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}