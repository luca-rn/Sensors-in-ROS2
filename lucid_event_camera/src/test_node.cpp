// arena_event_publisher_node.cpp
// Build with: add to your CMake (you said you'll handle CMake).
// Requires: Arena SDK headers/libraries + rclcpp + event_camera_msgs

#include <rclcpp/rclcpp.hpp>
#include "event_camera_msgs/msg/event_packet.hpp"

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <cstring>   // memcpy
#include <memory>

// === Arena SDK includes ===
// The exact include path can differ by Arena SDK version / installation.
// Common variants:
//   #include <Arena/Arena.h>
//   #include "Arena.h"
// If your SDK uses a different include, replace the line below accordingly.
#include <Arena/Arena.h>   // <-- adjust if your SDK path is different

using namespace std::chrono_literals;

class ArenaEventPublisher : public rclcpp::Node {
public:
  ArenaEventPublisher()
  : Node("arena_event_publisher"),
    running_(true)
  {
    // Parameters (can be overridden via ROS params)
    this->declare_parameter<int>("batch_ms", 5);
    this->declare_parameter<std::string>("topic", "/events/raw_packets");
    this->declare_parameter<int>("queue_size", 10);
    this->get_parameter("batch_ms", batch_ms_);
    this->get_parameter("topic", topic_);
    this->get_parameter("queue_size", queue_size_);

    RCLCPP_INFO(this->get_logger(), "ArenaEventPublisher: batch_ms=%d topic=%s", batch_ms_, topic_.c_str());

    // Publisher
    pub_ = this->create_publisher<event_camera_msgs::msg::EventPacket>(topic_, rclcpp::QoS(queue_size_).best_effort());

    // Initialize Arena System and device
    try {
      arena::OpenSystem();
      // Wait for devices and create first device
      devices_ = arena::System::CreateInstance()->GetDevices();
      if (devices_.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "No Arena devices found. Make sure Arena SDK can see your camera.");
        throw std::runtime_error("No devices");
      }
      // Use first device
      device_ = std::move(devices_[0]);
      RCLCPP_INFO(this->get_logger(), "Opened device: %s", device_->GetConfig()->GetString("DeviceModelName").c_str());

      // Configure device for event streaming if needed.
      // Many Lucid event cameras expose GenICam nodes to enable/disable event stream.
      // You may need to set relevant node values here; sample below is commented:
      //
      // auto nodeMap = device_->GetNodeMap();
      // if (nodeMap->IsReadable("TLParamsLocked")) { ... }
      //
      // Start streaming
      device_->StartStream();

    } catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Arena initialization failed: %s", ex.what());
      throw;
    }

    // Start worker thread to pull buffers
    worker_ = std::thread(&ArenaEventPublisher::bufferWorker, this);

    // Start batch timer
    batch_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(batch_ms_),
      std::bind(&ArenaEventPublisher::onBatchTimer, this)
    );
  }

  ~ArenaEventPublisher() override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    try {
      if (device_) {
        device_->StopStream();
      }
      arena::CloseSystem();
    } catch (...) {}
  }

private:
  // Worker that continuously gets buffers from Arena device
  void bufferWorker() {
    // Each SDK has different buffer API names. The code below relies on Arena C++ wrapper:
    //   auto buffer = device_->GetBuffer(timeout_ms);
    //   buffer->GetData(); buffer->GetDataSize(); buffer->GetTimestamp();
    // If your SDK uses different method names, adapt accordingly.
    while (running_.load()) {
      try {
        // Timeout 1000 ms to allow clean shutdown checks
        arena::Buffer* buf = device_->GetBuffer(1000); // Adjust timeout method as necessary

        if (!buf) {
          continue;
        }

        // Acquire packet data pointer and size
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(buf->GetData());
        size_t data_size = buf->GetDataSize();

        // camera timestamp if available (nanoseconds or device unit)
        uint64_t cam_ts = 0;
        if (buf->HasTimestamp()) {
          cam_ts = buf->GetTimestamp(); // adapt name if your SDK uses different function
        }

        // Copy into local vector
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          // Append a small header per packet (optional) so decoder can find packet boundaries:
          // [uint64_t: camera timestamp][uint32_t: packet size][data bytes...]
          uint64_t ts_le = cam_ts;
          uint32_t size32 = static_cast<uint32_t>(data_size);

          // append timestamp (8 bytes)
          const uint8_t* p_ts = reinterpret_cast<const uint8_t*>(&ts_le);
          packet_buffer_.insert(packet_buffer_.end(), p_ts, p_ts + sizeof(ts_le));

          // append size (4 bytes)
          const uint8_t* p_sz = reinterpret_cast<const uint8_t*>(&size32);
          packet_buffer_.insert(packet_buffer_.end(), p_sz, p_sz + sizeof(size32));

          // append raw packet bytes
          packet_buffer_.insert(packet_buffer_.end(), data_ptr, data_ptr + data_size);
        }

        // release buffer back to SDK
        device_->RequeueBuffer(buf); // or buf->Requeue(); or device_->QueueBuffer(buf); check SDK
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Exception in bufferWorker: %s", e.what());
        // small sleep to avoid busy-loop on repeated errors
        std::this_thread::sleep_for(50ms);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Unknown exception in bufferWorker");
        std::this_thread::sleep_for(50ms);
      }
    }
  }

  // Called every batch_ms_ to publish whatever has accumulated
  void onBatchTimer() {
    std::vector<uint8_t> to_publish;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (packet_buffer_.empty()) return;
      to_publish.swap(packet_buffer_);  // efficient move out; packet_buffer_ becomes empty
    }

    auto msg = event_camera_msgs::msg::EventPacket();
    // timestamp the packet with ROS clock now (or optionally use camera timestamp of first item)
    msg.header.stamp = this->now();
    msg.header.frame_id = "event_camera";   // adjust if you need a specific frame

    // metadata
    msg.encoding = "lucid_evma";  // custom tag to indicate format
    msg.width = static_cast<uint32_t>(0);  // set to camera width if known: fill in if you can
    msg.height = static_cast<uint32_t>(0);

    // place raw data
    msg.raw_events = std::move(to_publish);

    pub_->publish(std::move(msg));
  }

  // Members
  rclcpp::Publisher<event_camera_msgs::msg::EventPacket>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr batch_timer_;
  std::thread worker_;
  std::atomic<bool> running_;
  std::mutex queue_mutex_;
  std::vector<uint8_t> packet_buffer_;
  int batch_ms_{5};
  std::string topic_;
  int queue_size_{10};

  // Arena handles
  std::unique_ptr<arena::Device> device_;
  std::vector<std::unique_ptr<arena::Device>> devices_;
};

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ArenaEventPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Fatal exception in ArenaEventPublisher: %s", ex.what());
    return 1;
  }
  return 0;
}
