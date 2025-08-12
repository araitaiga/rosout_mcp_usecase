#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class TriggerProcessorNode : public rclcpp::Node
{
public:
  TriggerProcessorNode() : Node("trigger_processor")
  {
    RCLCPP_INFO(this->get_logger(), "=== TriggerProcessorNode initialization started ===");

    try {
      // メインのトリガーサービスサーバーを作成
      RCLCPP_DEBUG(this->get_logger(), "Creating main trigger service server...");
      main_trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "main_trigger", [this](
                          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          this->handle_main_trigger(request, response);
        });
      RCLCPP_INFO(this->get_logger(), "Main trigger service server created successfully");

      // サブ処理用のトリガーサービスクライアントを作成
      RCLCPP_DEBUG(this->get_logger(), "Creating sub trigger service client...");
      sub_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("sub_trigger");
      RCLCPP_INFO(this->get_logger(), "Sub trigger service client created successfully");

      RCLCPP_INFO(
        this->get_logger(), "=== TriggerProcessorNode initialization completed successfully ===");
      RCLCPP_INFO(this->get_logger(), "Main trigger service is ready and listening");
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize TriggerProcessorNode: %s", e.what());
      throw;
    } catch (...) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize TriggerProcessorNode: Unknown error");
      throw;
    }
  }

private:
  void handle_main_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "=== Main trigger service request received ===");

    try {
      RCLCPP_DEBUG(this->get_logger(), "Processing main trigger request...");

      // サブ処理用のトリガーサービスが利用可能かチェック
      RCLCPP_DEBUG(this->get_logger(), "Checking if sub trigger service is available...");
      if (!sub_trigger_client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Sub trigger service is not available within timeout");
        response->success = false;
        response->message = "Main trigger failed: sub trigger service unavailable";
        RCLCPP_ERROR(
          this->get_logger(), "=== Main trigger service request failed: service unavailable ===");
        return;
      }
      RCLCPP_DEBUG(this->get_logger(), "Sub trigger service is available");

      // サブ処理用のトリガーサービスを呼び出す
      RCLCPP_DEBUG(this->get_logger(), "Sending request to sub trigger service...");
      auto sub_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = sub_trigger_client_->async_send_request(sub_request);
      RCLCPP_DEBUG(
        this->get_logger(),
        "Request sent to sub trigger service, waiting for response with timeout...");

      // 既存のエグゼキュータにノードが追加済みのため、spin_until_future_completeは使用しない
      // マルチスレッドエグゼキュータでスピン中であれば、wait_forで応答を待てる
      auto wait_status = future.wait_for(std::chrono::seconds(10));

      if (wait_status == std::future_status::ready) {
        RCLCPP_DEBUG(this->get_logger(), "Sub trigger service future ready, getting result...");
        auto result = future.get();

        if (result->success) {
          RCLCPP_INFO(
            this->get_logger(), "Sub trigger service call successful: %s", result->message.c_str());
          response->success = true;
          response->message = "Main trigger completed successfully after calling sub trigger";
          RCLCPP_INFO(
            this->get_logger(), "=== Main trigger service request completed successfully ===");
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "Sub trigger service call returned failure: %s",
            result->message.c_str());
          response->success = false;
          response->message = "Main trigger failed because sub trigger failed: " + result->message;
          RCLCPP_ERROR(
            this->get_logger(),
            "=== Main trigger service request failed due to sub trigger failure ===");
        }
      } else if (wait_status == std::future_status::timeout) {
        RCLCPP_ERROR(
          this->get_logger(), "Sub trigger service call timed out while waiting for response");
        response->success = false;
        response->message = "Main trigger failed: sub trigger service call timed out";
        RCLCPP_ERROR(
          this->get_logger(), "=== Main trigger service request failed due to timeout ===");
      } else {  // std::future_status::deferred など
        RCLCPP_ERROR(
          this->get_logger(), "Sub trigger service call failed with unexpected future status");
        response->success = false;
        response->message = "Main trigger failed: unexpected future status";
        RCLCPP_ERROR(
          this->get_logger(),
          "=== Main trigger service request failed due to unexpected future status ===");
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during main trigger processing: %s", e.what());
      response->success = false;
      response->message = "Main trigger failed due to exception: " + std::string(e.what());
      RCLCPP_ERROR(
        this->get_logger(), "=== Main trigger service request failed due to exception ===");
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception during main trigger processing");
      response->success = false;
      response->message = "Main trigger failed due to unknown exception";
      RCLCPP_ERROR(
        this->get_logger(), "=== Main trigger service request failed due to unknown exception ===");
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr main_trigger_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sub_trigger_client_;
};

int main(int argc, char ** argv)
{
  RCLCPP_INFO(rclcpp::get_logger("main"), "=== Starting TriggerProcessorNode application ===");

  try {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ROS2...");
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS2 initialized successfully");

    RCLCPP_INFO(rclcpp::get_logger("main"), "Creating TriggerProcessorNode...");
    auto node = std::make_shared<TriggerProcessorNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "TriggerProcessorNode created successfully");

    // マルチスレッドエグゼキュータでスピン
    RCLCPP_INFO(
      rclcpp::get_logger("main"),
      "Starting to spin TriggerProcessorNode with MultiThreadedExecutor...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    RCLCPP_INFO(rclcpp::get_logger("main"), "TriggerProcessorNode spin completed");

  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error in main: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Unknown fatal error in main");
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down ROS2...");
  rclcpp::shutdown();
  RCLCPP_INFO(
    rclcpp::get_logger("main"), "=== TriggerProcessorNode application terminated successfully ===");
  return 0;
}
