#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class SubProcessorNode : public rclcpp::Node
{
public:
  SubProcessorNode() : Node("sub_processor")
  {
    RCLCPP_INFO(this->get_logger(), "=== SubProcessorNode initialization started ===");

    try {
      // サブ処理用のトリガーサービスサーバーを作成
      RCLCPP_DEBUG(this->get_logger(), "Creating sub trigger service server...");
      sub_trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "sub_trigger", [this](
                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          this->handle_sub_trigger(request, response);
        });
      RCLCPP_INFO(this->get_logger(), "Sub trigger service server created successfully");

      // メインのトリガーサービスクライアントを作成
      RCLCPP_DEBUG(this->get_logger(), "Creating main trigger service client...");
      main_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("main_trigger");
      RCLCPP_INFO(this->get_logger(), "Main trigger service client created successfully");

      RCLCPP_INFO(
        this->get_logger(), "=== SubProcessorNode initialization completed successfully ===");
      RCLCPP_INFO(this->get_logger(), "Sub trigger service is ready and listening");

      // スピン開始後に初回メイン・トリガー呼び出しを行うワンショットタイマー
      initial_call_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
        RCLCPP_INFO(this->get_logger(), "=== Executing initial main trigger service call ===");
        bool success = this->call_main_trigger();
        if (success) {
          RCLCPP_INFO(
            this->get_logger(), "Initial call to main trigger service completed successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Initial call to main trigger service failed");
        }
        // 一度きりの実行にする
        initial_call_timer_->cancel();
      });
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize SubProcessorNode: %s", e.what());
      throw;
    } catch (...) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize SubProcessorNode: Unknown error");
      throw;
    }
  }

  // メインのトリガーサービスを呼び出すメソッド
  bool call_main_trigger()
  {
    RCLCPP_INFO(this->get_logger(), "=== Starting main trigger service call ===");

    try {
      // サービスが利用可能になるまで待つ
      RCLCPP_DEBUG(this->get_logger(), "Waiting for main trigger service to become available...");
      int wait_count = 0;
      const int max_wait_attempts = 30;  // 30秒まで待機

      while (!main_trigger_client_->wait_for_service(std::chrono::seconds(1))) {
        wait_count++;
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            this->get_logger(), "Interrupted while waiting for main trigger service (attempt %d)",
            wait_count);
          return false;
        }
        if (wait_count >= max_wait_attempts) {
          RCLCPP_ERROR(
            this->get_logger(), "Timeout waiting for main trigger service after %d attempts",
            max_wait_attempts);
          return false;
        }
        RCLCPP_WARN(
          this->get_logger(),
          "Waiting for main trigger service to become available... (attempt %d/%d)", wait_count,
          max_wait_attempts);
      }

      RCLCPP_INFO(
        this->get_logger(), "Main trigger service is now available after %d attempts", wait_count);

      // リクエストを送信
      RCLCPP_DEBUG(this->get_logger(), "Sending request to main trigger service...");
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = main_trigger_client_->async_send_request(request);
      RCLCPP_DEBUG(this->get_logger(), "Request sent, waiting for response with timeout...");

      // 既存のエグゼキュータにノードが追加済みのため、spin_until_future_completeは使用しない
      auto wait_status = future.wait_for(std::chrono::seconds(10));

      if (wait_status == std::future_status::ready) {
        RCLCPP_DEBUG(this->get_logger(), "Future ready, getting result...");
        auto result = future.get();

        if (result->success) {
          RCLCPP_INFO(
            this->get_logger(), "Main trigger service call successful: %s",
            result->message.c_str());
          RCLCPP_INFO(
            this->get_logger(), "=== Main trigger service call completed successfully ===");
          return true;
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "Main trigger service call returned failure: %s",
            result->message.c_str());
          RCLCPP_ERROR(
            this->get_logger(), "=== Main trigger service call failed with service error ===");
          return false;
        }
      } else if (wait_status == std::future_status::timeout) {
        RCLCPP_ERROR(
          this->get_logger(), "Main trigger service call timed out while waiting for response");
        RCLCPP_ERROR(this->get_logger(), "=== Main trigger service call failed due to timeout ===");
        return false;
      } else {  // std::future_status::deferred など
        RCLCPP_ERROR(
          this->get_logger(), "Main trigger service call failed with unexpected future status");
        RCLCPP_ERROR(
          this->get_logger(),
          "=== Main trigger service call failed due to unexpected future status ===");
        return false;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during main trigger service call: %s", e.what());
      RCLCPP_ERROR(this->get_logger(), "=== Main trigger service call failed due to exception ===");
      return false;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception during main trigger service call");
      RCLCPP_ERROR(
        this->get_logger(), "=== Main trigger service call failed due to unknown exception ===");
      return false;
    }
  }

private:
  void handle_sub_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "=== Sub trigger service request received ===");

    try {
      // リクエストの詳細をログ出力
      RCLCPP_DEBUG(this->get_logger(), "Processing sub trigger request...");

      // 簡単な処理を実行
      response->success = true;
      response->message = "Sub trigger completed successfully";

      RCLCPP_INFO(this->get_logger(), "Sub trigger processing completed successfully");
      RCLCPP_INFO(
        this->get_logger(), "Response: success=%s, message='%s'",
        response->success ? "true" : "false", response->message.c_str());
      RCLCPP_INFO(this->get_logger(), "=== Sub trigger service request completed successfully ===");

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during sub trigger processing: %s", e.what());
      response->success = false;
      response->message = "Sub trigger failed due to exception: " + std::string(e.what());
      RCLCPP_ERROR(
        this->get_logger(), "=== Sub trigger service request failed due to exception ===");
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception during sub trigger processing");
      response->success = false;
      response->message = "Sub trigger failed due to unknown exception";
      RCLCPP_ERROR(
        this->get_logger(), "=== Sub trigger service request failed due to unknown exception ===");
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sub_trigger_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr main_trigger_client_;
  rclcpp::TimerBase::SharedPtr initial_call_timer_;
};

int main(int argc, char ** argv)
{
  RCLCPP_INFO(rclcpp::get_logger("main"), "=== Starting SubProcessorNode application ===");

  try {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ROS2...");
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS2 initialized successfully");

    RCLCPP_INFO(rclcpp::get_logger("main"), "Creating SubProcessorNode...");
    auto node = std::make_shared<SubProcessorNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "SubProcessorNode created successfully");

    // 初回メイン・トリガー呼び出しは、ノード内のワンショットタイマーによりスピン開始後に実施
    RCLCPP_INFO(
      rclcpp::get_logger("main"),
      "Initial main trigger call will be scheduled by node timer after spin starts");

    // マルチスレッドエグゼキュータでスピン
    RCLCPP_INFO(
      rclcpp::get_logger("main"),
      "Starting to spin SubProcessorNode with MultiThreadedExecutor...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    RCLCPP_INFO(rclcpp::get_logger("main"), "SubProcessorNode spin completed");

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
    rclcpp::get_logger("main"), "=== SubProcessorNode application terminated successfully ===");
  return 0;
}
