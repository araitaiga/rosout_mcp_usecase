#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class SubProcessorNode : public rclcpp::Node
{
public:
  SubProcessorNode() : Node("sub_processor_node")
  {
    RCLCPP_INFO(this->get_logger(), "SubProcessorNode starting...");

    sub_service_ = this->create_service<std_srvs::srv::Trigger>(
      "sub_trigger", [this](
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handle_sub_trigger(request, response);
      });

    main_client_ = this->create_client<std_srvs::srv::Trigger>("main_trigger");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() {
      RCLCPP_INFO(this->get_logger(), "Executing main trigger call");
      bool success = this->call_main_trigger();
      if (success) {
        RCLCPP_INFO(this->get_logger(), "Main trigger call successful");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Main trigger call failed");
      }
    });

    RCLCPP_INFO(this->get_logger(), "SubProcessorNode initialized successfully");
  }

  bool call_main_trigger()
  {
    RCLCPP_INFO(this->get_logger(), "Calling main service");

    int wait_count = 0;
    const int max_wait_attempts = 30;

    while (rclcpp::ok() && !main_client_->wait_for_service(std::chrono::seconds(1))) {
      wait_count++;
      if (wait_count >= max_wait_attempts) {
        RCLCPP_ERROR(
          this->get_logger(), "Main service unavailable after %d attempts", max_wait_attempts);
        return false;
      }
      if (wait_count % 5 == 0) {
        RCLCPP_WARN(
          this->get_logger(), "Waiting for main service... (%d/%d)", wait_count, max_wait_attempts);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Main service available after %d attempts", wait_count);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = main_client_->async_send_request(request);

    auto wait_status = future.wait_for(std::chrono::seconds(10));

    if (wait_status == std::future_status::ready) {
      auto result = future.get();

      if (result->success) {
        RCLCPP_INFO(
          this->get_logger(), "Main trigger call successful: %s", result->message.c_str());
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Main trigger call failed: %s", result->message.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Main call failed");
      return false;
    }
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sub_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr main_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void handle_sub_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Sub trigger request received");

    response->success = true;
    response->message = "Sub trigger completed successfully";

    RCLCPP_INFO(this->get_logger(), "Sub trigger completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubProcessorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
