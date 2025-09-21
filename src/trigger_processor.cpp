#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MainProcessorNode : public rclcpp::Node
{
public:
  MainProcessorNode() : Node("main_processor_node")
  {
    RCLCPP_INFO(this->get_logger(), "MainProcessorNode starting...");

    main_service_ = this->create_service<std_srvs::srv::Trigger>(
      "main_trigger", [this](
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handle_main_trigger(request, response);
      });

    sub_client_ = this->create_client<std_srvs::srv::Trigger>("sub_trigger");

    RCLCPP_INFO(this->get_logger(), "MainProcessorNode initialized successfully");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr main_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sub_client_;

  void handle_main_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Main trigger request received");

    if (!sub_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Sub trigger service unavailable");
      response->success = false;
      response->message = "Sub trigger service unavailable";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sub trigger service available");

    auto sub_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = sub_client_->async_send_request(sub_request);

    auto wait_status = future.wait_for(std::chrono::seconds(10));

    if (wait_status == std::future_status::ready) {
      auto result = future.get();

      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Main trigger completed successfully");
        response->success = true;
        response->message = "Main trigger completed successfully";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Sub trigger failed: %s", result->message.c_str());
        response->success = false;
        response->message = "Sub trigger failed: " + result->message;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Sub trigger service call failed");
      response->success = false;
      response->message = "Sub trigger service call failed";
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainProcessorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
