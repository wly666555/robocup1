#pragma once
#include <cstdint>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include <behaviortree_cpp/contrib/json.hpp>
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr req_suber_;  // 修正类型

public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
            std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))) {}

  int32_t Call(Request req, nlohmann::json& js) {
    std::promise<const std::shared_ptr<const Response>> response_promise;
    auto response_future = response_promise.get_future();
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;

    // 临时订阅，不用成员变量
    auto temp_suber = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1),
        [&response_promise,
         identity_id](const std::shared_ptr<const Response> data) {
          if (data->header.identity.id == identity_id) {
            response_promise.set_value(data);
          }
        });

    req_puber_->publish(req);
    auto status = response_future.wait_for(std::chrono::seconds(5));

    Response response;
    if (status == std::future_status::ready) {
      response = *response_future.get();
      if (response.header.status.code != 0) {
        std::cout << "error code: " << response.header.status.code << std::endl;
        return response.header.status.code;
      }
      try {
        js = nlohmann::json::parse(response.data.data());
      } catch (nlohmann::detail::exception& e) {
      }
      return UT_ROBOT_SUCCESS;
    }
    if (status == std::future_status::timeout) {
      return UT_ROBOT_TASK_TIMEOUT;
    }
    return UT_ROBOT_TASK_UNKNOWN_ERROR;
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js);
  }

  std::future<Response> AsyncCall(Request req) {
    using namespace std;
    auto promise_ptr = make_shared<promise<Response>>();
    auto future = promise_ptr->get_future();

    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;

    std::function<void(const std::shared_ptr<const Response>)> callback =
        [promise_ptr, identity_id, this](const std::shared_ptr<const Response> data) mutable {
            if (data->header.identity.id == identity_id) {
                promise_ptr->set_value(*data);
                this->req_suber_.reset();
            }
        };

    req_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1), callback
    );
    req_puber_->publish(req);
    return future;
  }

};
