//
// Created by kook on 12/2/23.
//

#include "rm_drive_analyzer/RmEcatDiagnosticPublisher.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <regex>

namespace rm_device_analyzer {

RmEcatDiagnosticPublisher::RmEcatDiagnosticPublisher(ros::NodeHandle& nh) {
  diag_sub_ = nh.subscribe("/rosout", 10, &RmEcatDiagnosticPublisher::logCallBack, this);
  diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/rm_drive_diagnostics", 10);

  ros::NodeHandle check_group_nh(nh, "check_group");
  XmlRpc::XmlRpcValue rpc_value;

  ROS_INFO_STREAM(nh.getNamespace());
  ROS_INFO_STREAM(nh.getParam("check_index", rpc_value));  // todo: add null protections
  for (int i = 0; i < rpc_value.size(); i++) {
    check_index_.push_back(rpc_value[i]);
  }
  for (const auto& index : check_index_) {
    check_group_nh.getParam(index, rpc_value);
    for (int i = 0; i < rpc_value.size(); i++) {
      ROS_INFO_STREAM(rpc_value[i]["check_item"]);
      check_group_.emplace(index, std::make_pair(rpc_value[i]["check_item"], 0.));
      std::regex re((std::string)rpc_value[i]["regular_expression"]);
      regulations_.emplace(rpc_value[i]["check_item"], std::make_pair(rpc_value[i]["level"], re));
    }
  }
}

RmEcatDiagnosticPublisher::~RmEcatDiagnosticPublisher() {
  diag_sub_.shutdown();
  diag_pub_.shutdown();
}

void RmEcatDiagnosticPublisher::publishDiagnosticDatas() {
  diagnostic_msgs::DiagnosticArray m;
  diagnostic_updater::DiagnosticStatusWrapper status_;
  m.status.clear();

  for (const auto& index : check_index_) {
    status_.clearSummary();
    status_.clear();
    status_.name = index;
    for (const auto& check : check_group_) {
      if (check.first == index) {
        for (size_t i = 0; i < check.second.first.size(); i++) {
          status_.add(check.second.first, check.second.second);
        }  // todo:size_t 与 int 的区别
      }
    }
    status_.summary(diagnostic_msgs::DiagnosticStatus::WARN, index);
    m.status.push_back(status_);
  }
  m.header.stamp = ros::Time::now();
  diag_pub_.publish(m);
}

void RmEcatDiagnosticPublisher::logCallBack(const rosgraph_msgs::Log::ConstPtr& log) {
  std::string except_messages_ = log->msg;
  if (log->level == rosgraph_msgs::Log::WARN) {
    ROS_INFO("Received a warning message: [%s]", log->msg.c_str());
    // 使用正则表达式来匹配包含特定数字的行
    for (const auto& re : regulations_) {
      std::smatch match;
      if (std::regex_search(except_messages_, match, re.second.second)) {
        // 找到包含特定数字的行
        std::string number_str = match.str(1);  // 第一个捕获组中的内容就是我们想要的数字
        warning_number_++;                      // 将字符串转换为整数
      }
      // 在这里执行你的警告消息处理逻辑
    }
  }
}
}  // namespace rm_device_analyzer