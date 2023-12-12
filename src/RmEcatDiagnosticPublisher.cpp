//
// Created by kook on 12/2/23.
//

#include "rm_drive_analyzer/RmEcatDiagnosticPublisher.h"
#include <regex>
#include "diagnostic_msgs/DiagnosticArray.h"

namespace rm_device_analyzer {

RmEcatDiagnosticPublisher::RmEcatDiagnosticPublisher(ros::NodeHandle& nh) {
  diag_sub_ = nh.subscribe("/rosout", 10, &RmEcatDiagnosticPublisher::logCallBack, this);
  diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/rm_drive_diagnostics", 10);
}

RmEcatDiagnosticPublisher::~RmEcatDiagnosticPublisher() {
  diag_sub_.shutdown();
  diag_pub_.shutdown();
}

void RmEcatDiagnosticPublisher::publishDiagnosticDatas() {
  diagnostic_msgs::DiagnosticArray m;
  m.status.clear();

  status_.clearSummary();
  status_.clear();
  status_.name = "rm_ecat";
  // 一直往这个status里面add,放在status下
  status_.add("warnings", warning_number_);
  status_.add("rm_hw", warning_number_);
  status_.add("rm_ok", warning_number_);
  status_.summary(diagnostic_msgs::DiagnosticStatus::WARN, "realtime warnings");
  m.status.push_back(status_);

  status_.clearSummary();
  status_.clear();
  status_.name = "rm_hw";
  // 一直往这个status里面add,放在status下
  status_.add("rm_write", warning_number_);
  status_.add("rm_hw", warning_number_);
  status_.add("warnings", warning_number_);
  status_.summary(diagnostic_msgs::DiagnosticStatus::WARN, "warnings");
  m.status.push_back(status_);

  m.header.stamp = ros::Time::now();
  diag_pub_.publish(m);
}

void RmEcatDiagnosticPublisher::logCallBack(const rosgraph_msgs::Log::ConstPtr& log) {
  std::string except_messages_ = log->msg;
  if (log->level == rosgraph_msgs::Log::WARN) {
    ROS_INFO("Received a warning message: [%s]", log->msg.c_str());
    // 使用正则表达式来匹配包含特定数字的行
    std::string s = ".*cycle time: (\\d+).*";
    std::regex number_regex(s);
    std::smatch match;
    if (std::regex_search(except_messages_, match, number_regex)) {
      // 找到包含特定数字的行
      std::string number_str = match.str(1);  // 第一个捕获组中的内容就是我们想要的数字
      warning_number_++;                      // 将字符串转换为整数
    }
    // 在这里执行你的警告消息处理逻辑
  }
}
}  // namespace rm_device_analyzer