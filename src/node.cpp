//
// Created by kook on 12/2/23.
//

#include <ros/ros.h>
#include <iostream>
#include <regex>
#include "rm_drive_analyzer/RmEcatAggregator.h"
#include "rm_drive_analyzer/RmEcatDiagnosticPublisher.h"

// void logCallback(const rosgraph_msgs::Log::ConstPtr& log) {
//   if (log->level == rosgraph_msgs::Log::WARN) {
//     std::string warning_message = log->msg;
//     ROS_INFO("Received a warning message: [%s]", log->msg.c_str());
//     // 使用正则表达式来匹配包含特定数字的行
//     std::regex number_regex(".*cycle time: (\\d+).*");
//     std::smatch match;
//     if (std::regex_search(warning_message, match, number_regex)) {
//       // 找到包含特定数字的行
//       std::string number_str = match.str(1);  // 第一个捕获组中的内容就是我们想要的数字
//       int number = std::stoi(number_str);     // 将字符串转换为整数
//       std::cout << "Found number in warning message: " << number << std::endl;
//     }
//     // 在这里执行你的警告消息处理逻辑
//   }
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_device_analyzer");
  ros::NodeHandle nhp("~");
  ros::NodeHandle nh_pub("rm_drive_diagnostic_publisher");

  auto* d = new rm_device_analyzer::RmEcatDiagnosticPublisher(nh_pub);
  rm_device_analyzer::RmEcatAggregator agg(nhp);
  ros::Rate r(100);
  while (ros::ok()) {
    d->publishDiagnosticDatas();
    agg.publishData();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}