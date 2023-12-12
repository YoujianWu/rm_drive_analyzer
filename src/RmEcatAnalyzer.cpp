//
// Created by kook on 11/30/23.
//

#include "rm_drive_analyzer/RmEcatAnalyzer.h"

namespace rm_device_analyzer {

RmEcatAnalyzer::RmEcatAnalyzer() : warnings_(0), has_initialized_(false), has_eth_data_(false) {}

bool RmEcatAnalyzer::init(const std::string prefix, const ros::NodeHandle& nh) {
  if (!nh.getParam("match_name", match_name_)) {
    ROS_ERROR("no match_name defined in namespace : [ %s ]", nh.getNamespace().c_str());
    return false;
  }
  path_ = prefix + nh.getNamespace();  // 注意 getParam 和param的区别
  //  if (!nh.getParam("power_board_name", power_board_name_)) {
  //    ROS_ERROR("No power board name was specified in PR2MotorsAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s",
  //              nh.getNamespace().c_str());
  //    return false;
  //  }

  // Make a "missing" item for the EtherCAT Master
  boost::shared_ptr<diagnostic_aggregator::StatusItem> item(new diagnostic_aggregator::StatusItem("rm_ecat messages"));
  item_ = item;

  has_initialized_ = true;
  ROS_INFO("Init successfully!");

  return true;
}

// 通过rm_ecat来寻找匹配这个分析器
bool RmEcatAnalyzer::match(const std::string name) {
  if (name == match_name_) {
    return true;
  }
  return false;
}

bool RmEcatAnalyzer::analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item) {
  if (item->getName() == match_name_) {
    // 通过索引来查找
    // key value就是哈希表的那种类型
    warnings_ = std::stoi(item->getValue("warnings"));  // todo: add new regulations
    // ROS_INFO_STREAM(item->getValue("warnings"));
    //  Won't report this item
  }

  // We know our item is "EtherCAT Master"
  item_ = item;
  has_eth_data_ = true;

  return true;
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> RmEcatAnalyzer::report() {
  // add a path prefix to item_ analyzed
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ecat_messages = item_->toStatusMsg(path_);

  // Check all the data, if one has a problem, then report an error or a warning
  if (has_eth_data_ && (warnings_ != 0)) {
    ecat_messages->level = diagnostic_msgs::DiagnosticStatus::WARN;
  } else {
    ecat_messages->level = diagnostic_msgs::DiagnosticStatus::OK;
  }

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> output;
  output.push_back(ecat_messages);

  return output;
}

}  // namespace rm_device_analyzer
PLUGINLIB_EXPORT_CLASS(rm_device_analyzer::RmEcatAnalyzer, diagnostic_aggregator::Analyzer)