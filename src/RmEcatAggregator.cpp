//
// Created by kook on 12/1/23.
//
#include "rm_drive_analyzer/RmEcatAggregator.h"
#include <diagnostic_aggregator/status_item.h>

namespace rm_device_analyzer {

RmEcatAggregator::RmEcatAggregator(ros::NodeHandle& nh) : analyzer_group_(nullptr) {
  nh.param("prefix", prefix_, (std::string) "");
  //  if (!prefix_.empty() && prefix_.find('/') != 0) {
  //    prefix_ = "/" + prefix_;
  //  }

  analyzer_group_ = new diagnostic_aggregator::AnalyzerGroup();

  ROS_INFO_STREAM(nh.getNamespace());
  // init Analyzer group
  if (!analyzer_group_->init(prefix_, nh)) {
    ROS_ERROR("Analyzer group for diagnostic aggregator failed to initialize!");
    return;
  }

  diag_sub_ = nh.subscribe("/rm_drive_diagnostics", 10, &RmEcatAggregator::diagCallback, this);
  agg_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/rm_drive_diagnostics_agg", 10);
  toplevel_state_pub_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("/rm_drive_diagnostics_toplevel_state", 10);
}

// 检查DiagnosticArray->header.stamp.toSec()是否为0
void RmEcatAggregator::checkTimestamp(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg) {
  if (diag_msg->header.stamp.toSec() != 0) {
    return;
  }

  std::string stamp_warn = "No timestamp set for diagnostic message. Message names: ";
  std::vector<diagnostic_msgs::DiagnosticStatus>::const_iterator it;
  for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it) {
    if (it != diag_msg->status.begin()) {
      stamp_warn += ", ";
    }
    stamp_warn += it->name;
  }

  if (!ros_warnings_.count(stamp_warn)) {
    ROS_WARN("%s", stamp_warn.c_str());
    ros_warnings_.insert(stamp_warn);
  }
}

void RmEcatAggregator::diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg) {
  checkTimestamp(diag_msg);

  bool analyzed = false;
  {  // lock the whole loop to ensure nothing in the analyzer group changes during it.
    boost::mutex::scoped_lock lock(mutex_);
    auto statuses = diag_msg->status;
    for (const auto& status : statuses) {
      analyzed = false;
      boost::shared_ptr<diagnostic_aggregator::StatusItem> item(new diagnostic_aggregator::StatusItem(&status));

      if (analyzer_group_->match(item->getName())) {
        analyzed = analyzer_group_->analyze(item);
      }
      if (!analyzed) {
      }  // todo: fill this
    }
  }
}

RmEcatAggregator::~RmEcatAggregator() {
  if (analyzer_group_) delete analyzer_group_;
}

void RmEcatAggregator::publishData() {
  diagnostic_msgs::DiagnosticArray diag_array;

  diagnostic_msgs::DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = -1;
  int min_level = 255;

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> processed;
  {
    boost::mutex::scoped_lock lock(mutex_);
    // 获得处理过的信息
    processed = analyzer_group_->report();
  }
  const auto num_processed = processed.size();
  for (unsigned int i = 0; i < num_processed; ++i) {
    diag_array.status.push_back(*processed[i]);

    if (processed[i]->level > diag_toplevel_state.level) diag_toplevel_state.level = processed[i]->level;
    if (processed[i]->level < min_level) min_level = processed[i]->level;
  }

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> processed_other;
  {
    boost::mutex::scoped_lock lock(mutex_);
    // processed_other = other_analyzer_->report();
  }
  for (unsigned int i = 0; i < processed_other.size(); ++i) {
    diag_array.status.push_back(*processed_other[i]);

    if (processed_other[i]->level > diag_toplevel_state.level) diag_toplevel_state.level = processed_other[i]->level;
    if (processed_other[i]->level < min_level) min_level = processed_other[i]->level;
  }

  diag_array.header.stamp = ros::Time::now();

  agg_pub_.publish(diag_array);

  // Top level is error if we have stale items, unless all stale
  if (diag_toplevel_state.level > int(diagnostic_aggregator::DiagnosticLevel::Level_Error) &&
      min_level <= int(diagnostic_aggregator::DiagnosticLevel::Level_Error))
    diag_toplevel_state.level = diagnostic_aggregator::DiagnosticLevel::Level_Error;

  toplevel_state_pub_.publish(diag_toplevel_state);
}
}  // namespace rm_device_analyzer