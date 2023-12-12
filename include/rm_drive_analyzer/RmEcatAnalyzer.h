//
// Created by kook on 11/30/23.
//

#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>

namespace rm_device_analyzer {

class RmEcatAnalyzer : public diagnostic_aggregator::Analyzer {
 public:
  RmEcatAnalyzer();

  ~RmEcatAnalyzer() override = default;

  bool init(std::string prefix, const ros::NodeHandle& nh) override;

  bool match(std::string name) override;

  bool analyze(boost::shared_ptr<diagnostic_aggregator::StatusItem> item) override;

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> report() override;

  std::string getPath() const override { return path_; }

  std::string getName() const override { return match_name_; }

 private:
  // Store status item from /rm_drive_diagnostics
  boost::shared_ptr<diagnostic_aggregator::StatusItem> item_;

  std::string path_, match_name_;

  int warnings_;
  bool has_initialized_ = false, has_eth_data_ = false;
};

}  // namespace rm_device_analyzer