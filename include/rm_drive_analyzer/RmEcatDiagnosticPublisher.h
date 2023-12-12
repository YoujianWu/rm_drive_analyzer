//
// Created by kook on 12/2/23.
//

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <rm_ecat_slave/RmEcatSlave.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include "rm_drive_analyzer/RmEcatAnalyzer.h"

namespace rm_device_analyzer {
class RmEcatDiagnosticPublisher {
 public:
  explicit RmEcatDiagnosticPublisher(ros::NodeHandle& nh);
  ~RmEcatDiagnosticPublisher();

  void publishDiagnosticDatas();

 private:
  diagnostic_msgs::DiagnosticArray data_;
  ros::Subscriber diag_sub_;
  ros::Publisher diag_pub_;
  int warning_number_{0};
  diagnostic_updater::DiagnosticStatusWrapper status_;

  std::vector<diagnostic_msgs::DiagnosticArray> messages_;
  std::vector<std::shared_ptr<rm_ecat::RmEcatSlave>> slaves_;

  void logCallBack(const rosgraph_msgs::Log::ConstPtr& log);
};
}  // namespace rm_device_analyzer