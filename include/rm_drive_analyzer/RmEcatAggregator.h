#include <bondcpp/bond.h>
#include <diagnostic_msgs/AddDiagnostics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "XmlRpcValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/analyzer_group.h"
#include "diagnostic_aggregator/other_analyzer.h"
#include "diagnostic_aggregator/status_item.h"

namespace rm_device_analyzer {
class RmEcatAggregator {
 public:
  /*!
   *\brief Constructor initializes with nh
   */
  explicit RmEcatAggregator(ros::NodeHandle& nh);
  ~RmEcatAggregator();

  /*!
   *\brief Processes, publishes data. Should be called at pub_rate.
   */
  void publishData();

 private:
  ros::Subscriber diag_sub_;          /**< DiagnosticArray, /diagnostics */
  ros::Publisher agg_pub_;            /**< DiagnosticArray, /diagnostics_agg */
  ros::Publisher toplevel_state_pub_; /**< DiagnosticStatus, /diagnostics_toplevel_state */
  boost::mutex mutex_;

  /*!
   *\brief Callback for incoming "/rm_drive_diagnostics"
   */
  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);

  /*!
   *\brief Service request callback for addition of diagnostics.
   * Creates a bond between the calling node and the aggregator, and loads
   * information about new diagnostics into added_analyzers_, keeping track of
   * the formed bond in bonds_
   */
  bool addDiagnostics(diagnostic_msgs::AddDiagnostics::Request& req, diagnostic_msgs::AddDiagnostics::Response& res);

  diagnostic_aggregator::AnalyzerGroup* analyzer_group_;

  std::string prefix_; /**< \brief Prepended to all status names of aggregator. */

  std::set<std::string> ros_warnings_; /**< \brief Records all ROS warnings. No warnings are repeated. */

  /*!
   *!\brief Checks timestamp of message, and warns if timestamp is 0 (not set)
   */
  void checkTimestamp(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);
};

}  // namespace rm_device_analyzer