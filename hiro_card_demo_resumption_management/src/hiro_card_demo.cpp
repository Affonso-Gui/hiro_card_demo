#include <ros/ros.h>
#include <ros/package.h>

#define DEBUG  // rosbridgecpp logging
#include <roseus_bt/eus_nodes.h>
#include <roseus_bt/command_line_argument_mapping.h>
#include <roseus_bt/rosparam_argument_mapping.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <hiro_card_demo_resumption_management/DETECTAction.h>
#include <hiro_card_demo_resumption_management/PICK_AND_PLACEAction.h>
#include <hiro_card_demo_resumption_management/RETRIEVEAction.h>
#include <hiro_card_demo_resumption_management/HasClearRequest.h>
#include <hiro_card_demo_resumption_management/HasGoalRequest.h>

using namespace BT;


class DETECT: public EusActionNode<hiro_card_demo_resumption_management::DETECTAction>
{

public:
  DETECT(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<hiro_card_demo_resumption_management::DETECTAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "/hiro_card_demo/detect", "name of the Action Server"),
      InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class PICK_AND_PLACE: public EusActionNode<hiro_card_demo_resumption_management::PICK_AND_PLACEAction>
{

public:
  PICK_AND_PLACE(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<hiro_card_demo_resumption_management::PICK_AND_PLACEAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "/hiro_card_demo/pick_and_place", "name of the Action Server"),
      InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class RETRIEVE: public EusActionNode<hiro_card_demo_resumption_management::RETRIEVEAction>
{

public:
  RETRIEVE(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<hiro_card_demo_resumption_management::RETRIEVEAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "/hiro_card_demo/retrieve", "name of the Action Server"),
      InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class HasClearRequest: public EusConditionNode<hiro_card_demo_resumption_management::HasClearRequest>
{

public:
  HasClearRequest(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
  EusConditionNode<hiro_card_demo_resumption_management::HasClearRequest>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("service_name", "/hiro_card_demo/wait_clear", "name of the ROS service")
    };
  }

  void sendRequest(RequestType& request) override
  {

  }

  NodeStatus onResponse(const ResponseType& res) override
  {
    if (res.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class HasGoalRequest: public EusConditionNode<hiro_card_demo_resumption_management::HasGoalRequest>
{

public:
  HasGoalRequest(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
  EusConditionNode<hiro_card_demo_resumption_management::HasGoalRequest>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("service_name", "/hiro_card_demo/wait_goal", "name of the ROS service")
    };
  }

  void sendRequest(RequestType& request) override
  {

  }

  NodeStatus onResponse(const ResponseType& res) override
  {
    if (res.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hiro_card_demo_engine");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::map<std::string, std::string> init_variables;
  // please comment in if you want to use command line argument
  // if (!roseus_bt::parse_command_line(argc, argv, "Run the hiro_card_demo task.", init_variables)) return 1;
  if (!roseus_bt::parse_rosparam(pnh, init_variables)) return 1;

  BehaviorTreeFactory factory;

  RegisterRosAction<DETECT>(factory, "DETECT", nh);
  RegisterRosAction<PICK_AND_PLACE>(factory, "PICK_AND_PLACE", nh);
  RegisterRosAction<RETRIEVE>(factory, "RETRIEVE", nh);
  RegisterRosService<HasClearRequest>(factory, "HasClearRequest", nh);
  RegisterRosService<HasGoalRequest>(factory, "HasGoalRequest", nh);

  auto tree = factory.maybeCreateLayeredTreeFromFile(fmt::format("{0}/models/hiro_card_demo.xml", ros::package::getPath("hiro_card_demo_resumption_management")));
  roseus_bt::register_blackboard_variables(tree.get(), init_variables);

  std::string timestamp = std::to_string(ros::Time::now().toNSec());
  std::string log_filename(fmt::format("/home/affonso/.ros/hiro_card_demo_engine_{0}.fbl", timestamp));

  StdCoutLogger logger_cout(*tree);
  FileLogger logger_file(*tree, log_filename.c_str());
  PublisherZMQ publisher_zmq(*tree);

  NodeStatus status = NodeStatus::IDLE;

  std::cout << "Writing log to file: " << log_filename << std::endl;

  try {
    while( ros::ok() )
      {
        ros::spinOnce();
        status = tree->tickRoot();
        ros::Duration sleep_time(0.01);
        sleep_time.sleep();
      }
  }
  catch(BT::RuntimeError& err) {
    std::cerr << "Behavior Tree execution terminated after throwing an instance of 'BT::RuntimeError'" << "\n  what():  " << err.what() << std::endl;
  }

  std::cout << "Writed log to file: " << log_filename << std::endl;
  std::cout << "Behavior Tree execution finished with " << toStr(status, true).c_str() << std::endl;
  return (status != NodeStatus::SUCCESS);
}
