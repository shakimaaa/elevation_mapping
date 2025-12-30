/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const std::shared_ptr<rclcpp::Node>& nodeHandle) : nodeHandle_(nodeHandle) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  nodeHandle_->declare_parameter("inputs", std::vector<std::string>()); // 声明一个参数 inputs 空

  // Configure the visualizations from a configuration stored on the parameter server.
  std::vector<std::string> inputSourcesConfiguration;
  if (!nodeHandle_->get_parameter("inputs", inputSourcesConfiguration)) { // 读取到inputSourcesConfiguration
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Could not load the input sources configuration from parameter\n "
        "%s, are you sure it was pushed to the parameter server? Assuming\n "
        "that you meant to leave it empty. Not subscribing to any inputs!\n",
        inputSourcesNamespace.c_str());
    return false;
  }
  
  return configure(inputSourcesConfiguration, inputSourcesNamespace);  // inputSourcesNamespace = input_sources
}

// 这里只会使用inputs，没有使用input_sources
bool InputSourceManager::configure(const std::vector<std::string>& config, const std::string& sourceConfigurationName) {
  //                                                               inputs   input_sources  在config内这两个被设置的内容为同一个 [ground_truth_cloud]
  if (config.size() == 0) {  // 使用空数组作为特殊情况来显式配置无输入。
    return true;
  }
 
  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics; // std::set 如果有重复的元素，会自动删除
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_->get_parameter("robot_base_frame_id").as_string(),
                                                                      nodeHandle_->get_parameter("map_frame_id").as_string()};
  // Configure all input sources in the list.
  for (auto inputConfig : config) {
    // FIXME: fix namespace and subnode
    // return leading / -> rclcpp::expand_topic_or_service_name(sourceConfigurationName + "/" + inputConfig, nodeHandle_->get_name(), nodeHandle_->get_namespace()
    // auto subnode = nodeHandle_->create_sub_node(sourceConfigurationName + "/" + inputConfig);
    Input source = Input(nodeHandle_);

    bool configured = source.configure(inputConfig, sourceConfigurationName, generalSensorProcessorConfig);
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    std::string subscribedTopic = source.getSubscribedTopic();
    // set::insert() returns a pair<bool, set::iterator>
    // first 是一个迭代器，指向插入的元素，second 是一个布尔值，表明是否插入成功
    bool topicIsUnique = subscribedTopics.insert(subscribedTopic).second;

    if (topicIsUnique) {
      sources_.push_back(std::move(source)); // 将source移动到sources_
    } else {
      RCLCPP_WARN(nodeHandle_->get_logger(),
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping