#ifndef ROS_UTIL_H
#define ROS_UTIL_H


#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace ros_util {

struct EnumClassHash
{
    template <typename T>
    std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

template<class T>
T getParamOrThrow(ros::NodeHandle& nh, const std::string& param_name) {
  T param;
  bool gotParam = nh.getParam(param_name, param);
  ROS_INFO_STREAM("Got param [" << param_name << "] = " << param);
  if (!gotParam) {
    ROS_FATAL_STREAM("Unable to get param: [" << param_name << "]. Throwing...");
    throw std::runtime_error("Parameter initialization failed.");
  }
  return param;
}

template<class T>
class TopicHandler {
public:
  TopicHandler(ros::NodeHandle& t_nh, const std::string& t_topicName) :
    m_topicName(t_topicName) {
    m_subT = t_nh.subscribe(m_topicName, 1, &TopicHandler::callback, this);
  }

  const T& getData() {
    return m_data;
  }

private:
  void callback(const T& msg) {
    m_data = std::move(msg);
  }

  ros::Subscriber m_subT;
  const std::string m_topicName;
  T m_data;
};

template<class T>
class ParamHandler {
public:
  ParamHandler(T t_defaultConfig, const std::string& configName) :
    m_configServer(m_configMutex, ros::NodeHandle(configName)) {
    m_configServer.updateConfig(t_defaultConfig);
    auto paramCallback = boost::bind(&ParamHandler::paramCallback, this, _1, _2);
    m_configServer.setCallback(paramCallback);
  }

  const T& getData() {
    return m_currentConfig;
  }

private:
  void paramCallback(const T& cfgParams, uint32_t /* unused */) {
    m_currentConfig = std::move(cfgParams);
  }

  boost::recursive_mutex m_configMutex;
  dynamic_reconfigure::Server<T> m_configServer;
  T m_currentConfig;
};
}

#endif /* ROS_UTIL_H */