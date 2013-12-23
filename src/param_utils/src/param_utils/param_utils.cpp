
#include <param_utils/param_utils.h>

namespace param_utils
{

ParamHelper::ParamHelper(ros::NodeHandle & node_handle, const std::string & name) : node_handle_(node_handle), name_(name)
{

}

ParamHelper::~ParamHelper()
{

}

bool ParamHelper::getParamWithInfo(const std::string & key, XmlRpc::XmlRpcValue & v, const XmlRpc::XmlRpcValue & default_v, bool required) const
{
  return getParamWithInfo<XmlRpc::XmlRpcValue>(key, v, default_v, required);
}

bool ParamHelper::getParamWithInfo(const std::string & key, bool & b, bool default_b, bool required) const
{
  return getParamWithInfo<bool>(key, b, default_b, required);
}

bool ParamHelper::getParamWithInfo(const std::string & key, int & i, int default_i, bool required) const
{
  return getParamWithInfo<int>(key, i, default_i, required);
}

bool ParamHelper::getParamWithInfo(const std::string & key, double & d, double default_d, bool required) const
{
  return getParamWithInfo<double>(key, d, default_d, required);
}

bool ParamHelper::getParamWithInfo(const std::string & key, std::string & s, const std::string & default_s, bool required) const
{
  return getParamWithInfo<std::string>(key, s, default_s, required);
}

template<typename T>
bool ParamHelper::getParamWithInfo(const std::string & key, T & param_val, const T & default_val, bool required) const
{

  std::stringstream ss_param_val;

  if (node_handle_.getParam(key, param_val))
  {
    ss_param_val << param_val;
    ROS_INFO("[%s]: <%s> set to %s", name_.c_str(), key.c_str(), ss_param_val.str().c_str());
  }
  else if(!required)
  {
    ss_param_val << param_val;
    ROS_WARN("[%s]: <%s> not specified, defaulting to %s", name_.c_str(), key.c_str(), ss_param_val.str().c_str());
    param_val = default_val;
  }
  else
  {
    ROS_ERROR("[%s]: <%s> not specified, but is required", name_.c_str(), key.c_str());
  }

}

} // namespace param_utils

