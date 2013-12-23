
#ifndef PARAM_UTILS_H_
#define PARAM_UTILS_H_

#include <ros/ros.h>

namespace param_utils
{

/** \brief Implements helper functions to allow for easier and more informative parameter parsing.
  *
  * Authors: Sam Marden
  */
class ParamHelper
{

public:

  /** \brief Constructor with reference to node handle to be used for getting params and the name of the node requesting the params.
    *
    * \param node_handle Node handle used to retrieve parameters from server.
    * \param name Name of the node retrieving the parameters.
    */
  ParamHelper(ros::NodeHandle & node_handle, const std::string & name);

  /** \brief Destructor. */
  ~ParamHelper();

  /** \brief Function to get an XML/RPC value with information about the parameter
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param v Storage for the retrieved value.
    * \param default_v Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */
  bool getParamWithInfo(const std::string & key, XmlRpc::XmlRpcValue & v, const XmlRpc::XmlRpcValue & default_v, bool required = false) const;

  /** \brief Function to get a boolean value with information about the parameter.
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param b Storage for the retrieved value.
    * \param default_b Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */
  bool getParamWithInfo(const std::string & key, bool & b, bool default_b, bool required = false) const;

  /** \brief Function to get a integer value with information about the parameter.
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param i Storage for the retrieved value.
    * \param default_i Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */ 
  bool getParamWithInfo(const std::string & key, int & i, int default_i, bool required = false) const;

  /** \brief Function to get a double value with information about the parameter.
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param d Storage for the retrieved value.
    * \param default_d Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */
  bool getParamWithInfo(const std::string & key, double & d, double default_d, bool required  = false) const;

  /** \brief Function to get a string value with information about the parameter.
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param s Storage for the retrieved value.
    * \param default_s Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */
  bool getParamWithInfo(const std::string & key, std::string & s, const std::string & default_s, bool required = false) const;

private:

  /** \brief Function to get a templated value with information about the parameter.
    *
    * \param key The key to be used in the parameter server's dictionary.
    * \param param_val Storage for the retrieved value.
    * \param default_val Default for the retrieved value.
    * \param required Whether or not the parameter is required or optional.
    */
  template<typename T>
  bool getParamWithInfo(const std::string & key, T & param_val, const T & default_val, bool required) const;

  /** \brief Node handle used to retrieve parameters from server. */
  ros::NodeHandle node_handle_;

  /** \brief Name of the node retrieving the parameters. */
  std::string name_;

};

} // namespace param_utils

#endif // #ifndef PARAM_UTILS_H_

