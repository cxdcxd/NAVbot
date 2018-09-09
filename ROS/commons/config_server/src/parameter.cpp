// Client object
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/parameter.h>
#include <config_server/parameterclient.h>
#include <config_server/Subscribe.h>
#include <ros/service.h>
#include <ros/this_node.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>

#include <functional>

namespace config_server
{

ParameterBase::~ParameterBase()
{
	if(!m_name.empty())
		ParameterClient::instance()->unregisterParameter(this);
}

void ParameterBase::init(const ParameterDescription& desc, ros::NodeHandle* nh, bool create)
{
	m_name = desc.name;
	if(m_name[0] != '/')
		m_name = ros::this_node::getName() + "/" + desc.name;

	ParameterDescription createDesc;
	if(create)
		createDesc = desc;

	createDesc.name = m_name;

	ParameterClient::instance()->registerParameter(this, createDesc);
}

bool ParameterBase::handleSet(const std::string& value)
{
	if(!deserialize(value))
		return false;
	notifyClient();
	return true;
}

void ParameterBase::notifyServer()
{
	ParameterClient::instance()->notify(this, serialize());
}

void ParameterBase::notifyClient()
{
}

_ConfigServer<false> *_ConfigServer<false>::instance = nullptr;

_ConfigServer<false> *_ConfigServer<false>::get_instance()
{
  if (!instance)
    instance = new _ConfigServer<false>();

  return instance;
}

void _ConfigServer<false>::load(const std::string &filename)
{
  YAML::Node n = YAML::LoadFile(filename.c_str());
  std::function<void(const YAML::Node &n, const std::string &path, pt::ptree &ptree)> insert_in_ptree;

  insert_in_ptree = [&insert_in_ptree](const YAML::Node &n, const std::string &path, pt::ptree &ptree) {
    if (n.Type() == YAML::NodeType::Map)
    {
      pt::ptree c;
      for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
      {
        insert_in_ptree(it->second, it->first.as<std::string>(), c);
      }
      ptree.put_child(path, c);
    }
    else if (n.Type() == YAML::NodeType::Scalar)
    {
      ptree.put(path, n.as<std::string>());
    }
  };

  insert_in_ptree(n, "roboland", this->current_values);
}

void _ConfigServer<false>::clear()
{
  this->current_values.clear();
  this->default_values.clear();
}

void load(const std::string &filename)
{
  _ConfigServer<false>::get_instance()->load(filename);
}

void clear()
{
  _ConfigServer<false>::get_instance()->clear();
}

}
