#ifndef CONFIGSERVER_PARAMETER_H
#define CONFIGSERVER_PARAMETER_H

#include <config_server/Subscribe.h>
#include <config_server/SetParameter.h>
#include <config_server/ParameterDescription.h>

#include <ros/service_server.h>
#include <ros/callback_queue.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <boost/property_tree/ptree.hpp>

#ifndef USE_ROS
#define USE_ROS true
#endif

namespace pt = boost::property_tree;

namespace config_server
{

template <bool use_ros=USE_ROS>
class _ConfigServer
{
  _ConfigServer()
  {
    static_assert(use_ros, "Not implemented yet.");
  }
};

template <>
class _ConfigServer<false>
{
private:
  pt::ptree current_values;
  pt::ptree default_values;
  _ConfigServer() = default;
  static _ConfigServer<false> *instance;

public:
  static _ConfigServer<false> *get_instance();

  void load(const std::string& filename);
  void clear();

  template <typename T>
  void register_parameter(const std::string &path, const T &default_value)
  {
    ROS_DEBUG_STREAM("Register parameter " << path << " with default value " << default_value);
    this->default_values.put(path, default_value);
  }

  template <typename T>
  T get(const std::string &path)
  {
    auto v = this->current_values.get_optional<T>(path);

    if (v)
    {
      ROS_DEBUG_STREAM("Get parameter " << path << " from loaded values " << v.get());
      return v.get();
    }

    v = this->default_values.get_optional<T>(path);

    if (!v)
    {
      ROS_ERROR("Not registered or load or set path %s", path.data());
    }

    ROS_DEBUG_STREAM("Get parameter " << path << " from default values " << v.get());

    return v.get();
  }

  template <typename T>
  void set(const std::string &name, const T value)
  {
    current_values.put(name, value);
  }
};

void load(const std::string &filename);
void clear();

/**
 * Base class for all parameters
 **/
class ParameterBase
{
public:
	virtual ~ParameterBase();

protected:
	void notifyServer();
	virtual void notifyClient();

	virtual std::string serialize() const = 0;
	virtual bool deserialize(const std::string& value) = 0;

	void init(const ParameterDescription& desc,
		ros::NodeHandle* nh = 0,
		bool create = true
	);

	inline std::string name() const
	{ return m_name; }

	bool handleSet(const std::string& value);
private:
	friend class ParameterClient;

	std::string m_name;
};

template <class T, bool use_ros=USE_ROS>
class TypedParameter : public ParameterBase
{
public:
	T get() const
	{
		return m_value;
	}

	void set(const T& value)
	{
		m_value = value;
		notifyServer();
		notifyClient();
	}

	inline T operator()() const
	{ return get(); }

	void setCallback(const boost::function<void (const T&)>& callback)
	{
		m_callback = callback;
	}
protected:
	T m_value;
	boost::function<void (const T&)> m_callback;

	virtual void notifyClient()
	{
		if(m_callback)
		{
			m_callback(m_value);
		}
	}
};

/**
 * User API for parameters on the config_server.
 *
 * Example usage:
 * @code
 *  // Create a parameter with range [0.0, 1.0], step size 0.01, default value 0.5
 *  Parameter<float> myParam("myParam", 0.0, 0.01, 1.0, 0.5);
 *
 *  // fetch its value
 *  float value = myParam();
 *
 *  // and set it
 *  myParam.set(0.8);
 *
 *  // set a callback (use boost::bind for more complex setups)
 *  void callback(float value) {}
 *  myParam.setCallback(&callback);
 * @endcode
 **/

template <typename T, bool use_ros = USE_ROS>
class Parameter;


// Following are specializations for different value types.

template<>
class Parameter<std::string, true> : public TypedParameter<std::string, true>
{
public:
	Parameter(const std::string& name, const std::string& value = "")
	{
		config_server::ParameterDescription desc;
		desc.name = name;
		desc.type = "string";
		desc.default_value = value;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		m_value = value;
		return true;
	}

	virtual std::string serialize() const
	{
		return m_value;
	}
};

template<class T>
inline std::string toString(const T& value)
{
	std::stringstream ss;
	ss << value;
	return ss.str();
}

template<>
class Parameter<int, true> : public TypedParameter<int, true>
{
public:
	Parameter(const std::string& name, int min, int step, int max, int value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "int";
		desc.default_value = toString(value);
		desc.step = step;
		desc.min = min;
		desc.max = max;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		std::stringstream ss(value);

		int v;
		ss >> v;

		if(ss.fail() || ss.bad() || !ss.eof())
			return false;

		m_value = v;
		return true;
	}
	virtual std::string serialize() const
	{
		std::stringstream ss;
		ss << m_value;

		return ss.str();
	}
};

template<>
class Parameter<float, true> : public TypedParameter<float, true>
{
public:
	Parameter(const std::string& name, float min, float step, float max, float value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "float";
		desc.default_value = toString(value);
		desc.step = step;
		desc.min = min;
		desc.max = max;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		std::stringstream ss(value);

		float v;
		ss >> v;

		if(ss.fail() || ss.bad() || !ss.eof())
			return false;

		m_value = v;
		return true;
	}
	virtual std::string serialize() const
	{
		std::stringstream ss;
		ss << m_value;

		return ss.str();
	}
};

template<>
class Parameter<bool, true> : public TypedParameter<bool, true>
{
public:
	Parameter(const std::string& name, bool default_value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "bool";
		desc.default_value = default_value ? "1" : "0";
		desc.step = 1;
		desc.min = 0;
		desc.max = 1;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		if(value.size() == 0 || value == "0")
			m_value = false;
		else
			m_value = true;

		return true;
	}

	virtual std::string serialize() const
	{
		return m_value ? "1" : "0";
	}
};

template <typename T>
class TypedParameter<T, false>
{
private:
  const std::string path;

public:
  TypedParameter(const std::string &path, const T &value):
    path(convertFromLegacy(path))
  {
    config_server::_ConfigServer<false>::get_instance()->register_parameter(this->path, value);
  }

  T operator()() const
  {
    return config_server::_ConfigServer<false>::get_instance()->get<T>(this->path);
  }

  static std::string convertFromLegacy(const std::string &str)
  {
    std::string ret = str;
    ret.replace(0, 1, "roboland.", 0, std::string::npos); // drop the first slash and replace it with roboland.
    std::replace(ret.begin(), ret.end(), '/', '.');

    return ret;
  }
};

template <>
class Parameter<std::string, false> : public TypedParameter<std::string, false>
{
public:
  Parameter(const std::string &name, const std::string value = ""):
    TypedParameter<std::string, false>(name, value)
  {
  }
};

template <>
class Parameter<float, false> : public TypedParameter<float, false>
{
public:
  Parameter(const std::string &name, const float min, const float step, const float max, const float default_value):
    TypedParameter<float, false>(name, default_value)
  {
  }
};

template <>
class Parameter<int, false> : public TypedParameter<int, false>
{
public:
  Parameter(const std::string &name, const int min, const int step, const int max, const int default_value):
    TypedParameter<int, false>(name, default_value)
  {
  }
};

template <>
class Parameter<bool, false> : public TypedParameter<bool, false>
{
public:
  Parameter(const std::string &name, const bool default_value):
    TypedParameter<bool, false>(name, default_value)
  {
  }
};

} // namespace config_server

#endif
