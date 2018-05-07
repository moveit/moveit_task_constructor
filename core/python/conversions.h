#pragma once

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>

namespace moveit {
namespace python {

template <typename T>
std::vector<T> fromList(const boost::python::list& values)
{
	boost::python::stl_input_iterator<T> begin(values), end;
	return std::vector<T>(begin, end);
}

template <typename T>
boost::python::list toList(const std::vector<T>& v)
{
	boost::python::list l;
	for (const T& value : v)
		l.append(value);
	return l;
}


template <typename T>
std::map<std::string, T> fromDict(const boost::python::dict& values)
{
	std::map<std::string, T> m;
	for (boost::python::stl_input_iterator<boost::python::tuple> it(values.iteritems()), end; it != end; ++it) {
		const std::string& key = boost::python::extract<std::string>((*it)[0]);
		const T& value = boost::python::extract<T>((*it)[1]);
		m.insert(std::make_pair(key, value));
	}
	return m;
}

template <typename T>
boost::python::dict toDict(const std::map<std::string, T>& v)
{
  boost::python::dict d;
  for (const std::pair<std::string, T>& p : v)
    d[p.first] = p.second;
  return d;
}

} }
