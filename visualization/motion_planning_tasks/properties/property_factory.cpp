/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#include "property_factory.h"
#include <boost/functional/factory.hpp>
#include <moveit/task_constructor/properties.h>

#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

/// TODO: We also need to provide methods to sync both properties in both directions.
/// In our Property we could store a callback function to update the rviz::Property.
/// The rviz::Property can simply use a lambda-function slot to update our Property.
/// However, we need to avoid infinite loops in doing so. Our Property cannot compare values...
template <typename T, typename RVIZProp>
RVIZProp* helper(const QString& name, Property* prop) {
	T value = prop->defined() ? boost::any_cast<T>(prop->value()) : T();
	return new RVIZProp(name, value, QString::fromStdString(prop->description()));
}

PropertyFactory::PropertyFactory()
{
	// registe some standard types
	registerType<double>(&helper<double, rviz::FloatProperty>);
	registerType<QString>(&helper<QString, rviz::StringProperty>);
}

PropertyFactory& PropertyFactory::instance()
{
	static PropertyFactory instance_;
	return instance_;
}

void PropertyFactory::registerType(const std::string &type_name, const FactoryFunction &f)
{
	registry_.insert(std::make_pair(type_name, f));
}

rviz::Property* PropertyFactory::create(const std::string& prop_name, Property* prop) const
{
	auto it = registry_.find(prop->typeName());
	if (it == registry_.end()) return nullptr;
	return it->second(QString::fromStdString(prop_name), prop);
}

rviz::Property* PropertyFactory::create(const moveit_task_constructor_msgs::Property& p, rviz::Property* old) const
{
	if (old) {  // reuse existing Property?
		old->setDescription(QString::fromStdString(p.description));
		old->setValue(QString::fromStdString(p.value));
		return old;
	} else {  // create new Property?
		rviz::Property *result = new rviz::StringProperty(QString::fromStdString(p.name),
		                                                  QString::fromStdString(p.value),
		                                                  QString::fromStdString(p.description));
		result->setReadOnly(true);
		return result;
	}
}

rviz::PropertyTreeModel* createPropertyTreeModel(PropertyMap& properties, QObject* parent) {
	PropertyFactory& factory = PropertyFactory::instance();

	rviz::Property* root = new rviz::Property();
	rviz::PropertyTreeModel *model = new rviz::PropertyTreeModel(root, parent);
	for (auto& prop : properties) {
		rviz::Property* rviz_prop = factory.create(prop.first, &prop.second);
		if (!rviz_prop) rviz_prop = new rviz::Property(QString::fromStdString(prop.first));
		rviz_prop->setParent(root);
	}
	// just to see something, when no properties are defined
	if (model->rowCount() == 0)
		new rviz::Property("no properties", QVariant(), QString(), root);
	return model;
}

}
