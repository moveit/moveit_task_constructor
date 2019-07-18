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
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/properties.h>

#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>

namespace mtc = ::moveit::task_constructor;

namespace moveit_rviz_plugin {

static rviz::StringProperty* stringFactory(const QString& name, mtc::Property& mtc_prop,
                                           const planning_scene::PlanningScene*, rviz::DisplayContext*) {
	std::string value;
	if (!mtc_prop.value().empty())
		value = boost::any_cast<std::string>(mtc_prop.value());
	rviz::StringProperty* rviz_prop =
	    new rviz::StringProperty(name, QString::fromStdString(value), QString::fromStdString(mtc_prop.description()));
	QObject::connect(rviz_prop, &rviz::StringProperty::changed,
	                 [rviz_prop, &mtc_prop]() { mtc_prop.setValue(rviz_prop->getStdString()); });
	return rviz_prop;
}
template <typename T>
static rviz::FloatProperty* floatFactory(const QString& name, mtc::Property& mtc_prop,
                                         const planning_scene::PlanningScene*, rviz::DisplayContext*) {
	T value = !mtc_prop.value().empty() ? boost::any_cast<T>(mtc_prop.value()) : T();
	rviz::FloatProperty* rviz_prop =
	    new rviz::FloatProperty(name, value, QString::fromStdString(mtc_prop.description()));
	QObject::connect(rviz_prop, &rviz::FloatProperty::changed,
	                 [rviz_prop, &mtc_prop]() { mtc_prop.setValue(T(rviz_prop->getFloat())); });
	return rviz_prop;
}

PropertyFactory::PropertyFactory() {
	// register some standard types
	registerType<float>(&floatFactory<float>);
	registerType<double>(&floatFactory<double>);
	registerType<std::string>(&stringFactory);
}

PropertyFactory& PropertyFactory::instance() {
	static PropertyFactory instance_;
	return instance_;
}

void PropertyFactory::registerType(const std::string& type_name, const PropertyFactoryFunction& f) {
	if (type_name.empty())
		return;
	property_registry_.insert(std::make_pair(type_name, f));
}

void PropertyFactory::registerStage(const std::type_index& type_index, const PropertyFactory::TreeFactoryFunction& f) {
	stage_registry_.insert(std::make_pair(type_index, f));
}

rviz::Property* PropertyFactory::create(const std::string& prop_name, mtc::Property& prop,
                                        const planning_scene::PlanningScene* scene,
                                        rviz::DisplayContext* display_context) const {
	auto it = property_registry_.find(prop.typeName());
	if (it == property_registry_.end())
		return createDefault(prop_name, prop.typeName(), prop.description(), prop.serialize());
	return it->second(QString::fromStdString(prop_name), prop, scene, display_context);
}

rviz::PropertyTreeModel* PropertyFactory::createPropertyTreeModel(moveit::task_constructor::Stage& stage,
                                                                  const planning_scene::PlanningScene* scene,
                                                                  rviz::DisplayContext* display_context) {
	auto it = stage_registry_.find(typeid(stage));
	if (it == stage_registry_.end())
		return defaultPropertyTreeModel(stage.properties(), scene, display_context);
	return it->second(stage.properties(), scene, display_context);
}

rviz::PropertyTreeModel* PropertyFactory::defaultPropertyTreeModel(mtc::PropertyMap& properties,
                                                                   const planning_scene::PlanningScene* scene,
                                                                   rviz::DisplayContext* display_context) {
	auto root = new rviz::Property();
	addRemainingProperties(root, properties, scene, display_context);
	return new rviz::PropertyTreeModel(root, nullptr);
}

static bool hasChild(rviz::Property* root, const QString& name) {
	for (int i = 0, end = root->numChildren(); i != end; ++i) {
		if (root->childAt(i)->getName() == name)
			return true;
	}
	return false;
}

void PropertyFactory::addRemainingProperties(rviz::Property* root, mtc::PropertyMap& properties,
                                             const planning_scene::PlanningScene* scene,
                                             rviz::DisplayContext* display_context) {
	for (auto& prop : properties) {
		const QString& name = QString::fromStdString(prop.first);
		if (hasChild(root, name))
			continue;

		rviz::Property* rviz_prop = create(prop.first, prop.second, scene, display_context);
		if (!rviz_prop)
			rviz_prop = new rviz::Property(name);
		root->addChild(rviz_prop);
	}

	// just to see something, when no properties are defined
	if (root->numChildren() == 0)
		new rviz::Property("no properties", QVariant(), QString(), root);
}

#ifndef HAVE_YAML
rviz::Property* PropertyFactory::createDefault(const std::string& name, const std::string& type,
                                               const std::string& description, const std::string& value,
                                               rviz::Property* old) {
	if (old) {  // reuse existing Property?
		assert(old->getNameStd() == name);
		old->setDescription(QString::fromStdString(description));
		old->setValue(QString::fromStdString(value));
		return old;
	} else {  // create new Property?
		rviz::Property* result = new rviz::StringProperty(QString::fromStdString(name), QString::fromStdString(value),
		                                                  QString::fromStdString(description));
		result->setReadOnly(true);
		return result;
	}
}
#endif
}
