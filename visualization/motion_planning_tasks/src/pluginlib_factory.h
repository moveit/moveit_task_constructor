/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#pragma once

#include <QHash>
#include <QString>
#include <QStringList>

#include <string>
#include <vector>

#ifndef Q_MOC_RUN
#include <pluginlib/class_loader.hpp>
#include <rviz_common/load_resource.hpp>
#include <functional>
#endif

#include <rclcpp/logging.hpp>
#include <rviz_common/factory/factory.hpp>

namespace moveit_rviz_plugin {

/** Templated factory to create objects of a given pluginlib base class type.
 *  This is a slightly modified version of rviz::PluginlibFactory, providing a custom mime type.
 */
template <class Type>
class PluginlibFactory : public rviz_common::Factory
{
private:
	struct BuiltInClassRecord
	{
		QString class_id_;
		QString package_;
		QString name_;
		QString description_;
		std::function<Type*()> factory_function_;
	};

public:
	PluginlibFactory(const QString& package, const QString& base_class_type)
	  : mime_type_(QString("application/%1/%2").arg(package, base_class_type)) {
		class_loader_ = new pluginlib::ClassLoader<Type>(package.toStdString(), base_class_type.toStdString());
	}
	~PluginlibFactory() override { delete class_loader_; }

	/// retrieve mime type used for given factory
	QString mimeType() const { return mime_type_; }

	std::vector<rviz_common::PluginInfo> getDeclaredPlugins() override {
		std::vector<rviz_common::PluginInfo> plugins;
		for (auto iter = built_ins_.cbegin(); iter != built_ins_.cend(); ++iter)
			plugins.emplace_back(getPluginInfo(iter.key()));
		for (const auto& id : class_loader_->getDeclaredClasses()) {
			auto sid = QString::fromStdString(id);
			if (std::find_if(plugins.cbegin(), plugins.cend(), [&sid](const rviz_common::PluginInfo& plugin_info) {
				    return plugin_info.id == sid;
			    }) != plugins.cend())
				continue;  // built_in take precedence
			plugins.emplace_back(getPluginInfo(QString::fromStdString(id)));
		}
		return plugins;
	}

	rviz_common::PluginInfo getPluginInfo(const QString& class_id) const override {
		rviz_common::PluginInfo info;
		const auto iter = built_ins_.find(class_id);
		if (iter != built_ins_.end()) {
			info.id = iter->class_id_;
			info.name = iter->name_;
			info.package = iter->package_;
			info.description = iter->description_;
			info.icon = getIcon(info);
			return info;
		}
		auto class_id_std = class_id.toStdString();
		info.id = class_id;
		info.name = QString::fromStdString(class_loader_->getName(class_id_std));
		info.package = QString::fromStdString(class_loader_->getClassPackage(class_id_std));
		info.description = QString::fromStdString(class_loader_->getClassDescription(class_id_std));
		info.icon = getIcon(info);
		return info;
	}

	virtual QString getPluginManifestPath(const QString& class_id) const {
		auto it = built_ins_.find(class_id);
		if (it != built_ins_.end()) {
			return "";
		}
		return QString::fromStdString(class_loader_->getPluginManifestPath(class_id.toStdString()));
	}

	QIcon getIcon(const rviz_common::PluginInfo& info) const {
		QIcon icon = rviz_common::loadPixmap("package://" + info.package + "/icons/classes/" + info.name + ".svg");
		if (icon.isNull()) {
			icon = rviz_common::loadPixmap("package://" + info.package + "/icons/classes/" + info.name + ".png");
			if (icon.isNull()) {
				icon = rviz_common::loadPixmap("package://rviz/icons/default_class_icon.png");
			}
		}
		return icon;
	}

	void addBuiltInClass(const QString& package, const QString& name, const QString& description,
	                     const std::function<Type*()>& factory_function) {
		BuiltInClassRecord record;
		record.class_id_ = package + "/" + name;
		record.package_ = package;
		record.name_ = name;
		record.description_ = description;
		record.factory_function_ = factory_function;
		built_ins_[record.class_id_] = record;
	}
	template <class Derived>
	void addBuiltInClass(const QString& name, const QString& description) {
		addBuiltInClass("Built Ins", name, description, [] { return new Derived(); });
	}

	/** @brief Instantiate and return a instance of a subclass of Type using our
	 *         pluginlib::ClassLoader.
	 * @param class_id A string identifying the class uniquely among
	 *        classes of its parent class.  rviz::GridDisplay might be
	 *        rviz/Grid, for example.
	 * @param error_return If non-NULL and there is an error, *error_return is set to a description of the problem.
	 * @return A new instance of the class identified by class_id, or NULL if there was an error.
	 *
	 * If makeRaw() returns NULL and error_return is not NULL, *error_return will be set.
	 * On success, *error_return will not be changed. */
	virtual Type* makeRaw(const QString& class_id, QString* error_return = nullptr) {
		typename QHash<QString, BuiltInClassRecord>::const_iterator iter = built_ins_.find(class_id);
		if (iter != built_ins_.end()) {
			Type* instance = iter->factory_function_();
			if (instance == nullptr && error_return != nullptr) {
				*error_return = "Factory function for built-in class '" + class_id + "' returned NULL.";
			}
			return instance;
		}
		try {
			return class_loader_->createUnmanagedInstance(class_id.toStdString());
		} catch (pluginlib::PluginlibException& ex) {
			RCLCPP_ERROR(rclcpp::get_logger("moveit_task_constructor_visualization.pluginlib_factory"),
			             "PluginlibFactory: The plugin for class '%s' failed to load.  Error: %s", qPrintable(class_id),
			             ex.what());
			if (error_return) {
				*error_return = QString::fromStdString(ex.what());
			}
			return nullptr;
		}
	}

private:
	const QString mime_type_;
	pluginlib::ClassLoader<Type>* class_loader_;
	QHash<QString, BuiltInClassRecord> built_ins_;
};
}  // namespace moveit_rviz_plugin
