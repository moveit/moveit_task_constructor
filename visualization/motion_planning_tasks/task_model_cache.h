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

#pragma once

#include "composite_proxy_model.h"
#include "task_model.h"
#include <map>

class QStandardItemModel;
class QStandardItem;
namespace moveit_rviz_plugin {

class TaskDisplay;

/** TaskModelCache maintains the set of all known TaskModels.
 *
 *  This is a singleton instance.
 */
class TaskModelCache : public moveit::tools::CompositeProxyItemModel {
	Q_OBJECT
	struct Entry : public std::pair<TaskModelWeakPtr, QStandardItem*> {
		TaskModelWeakPtr& model() { return first; }
		QStandardItem*& item() { return second; }
	};
	std::map<const TaskDisplay*, Entry> display_map_;
	std::map<std::pair<QString, QString>, TaskModelWeakPtr> task_model_map_;
	QStandardItemModel *root_model_;

	/// class is non-copyable
	TaskModelCache(const TaskModelCache&) = delete;
	void operator=(const TaskModelCache&) = delete;

	QModelIndex getOrCreateItem(TaskModelCache::Entry &e, const QString &name, int insert_row = -1);
	TaskModelPtr getOrCreateTaskModel(Entry &e, const QModelIndex &mount_point);

public:
	TaskModelCache(QObject *parent = nullptr);

	/// get TaskModel for a TaskDisplay
	TaskModelPtr getTaskModel(const TaskDisplay& display,
	                          const QString& task_monitor_topic,
	                          const QString& task_solution_topic);
	/// release TaskModel for given TaskDisplay
	void release(TaskDisplay& display);

	/// call to keep model in sync with display name
	void updateDisplayName(const TaskDisplay& display);

	/// get global TaskModel instance used for panels
	TaskModelPtr getTaskModel();
};

}
