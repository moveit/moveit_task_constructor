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

#include "task_model_cache.h"
#include "task_display.h"

#include <ros/console.h>
#include <QStandardItemModel>

namespace moveit_rviz_plugin {

TaskModelCache::TaskModelCache(QObject *parent)
   : CompositeProxyItemModel(parent)
   , root_model_ (new QStandardItemModel(this))
{
	QStringList header_labels = {tr("Name")};
	root_model_->setHorizontalHeaderLabels(header_labels);
	header_labels << tr("solved") << tr("failed");
	this->setHorizontalHeaderLabels(header_labels);
	attachModel(QModelIndex(), root_model_);
}

// if e.item() is NULL, create a new item with given name
// return model index of item in TaskModelCache
QModelIndex TaskModelCache::getOrCreateItem(TaskModelCache::Entry& e,
                                            const QString& name, int insert_row)
{
	QStandardItem*& item = e.item();
	// extend root_model_ if new display was inserted
	if (!item) {
		item = new QStandardItem(name);
		item->setFlags({Qt::ItemIsSelectable, Qt::ItemIsEnabled});
		if (insert_row < 0)
			insert_row = root_model_->rowCount();
		root_model_->insertRow(insert_row, item);
		ROS_DEBUG_NAMED("TaskModel", "added display: %s", name.toUtf8().constData());
	}
	return mapFromSource(root_model_->indexFromItem(item), root_model_).front();
}

// attention: as a side-effect, this function will also set existing_model
TaskModelPtr TaskModelCache::getOrCreateTaskModel(TaskModelCache::Entry& e,
                                                  const QModelIndex& mount_point)
{
	TaskModelPtr result;

	if (e.model().expired()) {
		// create new model, store in result (otherwise it would be released again)
		e.model() = result = TaskModelPtr(new TaskModel());
	} else
		result = e.model().lock();

	// store model in cache, and mount it
	attachModel(mount_point, result.get());

	ROS_DEBUG_NAMED("TaskModel", "retrieved task model: %p", result.get());
	return result;
}

TaskModelPtr TaskModelCache::getTaskModel(const TaskDisplay &display,
                                          const QString &task_monitor_topic,
                                          const QString &task_solution_topic)
{
	// retrieve entry from cache or create new one
	Entry& e = display_map_[&display];
	QModelIndex mount_point = getOrCreateItem(e, display.getName());

	if (task_monitor_topic.isEmpty() || task_solution_topic.isEmpty()) {
		// invalid topics: release previously attached model
		ROS_DEBUG_NAMED("TaskModel", "releasing task model: %p  use count: %ld",
		                e.model().lock().get(), e.model().use_count());
		detachModel(mount_point);
		e.model().reset();
		return TaskModelPtr();
	} else {
		// retrieve existing model for given topic pair
		TaskModelWeakPtr& existing_model = task_model_map_[std::make_pair(task_monitor_topic, task_solution_topic)];
		e.model() = existing_model; // reuse model from cache
		TaskModelPtr result = getOrCreateTaskModel(e, mount_point);
		existing_model = e.model(); // store (newly created) model in cache
		return result;
	}
}

void TaskModelCache::release(TaskDisplay &display)
{
	auto it = display_map_.find(&display);
	if (it == display_map_.end())
		return;
	// remove display from root_model_
	root_model_->removeRow(root_model_->indexFromItem(it->second.item()).row());
	// remove entry from display_map_
	display_map_.erase(it);
}

void TaskModelCache::updateDisplayName(const TaskDisplay &display)
{
	auto it = display_map_.find(&display);
	if (it == display_map_.end())
		return;
	it->second.item()->setText(display.getName());
}

TaskModelPtr TaskModelCache::getTaskModel() {
	// retrieve entry from cache or create new one
	Entry& e = display_map_[nullptr];
	QModelIndex mount_point = getOrCreateItem(e, "Tasks Panel", 0);
	return getOrCreateTaskModel(e, mount_point);
}

}
