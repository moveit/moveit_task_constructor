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

#include "task_list_model.h"
#include <map>

namespace moveit_rviz_plugin {

/** TaskListModelCache maintains a global TaskListModel comprising all known TaskModels.
 *
 *  This global model instance is used by TaskPanels and can be retrieved via getGlobalModel().
 *  Additionally, this instance maintains a cache for all TaskListModels used e.g. by TaskDisplays.
 *  Displays subscribing to the same topic namespace, will share the same model.
 *
 *  This is a singleton instance.
 */
class TaskListModelCache : public QObject {
	Q_OBJECT

	TaskListModelPtr global_model_;
	std::map<std::string, TaskListModelWeakPtr> cache_;

	/// class is singleton
	TaskListModelCache();
	TaskListModelCache(const TaskListModelCache&) = delete;
	void operator=(const TaskListModelCache&) = delete;

private Q_SLOTS:
	void onInsertTasks(const QModelIndex &parent, int first, int last);
	void onRemoveTasks(const QModelIndex &parent, int first, int last);

public:
	static TaskListModelCache& instance();

	/// get TaskListModel for a TaskDisplay
	TaskListModelPtr getModel(const std::__cxx11::string &ns);

	/// get global TaskListModel instance used for panels
	TaskListModelPtr getGlobalModel();
};

}
