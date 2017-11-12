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

#include "task_list_model_cache.h"

namespace moveit_rviz_plugin {

TaskListModelCache::TaskListModelCache()
{
	global_model_.reset(new TaskListModel());
}

TaskListModelCache &TaskListModelCache::instance()
{
	static TaskListModelCache instance_;
	return instance_;
}

TaskListModelPtr TaskListModelCache::getModel(const std::string& ns)
{
	if (ns.empty()) {
		return TaskListModelPtr();
	} else {
		// retrieve existing model for given topic pair
		TaskListModelWeakPtr& model = cache_[ns];
		TaskListModelPtr result;

		if (model.expired()) {
			// create new model, store in result (otherwise it would be released again)
			model = result = TaskListModelPtr(new TaskListModel());

			// connect newly created TaskListModel to global model
			connect(result.get(), SIGNAL(rowsInserted(QModelIndex,int,int)),
			        this, SLOT(onInsertTasks(QModelIndex,int,int)));
			connect(result.get(), SIGNAL(rowsAboutToBeRemoved(QModelIndex,int,int)),
			        this, SLOT(onRemoveTasks(QModelIndex,int,int)));
		} else
			result = model.lock();

		return result;
	}
}

TaskListModelPtr TaskListModelCache::getGlobalModel() {
	return global_model_;
}

void TaskListModelCache::onInsertTasks(const QModelIndex &parent, int first, int last)
{
	if (parent.isValid())
		return; // we are only interested in top-level insertions

	TaskListModel *m = static_cast<TaskListModel*>(sender());
	for(; first <= last; ++first) {
		BaseTaskModel *t = m->getTask(first);
		global_model_->insertTask(t);
	}
}

void TaskListModelCache::onRemoveTasks(const QModelIndex &parent, int first, int last)
{
	if (parent.isValid())
		return; // we are only interested in top-level insertions

	TaskListModel *m = static_cast<TaskListModel*>(sender());
	for(; first <= last; ++first) {
		BaseTaskModel *t = m->getTask(first);
		global_model_->removeTask(t);
	}
}

}
