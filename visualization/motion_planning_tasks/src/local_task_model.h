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
#include <moveit/task_constructor/task.h>

namespace moveit_rviz_plugin {

class LocalTaskModel : public BaseTaskModel, public moveit::task_constructor::Task
{
	Q_OBJECT
	using Node = moveit::task_constructor::Stage;
	Node* root_;
	StageFactoryPtr stage_factory_;
	std::map<Node*, rviz_common::properties::PropertyTreeModel*> properties_;

	inline Node* node(const QModelIndex& index) const;
	QModelIndex index(Node* n) const;

public:
	LocalTaskModel(ContainerBase::pointer&& container, const planning_scene::PlanningSceneConstPtr& scene,
	               rviz_common::DisplayContext* display_context, QObject* parent = nullptr);
	int rowCount(const QModelIndex& parent = QModelIndex()) const override;

	QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex& index) const override;

	Qt::ItemFlags flags(const QModelIndex& index) const override;
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

	bool removeRows(int row, int count, const QModelIndex& parent) override;

	/// providing a StageFactory makes the model accepting drops
	void setStageFactory(const StageFactoryPtr& factory) override;
	bool dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column,
	                  const QModelIndex& parent) override;

	QModelIndex indexFromStageId(size_t id) const override;

	QAbstractItemModel* getSolutionModel(const QModelIndex& index) override;
	DisplaySolutionPtr getSolution(const QModelIndex& index) override;

	rviz_common::properties::PropertyTreeModel* getPropertyModel(const QModelIndex& index) override;
};
}  // namespace moveit_rviz_plugin
