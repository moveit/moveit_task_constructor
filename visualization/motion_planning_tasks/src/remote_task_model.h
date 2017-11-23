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
#include <moveit/visualization_tools/display_solution.h>
#include <memory>

namespace moveit_rviz_plugin {

/** Model representing a remote task
 *
 *  filled via TaskDescription + TaskStatistics messages
 */
class RemoteTaskModel : public BaseTaskModel {
	Q_OBJECT
	class Node;
	Node* const root_;
	planning_scene::PlanningSceneConstPtr scene_;
	std::map<uint32_t, Node*> id_to_stage_;
	std::map<uint32_t, DisplaySolutionPtr> id_to_solution_;

	inline Node* node(const QModelIndex &index) const;
	QModelIndex index(const Node* n) const;

public:
	RemoteTaskModel(const planning_scene::PlanningSceneConstPtr &scene, QObject *parent = nullptr);
	~RemoteTaskModel();
	int rowCount(const QModelIndex &parent = QModelIndex()) const override;

	QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex &index) const override;

	Qt::ItemFlags flags(const QModelIndex & index) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;

	void processStageDescriptions(const moveit_task_constructor_msgs::TaskDescription::_stages_type &msg);
	void processStageStatistics(const moveit_task_constructor_msgs::TaskStatistics::_stages_type &msg);
	DisplaySolutionPtr processSolutionMessage(const moveit_task_constructor_msgs::Solution &msg);

	QAbstractItemModel* getSolutionModel(const QModelIndex& index) override;
	DisplaySolutionPtr getSolution(const QModelIndex &index) override;
};


/** Model representing solutions of a remote task */
class RemoteSolutionModel : public QAbstractListModel {
	Q_OBJECT
	std::vector<uint32_t> ids_;

public:
	RemoteSolutionModel(QObject *parent = nullptr);
	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

	bool processSolutionIDs(const std::vector<uint32_t>& ids);
};

}
