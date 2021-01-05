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
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>
#include <rclcpp/client.hpp>
#include <memory>
#include <limits>

namespace moveit_rviz_plugin {

class RemoteSolutionModel;
/** Model representing a remote task
 *
 *  filled via TaskDescription + TaskStatistics messages
 */
class RemoteTaskModel : public BaseTaskModel
{
	Q_OBJECT
	struct Node;
	Node* const root_;
	rclcpp::Client<moveit_task_constructor_msgs::srv::GetSolution>::SharedPtr get_solution_client_;
	// TODO(JafarAbdi): We shouldn't need this, replace with callback groups (should be fully available in Galactic)
	// RViz have a single threaded executor which is causing the get_solution_client_ to timeout without
	// getting the result
	rclcpp::Node::SharedPtr node_;

	std::map<uint32_t, Node*> id_to_stage_;
	std::map<uint32_t, DisplaySolutionPtr> id_to_solution_;

	inline Node* node(const QModelIndex& index) const;
	QModelIndex index(const Node* n) const;

	Node* node(uint32_t stage_id) const;
	inline RemoteSolutionModel* getSolutionModel(uint32_t stage_id) const;
	void setSolutionData(const moveit_task_constructor_msgs::msg::SolutionInfo& info);

public:
	RemoteTaskModel(const std::string& service_name, const planning_scene::PlanningSceneConstPtr& scene,
	                rviz_common::DisplayContext* display_context, QObject* parent = nullptr);
	~RemoteTaskModel() override;

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;

	QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex& index) const override;

	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

	QModelIndex indexFromStageId(size_t id) const override;
	void processStageDescriptions(const moveit_task_constructor_msgs::msg::TaskDescription::_stages_type& msg);
	void processStageStatistics(const moveit_task_constructor_msgs::msg::TaskStatistics::_stages_type& msg);
	DisplaySolutionPtr processSolutionMessage(const moveit_task_constructor_msgs::msg::Solution& msg);

	QAbstractItemModel* getSolutionModel(const QModelIndex& index) override;
	DisplaySolutionPtr getSolution(const QModelIndex& index) override;

	rviz_common::properties::PropertyTreeModel* getPropertyModel(const QModelIndex& index) override;
};

/** Model representing solutions of a remote task */
class RemoteSolutionModel : public QAbstractTableModel
{
	Q_OBJECT
	struct Data
	{
		uint32_t id;
		double cost;  // nan if unknown, inf if failed
		QString comment;
		uint32_t creation_rank;  // rank, ordered by creation
		uint32_t cost_rank;  // rank, ordering by cost

		Data(uint32_t id, float cost, uint32_t cost_rank, const QString& name = QString())
		  : id(id), cost(cost), comment(name), creation_rank(0), cost_rank(cost_rank) {}

		inline bool operator<(const Data& other) const { return this->id < other.id; }
	};
	// successful and failed solutions ordered by id / creation
	using DataList = std::list<Data>;
	DataList data_;
	size_t num_failed_data_ = 0;  // number of failed solutions in data_
	size_t num_failed_ = 0;  // number of reported failures
	double total_compute_time_ = 0.0;

	// solutions ordered (by default according to cost)
	int sort_column_ = -1;
	Qt::SortOrder sort_order_ = Qt::AscendingOrder;
	double max_cost_ = std::numeric_limits<double>::infinity();
	std::vector<DataList::iterator> sorted_;

	inline bool isVisible(const Data& item) const;
	void processSolutionIDs(const std::vector<uint32_t>& ids, bool successful);
	void sortInternal();

public:
	RemoteSolutionModel(QObject* parent = nullptr);

	uint numSuccessful() const { return data_.size() - num_failed_data_; }
	uint numFailed() const { return num_failed_; }
	double totalComputeTime() const { return total_compute_time_; }

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	void sort(int column, Qt::SortOrder order) override;

	void setSolutionData(uint32_t id, float cost, const QString& comment);
	void processSolutionIDs(const std::vector<uint32_t>& successful, const std::vector<uint32_t>& failed,
	                        size_t num_failed, double total_compute_time);
};
}  // namespace moveit_rviz_plugin
