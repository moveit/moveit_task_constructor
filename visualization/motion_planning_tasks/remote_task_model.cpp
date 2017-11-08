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

#include <stdio.h>

#include "remote_task_model.h"
#include <moveit_task_constructor/container.h>
#include <ros/console.h>
#include <qflags.h>

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

enum NodeFlag {
	WAS_VISITED        = 0x01, // indicate that model should emit change notifications
};
typedef QFlags<NodeFlag> NodeFlags;

struct RemoteTaskModel::Node {
	Node *parent_;
	std::vector<std::unique_ptr<Node>> children_;
	QString name_;
	InterfaceFlags interface_flags_;
	NodeFlags node_flags_;

	inline Node(Node *parent) : parent_(parent) {}

	bool setName(const QString& name) {
		if (name == name_) return false;
		name_ = name;
		return true;
	}
};

// return Node* corresponding to index
RemoteTaskModel::Node* RemoteTaskModel::node(const QModelIndex &index) const
{
	if (!index.isValid())
		return root_;

	if (index.model() != this) {
		ROS_ERROR_NAMED("TaskModel", "invalid model in QModelIndex");
		return nullptr;
	}

	// internal pointer refers to parent node
	Node *parent = static_cast<Node*>(index.internalPointer());
	Q_ASSERT(index.row() >= 0 && (size_t)index.row() < parent->children_.size());
	return parent->children_.at(index.row()).get();
}

// return QModelIndex corresponding to Node*
QModelIndex RemoteTaskModel::index(const Node *n) const
{
	if (n == root_)
		return QModelIndex();

	Node *parent = n->parent_;

	// the internal pointer refers to the parent node of n
	for (int row = 0, end = parent->children_.size(); row != end; ++row)
		if (parent->children_.at(row).get() == n)
			return createIndex(row, 0, parent);
	Q_ASSERT(false);
}

RemoteTaskModel::RemoteTaskModel(QObject *parent)
   : BaseTaskModel(parent), root_(new Node(nullptr))
{
	id_to_stage_[0] = root_; // root node has ID 0
}

RemoteTaskModel::~RemoteTaskModel()
{
	delete root_;
}

int RemoteTaskModel::rowCount(const QModelIndex &parent) const
{
	if (parent.column() > 0)
		return 0;

	Node *n = node(parent);
	if (!n) return 0; // invalid model in parent

	n->node_flags_ |= WAS_VISITED;
	return n->children_.size();
}

QModelIndex RemoteTaskModel::index(int row, int column, const QModelIndex &parent) const
{
	if (column < 0 || column >= columnCount())
		return QModelIndex();

	Node *p = node(parent);
	if (!p || row < 0 || (size_t)row >= p->children_.size())
		return QModelIndex();

	// the internal pointer refers to the parent node
	return createIndex(row, column, p);
}

QModelIndex RemoteTaskModel::parent(const QModelIndex &child) const
{
	if (!child.isValid())
		return QModelIndex();

	// the internal pointer refers to the parent node
	Node *p = static_cast<Node*>(child.internalPointer());
	Q_ASSERT(p);
	if (child.model() != this || p == root_)
		return QModelIndex();

	return this->index(p);
}

Qt::ItemFlags RemoteTaskModel::flags(const QModelIndex &index) const
{
	Qt::ItemFlags flags = BaseTaskModel::flags(index);
	if (index.column() == 0)
		flags |= Qt::ItemIsEditable; // name is editable
	return flags;
}

QVariant RemoteTaskModel::data(const QModelIndex &index, int role) const
{
	Node *n = node(index);
	if (!n) return QVariant(); // invalid model in index

	switch (role) {
	case Qt::EditRole:
	case Qt::DisplayRole:
		switch (index.column()) {
		case 0: return n->name_;
		case 1: return 0;
		case 2: return 0;
		}
		break;
	}
	return QVariant();
}

bool RemoteTaskModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	Node *n = node(index);
	if (!n || index.column() != 0 || role != Qt::EditRole)
		return false;
	return n->setName(value.toString());
}

void RemoteTaskModel::processStageDescriptions(const moveit_task_constructor::TaskDescription::_description_type &msg)
{
	// iterate over descriptions and create new / update existing nodes where needed
	for (const auto &s : msg) {
		// find parent node for stage s, this should always exist
		auto parent_it = id_to_stage_.find(s.parent_id);
		if (parent_it == id_to_stage_.end()) {
			ROS_ERROR_NAMED("TaskListModel", "No parent found for stage %d (%s)", s.id, s.name.c_str());
			continue;
		}
		Node *parent = parent_it->second;

		Node*& n = id_to_stage_[s.id];
		if (!n) { // create a new Node if neccessary
			// only emit notify signal if parent node was ever visited
			bool notify = parent->node_flags_ & WAS_VISITED;
			QModelIndex parentIdx = index(parent);
			int row = parent->children_.size();

			if (notify) beginInsertRows(parentIdx, row, row);
			parent->children_.push_back(std::make_unique<Node>(parent));
			if (notify) endInsertRows();

			// store Node* in id_to_stage_
			n = parent->children_.back().get();
		}
		Q_ASSERT(n->parent_ == parent);

		// set content of stage
		bool changed = n->setName(QString::fromStdString(s.name));
		InterfaceFlags old_flags = n->interface_flags_;
		n->interface_flags_ = InterfaceFlags();
		for (auto f : {READS_START, READS_END, WRITES_NEXT_START, WRITES_PREV_END}) {
			if (s.flags & f) n->interface_flags_ |= f;
			else n->interface_flags_ &= ~f;
		}
		changed |= (n->interface_flags_ != old_flags);

		// emit notify about model changes when node was already visited
		if (changed && (n->node_flags_ & WAS_VISITED)) {
			QModelIndex idx = index(n);
			dataChanged(idx, idx);
		}
	}
}

void RemoteTaskModel::processStageStatistics(const moveit_task_constructor::TaskDescription::_statistics_type &msg)
{
}

}
