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
#include "properties/property_factory.h"
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/properties.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rclcpp/logging.hpp>

#include <QApplication>
#include <QPalette>
#include <qglobal.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.task_list_model");

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

enum NodeFlag
{
	WAS_VISITED = 0x01,  // indicate that model should emit change notifications
	NAME_CHANGED = 0x02,  // indicate that name was manually changed
};
using NodeFlags = QFlags<NodeFlag>;

struct RemoteTaskModel::Node
{
	Node* parent_;
	std::vector<std::unique_ptr<Node>> children_;
	QString name_;
	InterfaceFlags interface_flags_;
	NodeFlags node_flags_;
	std::unique_ptr<RemoteSolutionModel> solutions_;
	std::unique_ptr<rviz_common::properties::PropertyTreeModel> property_tree_;
	std::map<std::string, Property> properties_;

	inline Node(Node* parent) : parent_(parent) {
		solutions_.reset(new RemoteSolutionModel());
		property_tree_.reset(new rviz_common::properties::PropertyTreeModel(new rviz_common::properties::Property()));
	}

	bool setName(const QString& name) {
		if (name == name_)
			return false;
		name_ = name;
		return true;
	}

	void setProperties(const std::vector<moveit_task_constructor_msgs::msg::Property>& props,
	                   const planning_scene::PlanningSceneConstPtr& scene_,
	                   rviz_common::DisplayContext* display_context_);
	rviz_common::properties::Property* createProperty(const moveit_task_constructor_msgs::msg::Property& prop,
	                                                  rviz_common::properties::Property* old,
	                                                  const planning_scene::PlanningSceneConstPtr& scene_,
	                                                  rviz_common::DisplayContext* display_context_);
};

void RemoteTaskModel::Node::setProperties(const std::vector<moveit_task_constructor_msgs::msg::Property>& props,
                                          const planning_scene::PlanningSceneConstPtr& scene_,
                                          rviz_common::DisplayContext* display_context_) {
	// insert properties in same order as reported in description
	rviz_common::properties::Property* root = property_tree_->getRoot();
	int index = 0;  // current child index in root
	for (const auto& prop : props) {
		int num = root->numChildren();
		// find first child with name >= it->name
		int next = index;
		while (next < num && root->childAt(next)->getName().toStdString() < prop.name)
			++next;
		// and remove all children in range [index, next) at once
		root->removeChildren(index, next - index);
		num = root->numChildren();

		// if names differ, insert a new child, otherwise reuse existing
		rviz_common::properties::Property* old_child = index < num ? root->childAt(index) : nullptr;
		if (old_child && old_child->getName().toStdString() != prop.name)
			old_child = nullptr;

		rviz_common::properties::Property* new_child = createProperty(prop, old_child, scene_, display_context_);
		if (new_child != old_child)
			root->addChild(new_child, index);
		++index;
	}
	// remove remaining children
	root->removeChildren(index, root->numChildren() - index);
}

rviz_common::properties::Property* RemoteTaskModel::Node::createProperty(
    const moveit_task_constructor_msgs::msg::Property& prop, rviz_common::properties::Property* old,
    const planning_scene::PlanningSceneConstPtr& scene_, rviz_common::DisplayContext* display_context_) {
	auto& factory = PropertyFactory::instance();
	// try to deserialize from msg (using registered functions)
	boost::any value = Property::deserialize(prop.type, prop.value);
	if (!value.empty()) {  // if successful, create rviz_common::properties::Property from mtc::Property using factory
		                    // methods
		auto it = properties_.insert(std::make_pair(prop.name, Property())).first;
		it->second.setDescription(prop.description);
		it->second.setValue(value);
		if (rviz_common::properties::Property* rviz_prop =
		        factory.create(prop.name, it->second, scene_.get(), display_context_)) {
			rviz_prop->setReadOnly(true);
			return rviz_prop;
		} else
			properties_.erase(it);
	}

	// otherwise create default, read-only rviz_common::properties::Property by parsing serialized YAML
	return factory.createDefault(prop.name, prop.type, prop.description, prop.value, old);
}

// return Node* corresponding to index
RemoteTaskModel::Node* RemoteTaskModel::node(const QModelIndex& index) const {
	if (!index.isValid())
		return root_;

	if (index.model() != this) {
		RCLCPP_ERROR(LOGGER, "invalid model in QModelIndex");
		return nullptr;
	}

	// internal pointer refers to parent node
	Node* parent = static_cast<Node*>(index.internalPointer());
	Q_ASSERT(index.row() >= 0 && static_cast<size_t>(index.row()) < parent->children_.size());
	return parent->children_.at(index.row()).get();
}

// return Node* corresponding to stage_id
RemoteTaskModel::Node* RemoteTaskModel::node(uint32_t stage_id) const {
	auto it = id_to_stage_.find(stage_id);
	return (it == id_to_stage_.end()) ? nullptr : it->second;
}

// return QModelIndex corresponding to Node*
QModelIndex RemoteTaskModel::index(const Node* n) const {
	if (n == root_)
		return QModelIndex();

	Node* parent = n->parent_;

	// the internal pointer refers to the parent node of n
	for (int row = 0, end = parent->children_.size(); row != end; ++row)
		if (parent->children_.at(row).get() == n)
			return createIndex(row, 0, parent);
	Q_ASSERT(false);
	return QModelIndex();
}

RemoteTaskModel::RemoteTaskModel(const std::string& service_name, const planning_scene::PlanningSceneConstPtr& scene,
                                 rviz_common::DisplayContext* display_context, QObject* parent)
  : BaseTaskModel(scene, display_context, parent), root_(new Node(nullptr)) {
	id_to_stage_[0] = root_;  // root node has ID 0
	// Add random ID to prevent warnings about multiple publishers within the same node
	node_ = rclcpp::Node::make_shared("get_solution_node_" + std::to_string(reinterpret_cast<std::size_t>(this)),
	                                  "/moveit_task_constructor/remote_task_model");
	// service to request solutions
	get_solution_client_ = node_->create_client<moveit_task_constructor_msgs::srv::GetSolution>(service_name);
}

RemoteTaskModel::~RemoteTaskModel() {
	delete root_;
}

int RemoteTaskModel::rowCount(const QModelIndex& parent) const {
	if (parent.column() > 0)
		return 0;

	Node* n = node(parent);
	if (!n)
		return 0;  // invalid model in parent

	return n->children_.size();
}

QModelIndex RemoteTaskModel::index(int row, int column, const QModelIndex& parent) const {
	if (column < 0 || column >= columnCount())
		return QModelIndex();

	Node* p = node(parent);
	if (!p || row < 0 || static_cast<size_t>(row) >= p->children_.size())
		return QModelIndex();

	p->children_[row]->node_flags_ |= WAS_VISITED;
	// the internal pointer refers to the parent node
	return createIndex(row, column, p);
}

QModelIndex RemoteTaskModel::parent(const QModelIndex& child) const {
	if (!child.isValid())
		return QModelIndex();

	// the internal pointer refers to the parent node
	Node* p = static_cast<Node*>(child.internalPointer());
	Q_ASSERT(p);
	if (child.model() != this || p == root_)
		return QModelIndex();

	return this->index(p);
}

QVariant RemoteTaskModel::data(const QModelIndex& index, int role) const {
	Node* n = node(index);
	if (!n)
		return QVariant();  // invalid model in index

	switch (role) {
		case Qt::EditRole:
		case Qt::DisplayRole:
			switch (index.column()) {
				case 0:
					return n->name_;
				case 1:
					return n->solutions_->numSuccessful();
				case 2:
					return n->solutions_->numFailed();
				case 3:
					return QLocale().toString(n->solutions_->totalComputeTime(), 'f', 4);
			}
			break;
		case Qt::ForegroundRole:
			if (index.column() == 0 && !index.parent().isValid())
				return (flags_ & IS_DESTROYED) ? QColor(Qt::red) : QApplication::palette().text().color();
			break;
		case Qt::DecorationRole:
			if (index.column() == 0 && index.parent().isValid())
				return flowIcon(n->interface_flags_);
			break;
	}

	return BaseTaskModel::data(index, role);
}

bool RemoteTaskModel::setData(const QModelIndex& index, const QVariant& value, int role) {
	Node* n = node(index);
	if (!n || index.column() != 0 || role != Qt::EditRole)
		return false;
	n->setName(value.toString());
	n->node_flags_ |= NAME_CHANGED;
	dataChanged(index, index);
	return true;
}

QModelIndex RemoteTaskModel::indexFromStageId(size_t id) const {
	Node* n = node(id);
	return n ? index(n) : QModelIndex();
}

void RemoteTaskModel::processStageDescriptions(
    const moveit_task_constructor_msgs::msg::TaskDescription::_stages_type& msg) {
	// iterate over descriptions and create new / update existing nodes where needed
	for (const auto& s : msg) {
		// find parent node for stage s, this should always exist
		auto parent_it = id_to_stage_.find(s.parent_id);
		if (parent_it == id_to_stage_.end()) {
			RCLCPP_ERROR(LOGGER, "No parent found for stage %d (%s)", s.id, s.name.c_str());
			continue;
		}
		Node* parent = parent_it->second;

		Node*& n = id_to_stage_[s.id];
		if (!n) {  // create a new Node if neccessary
			// only emit notify signal if parent node was ever visited
			bool notify = parent->node_flags_ & WAS_VISITED;
			QModelIndex parent_idx = index(parent);
			int row = parent->children_.size();

			if (notify)
				beginInsertRows(parent_idx, row, row);
			parent->children_.push_back(std::make_unique<Node>(parent));
			if (notify)
				endInsertRows();

			// store Node* in id_to_stage_
			n = parent->children_.back().get();
		}
		Q_ASSERT(n->parent_ == parent);

		// set content of stage
		bool changed = false;
		if (!(n->node_flags_ & NAME_CHANGED))  // avoid overwriting a manually changed name
			changed |= n->setName(QString::fromStdString(s.name));

		n->setProperties(s.properties, scene_, display_context_);

		InterfaceFlags old_flags = n->interface_flags_;
		n->interface_flags_ = InterfaceFlags();
		for (auto f : { READS_START, READS_END, WRITES_NEXT_START, WRITES_PREV_END }) {
			if (s.flags & f)
				n->interface_flags_ |= f;
			else
				n->interface_flags_ &= ~f;
		}
		changed |= (n->interface_flags_ != old_flags);

		// emit notify about model changes when node was already visited
		if (changed && (n->node_flags_ & WAS_VISITED)) {
			QModelIndex idx = index(n);
			dataChanged(idx, idx.sibling(idx.row(), 2));
		}
	}

	if (msg.empty()) {
		flags_ |= IS_DESTROYED;
		dataChanged(index(0, 0), index(0, 2));
	}
}

void RemoteTaskModel::processStageStatistics(
    const moveit_task_constructor_msgs::msg::TaskStatistics::_stages_type& msg) {
	// iterate over statistics and update node's solutions where needed
	for (const auto& s : msg) {
		// find node for stage s, this should always exist
		auto it = id_to_stage_.find(s.id);
		if (it == id_to_stage_.end()) {
			RCLCPP_ERROR(LOGGER, "No stage %d", s.id);
			continue;
		}
		Node* n = it->second;
		n->solutions_->processSolutionIDs(s.solved, s.failed, s.num_failed, s.total_compute_time);

		// emit notify about model changes when node was already visited
		if (n->node_flags_ & WAS_VISITED) {
			QModelIndex idx = index(n);
			dataChanged(idx.sibling(idx.row(), 1), idx.sibling(idx.row(), 3));
		}
	}
}

void RemoteTaskModel::setSolutionData(const moveit_task_constructor_msgs::msg::SolutionInfo& info) {
	if (info.id == 0)
		return;
	if (RemoteSolutionModel* m = getSolutionModel(info.stage_id))
		m->setSolutionData(info.id, info.cost, QString::fromStdString(info.comment));
}

DisplaySolutionPtr RemoteTaskModel::processSolutionMessage(const moveit_task_constructor_msgs::msg::Solution& msg) {
	DisplaySolutionPtr s(new DisplaySolution);
	s->setFromMessage(scene_->diff(), msg);

	// store sub solution data in model
	for (const auto& sub : msg.sub_solution)
		setSolutionData(sub.info);
	for (const auto& sub : msg.sub_trajectory)
		setSolutionData(sub.info);

	// caching is only enabled for top-level solutions (stage_id == 1)
	// otherwise we would store PlanningScenes over and over
	if (!msg.sub_solution.empty() && msg.sub_solution.front().info.stage_id == 1 &&
	    msg.sub_solution.front().info.id != 0) {
		// cache solution for future use
		id_to_solution_[msg.sub_solution.front().info.id] = s;

		// cache DisplaySolutions for all individual sub trajectories
		uint i = 0;
		for (const auto& t : msg.sub_trajectory) {
			if (t.info.id == 0)
				continue;  // invalid id
			DisplaySolutionPtr& sub =
			    id_to_solution_.insert(std::make_pair(t.info.id, DisplaySolutionPtr())).first->second;
			if (!sub)
				sub.reset(new DisplaySolution(*s, i));
			i++;
		}
	}

	return s;
}

RemoteSolutionModel* RemoteTaskModel::getSolutionModel(uint32_t stage_id) const {
	Node* n = node(stage_id);
	return n ? n->solutions_.get() : nullptr;
}

QAbstractItemModel* RemoteTaskModel::getSolutionModel(const QModelIndex& index) {
	Node* n = node(index);
	if (!n)
		return nullptr;
	return n->solutions_.get();
}

DisplaySolutionPtr RemoteTaskModel::getSolution(const QModelIndex& index) {
	Q_ASSERT(index.isValid());

	uint32_t id = index.sibling(index.row(), 0).data(Qt::UserRole).toUInt();
	auto it = id_to_solution_.find(id);
	if (it == id_to_solution_.cend()) {
		// TODO: try to assemble (and cache) the solution from known leaves
		// to avoid some communication overhead
		DisplaySolutionPtr result;
		if (!(flags_ & IS_DESTROYED)) {
			if (get_solution_client_->service_is_ready()) {
				// request solution via service
				auto request = std::make_shared<moveit_task_constructor_msgs::srv::GetSolution::Request>();
				request->solution_id = id;
				auto result_future = get_solution_client_->async_send_request(request);
				if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
					id_to_solution_[id] = result = processSolutionMessage(result_future.get()->solution);
					return result;
				}
			}
			// on failure mark remote task as destroyed: don't retrieve more solutions
			get_solution_client_.reset();
			node_.reset();
			flags_ |= IS_DESTROYED;
		}
		return result;
	}
	return it->second;
}

rviz_common::properties::PropertyTreeModel* RemoteTaskModel::getPropertyModel(const QModelIndex& index) {
	Node* n = node(index);
	if (!n)
		return nullptr;
	return n->property_tree_.get();
}

namespace detail {
// method used by sorted_ container (requiring an additional dereference to access id)
template <class T>
typename T::iterator findById(T& c, decltype((*c.cbegin())->id) id) {
	return std::find_if(c.begin(), c.end(), [id](const typename T::value_type& item) { return item->id == id; });
}

// insert new Data item into data_ container
template <class T>
typename T::iterator insert(T& c, typename T::value_type&& item) {
	auto p = std::equal_range(c.begin(), c.end(), item);
	if (p.first == p.second)  // new item
		return c.insert(p.second, std::move(item));
	else
		return p.first;
}
}  // namespace detail

RemoteSolutionModel::RemoteSolutionModel(QObject* parent) : QAbstractTableModel(parent) {}

int RemoteSolutionModel::rowCount(const QModelIndex& /*parent*/) const {
	return sorted_.size();
}

int RemoteSolutionModel::columnCount(const QModelIndex& /*parent*/) const {
	return 3;
}

QVariant RemoteSolutionModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Horizontal) {
		switch (role) {
			case Qt::DisplayRole:
				switch (section) {
					case 0:
						return tr("#");
					case 1:
						return tr("cost");
					case 2:
						return tr("comment");
				}
				break;
			case Qt::TextAlignmentRole:
				return Qt::AlignLeft;
		}
	}
	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant RemoteSolutionModel::data(const QModelIndex& index, int role) const {
	Q_ASSERT(index.isValid());
	Q_ASSERT(!index.parent().isValid());

	const Data& item = *sorted_[index.row()];

	switch (role) {
		case Qt::UserRole:
			return item.id;

		case Qt::ToolTipRole:
#if 0  // show internal solution id in first column
			if (index.column() == 0)
				return item.id;
#endif
			// usually just show the comment
			return item.comment;
			break;

		case Qt::DisplayRole:
			switch (index.column()) {
				case 0:
					return item.creation_rank;
				case 1:
					if (std::isinf(item.cost))
						return tr(u8"∞");
					if (std::isnan(item.cost))
						return QVariant();
					return QLocale().toString(item.cost, 'f', 4);
				case 2:
					return item.comment;
			}
			break;

		case Qt::ForegroundRole:
			if (std::isinf(item.cost))
				return QColor(Qt::red);
			break;

		case Qt::TextAlignmentRole:
			return index.column() == 2 ? Qt::AlignLeft : Qt::AlignRight;
	}
	return QVariant();
}

void RemoteSolutionModel::setSolutionData(uint32_t id, float cost, const QString& comment) {
	// retrieve iterator and row corresponding to id
	auto sit = detail::findById(sorted_, id);
	int row = (sit != sorted_.end()) ? sit - sorted_.begin() : -1;
	auto it = (sit != sorted_.end()) ? *sit : detail::insert(data_, Data(id, cost, 0, comment));

	QModelIndex tl, br;
	Data& item = *it;
	if (item.cost != cost) {
		item.cost = cost;
		tl = br = index(row, 1);
	}
	if (item.comment != comment) {
		item.comment = comment;
		br = index(row, 2);
		if (!tl.isValid())
			tl = br;
	}
	if (tl.isValid())
		Q_EMIT dataChanged(tl, br);

	if (row < 0 && isVisible(*it))  // item was newly created: inform views
		sortInternal();
}

void RemoteSolutionModel::sort(int column, Qt::SortOrder order) {
	if (sort_column_ == column && sort_order_ == order)
		return;  // nothing to do

	sort_column_ = column;
	sort_order_ = order;

	sortInternal();
}

void RemoteSolutionModel::sortInternal() {
	Q_EMIT layoutAboutToBeChanged();
	QModelIndexList old_indexes = persistentIndexList();
	std::vector<DataList::iterator> old_sorted;
	std::swap(sorted_, old_sorted);

	// create new order in sorted_
	for (auto it = data_.begin(), end = data_.end(); it != end; ++it)
		if (isVisible(*it))
			sorted_.push_back(it);

	if (sort_column_ >= 0) {
		std::sort(sorted_.begin(), sorted_.end(),
		          [this](const DataList::iterator& left, const DataList::iterator& right) {
			          int comp = 0;
			          switch (sort_column_) {
				          case 1:  // cost order
					          if (left->cost_rank < right->cost_rank)
						          comp = -1;
					          else if (left->cost_rank > right->cost_rank)
						          comp = 1;
					          break;
				          case 2:  // comment
					          comp = left->comment.compare(right->comment);
					          break;
			          }
			          if (comp == 0)  // if still undecided, id decides
				          comp = (left->id < right->id ? -1 : 1);
			          return (sort_order_ == Qt::AscendingOrder) ? (comp < 0) : (comp >= 0);
		          });
	}

	// map old indexes to new ones
	std::map<int, int> old_to_new_row;
	QModelIndexList new_indexes;
	for (int i = 0, end = old_indexes.count(); i != end; ++i) {
		int old_row = old_indexes[i].row();
		auto it_inserted = old_to_new_row.insert(std::make_pair(old_row, -1));
		if (it_inserted.second) {  // newly inserted: find new row index
			auto it = detail::findById(sorted_, old_sorted[old_row]->id);
			if (it != sorted_.cend())
				it_inserted.first->second = it - sorted_.begin();
		}
		new_indexes.append(index(it_inserted.first->second, old_indexes[i].column()));
	}

	changePersistentIndexList(old_indexes, new_indexes);
	Q_EMIT layoutChanged();
}

// process solution ids received in stage statistics
void RemoteSolutionModel::processSolutionIDs(const std::vector<uint32_t>& successful,
                                             const std::vector<uint32_t>& failed, size_t num_failed,
                                             double total_compute_time) {
	// append new items to the end of data_
	processSolutionIDs(successful, true);
	processSolutionIDs(failed, false);

	// assign consecutive creation ranks
	uint32_t rank = 0;
	for (auto& item : data_)
		item.creation_rank = ++rank;

	// the task may not report failure ids (in failed),
	// but it may report the overall number of failures
	num_failed_data_ = failed.size();  // needed to compute number of successes
	num_failed_ = std::max(num_failed, num_failed_data_);
	total_compute_time_ = total_compute_time;

	sortInternal();
}

void RemoteSolutionModel::processSolutionIDs(const std::vector<uint32_t>& ids, bool successful) {
	// Interface axiom: ids are sorted by cost
	// insert them into data_ list sorted by id
	double default_cost =
	    successful ? std::numeric_limits<double>::quiet_NaN() : std::numeric_limits<double>::infinity();
	uint32_t cost_rank = 0;
	for (const uint32_t id : ids) {
		uint32_t rank = successful ? ++cost_rank : std::numeric_limits<uint32_t>::max();
		auto it = detail::insert(data_, Data(id, default_cost, rank));
		Q_ASSERT(it->id == id);
		it->cost_rank = rank;
	}
}

bool RemoteSolutionModel::isVisible(const RemoteSolutionModel::Data& item) const {
	return std::isnan(item.cost) || item.cost <= max_cost_;
}
}  // namespace moveit_rviz_plugin
