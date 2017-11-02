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

#include "local_task_model.h"
#include <container_p.h>

#include <ros/console.h>

using namespace moveit::task_constructor;

namespace moveit_rviz_plugin {

// return Node* corresponding to index
LocalTaskModel::Node* LocalTaskModel::node(const QModelIndex &index) const
{
	if (!index.isValid())
		return root_;

	Q_ASSERT(index.model() == this);
	// internal pointer refers to index' pimpl()
	return static_cast<Node*>(index.internalPointer());
}

// return QModelIndex corresponding to Node*
QModelIndex LocalTaskModel::index(Node *n) const
{
	if (n == root_)
		return QModelIndex();

	const ContainerBasePrivate *parent = n->parent()->pimpl();

	// the internal pointer refers to n
	int row = 0;
	for (const auto& child :  parent->children()) {
		if (child->pimpl() == n)
			return createIndex(row, 0, n);
		++row;
	}
	Q_ASSERT(false);
}

LocalTaskModel::LocalTaskModel(QObject *parent)
   : BaseTaskModel(parent)
   , Task()
{
	root_ = pimpl();
}

int LocalTaskModel::rowCount(const QModelIndex &parent) const
{
	if (parent.column() > 0)
		return 0;

	ContainerBasePrivate *c = dynamic_cast<ContainerBasePrivate*>(node(parent));
	if (!c) return 0;

	return c->children().size();
}

QModelIndex LocalTaskModel::index(int row, int column, const QModelIndex &parent) const
{
	if (column < 0 || column >= columnCount())
		return QModelIndex();

	Q_ASSERT(dynamic_cast<ContainerBasePrivate*>(node(parent)));
	ContainerBasePrivate *p = static_cast<ContainerBasePrivate*>(node(parent));
	if (!p || row < 0 || (size_t)row >= p->children().size())
		return QModelIndex();

	auto it = p->children().begin();
	std::advance(it, row);
	return createIndex(row, column, it->get()->pimpl());
}

QModelIndex LocalTaskModel::parent(const QModelIndex &index) const
{
	if (index.model() != this)
		return QModelIndex();

	ContainerBasePrivate *parent = node(index)->parent()->pimpl();
	Q_ASSERT(parent);

	return this->index(parent);
}

Qt::ItemFlags LocalTaskModel::flags(const QModelIndex &index) const
{
	Qt::ItemFlags flags = BaseTaskModel::flags(index);
	ContainerBasePrivate *c = dynamic_cast<ContainerBasePrivate*>(node(index));
	if (c) flags |= Qt::ItemIsDropEnabled;
	return flags;
}

QVariant LocalTaskModel::data(const QModelIndex &index, int role) const
{
	Node *n = node(index);
	if (!n) return QVariant();

	switch (role) {
	case Qt::EditRole:
	case Qt::DisplayRole:
		switch (index.column()) {
		case 0: return QString::fromStdString(n->name());
		case 1: return	(uint)n->me()->numSolutions();
		case 2: return 0;
		}
		break;
	}
	return QVariant();
}

bool LocalTaskModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	Node *n = node(index);
	if (!n || index.column() != 0 || role != Qt::EditRole)
		return false;

	// change name
	const QString& name = value.toString();
	if (name == n->name().c_str())
		return false;
	n->me()->setName(name.toStdString());
	return true;
}

}
