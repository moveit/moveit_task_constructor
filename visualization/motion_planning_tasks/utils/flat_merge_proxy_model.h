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

#include <QAbstractItemModel>
#include <QStringList>

namespace moveit_rviz_plugin {
namespace utils {

class FlatMergeProxyModelPrivate;
/** The FlatMergeProxyModel merges several models into one.
 *
 *  All top-level items of the embedded models will show up as top-level items of this model.
 *  Removing top-level items will remove the whole embedded model if all top-level items from
 *  this model are to be removed. Otherwise, removal is forwarded to the embedded model.
 */
class FlatMergeProxyModel : public QAbstractItemModel
{
	Q_OBJECT
	Q_DECLARE_PRIVATE(FlatMergeProxyModel)
	FlatMergeProxyModelPrivate* d_ptr;

protected:
	/// method called when a model is removed
	virtual void onRemoveModel(QAbstractItemModel* model);

public:
	FlatMergeProxyModel(QObject* parent = nullptr);
	~FlatMergeProxyModel() override;

	/// number of embedded models
	size_t modelCount() const;
	/// return true if this index corresponds to a top-level item of an embedded model
	bool isToplevel(const QModelIndex& index) const;

	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;

	QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex& index) const override;

	QModelIndex mapToSource(const QModelIndex& proxy_index) const;
	QModelIndex mapFromSource(const QModelIndex& src_index) const;

	Qt::ItemFlags flags(const QModelIndex& index) const override;
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

	void setMimeTypes(const QStringList& mime_types);
	QStringList mimeTypes() const override;
	bool dropMimeData(const QMimeData* mime, Qt::DropAction action, int row, int column,
	                  const QModelIndex& parent) override;

	bool removeRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;

	/// insert a new sub model, pos is relative to modelCount()
	bool insertModel(QAbstractItemModel* model, int pos = -1);

	/// retrieve model corresponding to given index
	std::pair<QAbstractItemModel*, QModelIndex> getModel(const QModelIndex& index) const;

public Q_SLOTS:
	/// remove (first) matching model
	bool removeModel(QAbstractItemModel* model);
	/// remove model at given position (relative to modelCount())
	bool removeModel(int pos);

private:
	Q_PRIVATE_SLOT(d_func(), void _q_sourceDestroyed(QObject*))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeInserted(QModelIndex, int, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsInserted(QModelIndex, int, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeRemoved(QModelIndex, int, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsRemoved(QModelIndex, int, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeMoved(QModelIndex, int, int, QModelIndex, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsMoved(QModelIndex, int, int, QModelIndex, int))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceDataChanged(QModelIndex, QModelIndex, QVector<int>))
};
}  // namespace utils
}  // namespace moveit_rviz_plugin
