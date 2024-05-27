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

#include "flat_merge_proxy_model.h"
#include <vector>
#include <numeric>

namespace moveit_rviz_plugin {
namespace utils {

class FlatMergeProxyModelPrivate
{
public:
	Q_DECLARE_PUBLIC(FlatMergeProxyModel)
	FlatMergeProxyModel* q_ptr;
	QStringList mime_types_;

	struct ModelData
	{
		ModelData(QAbstractItemModel* m) : model_(m) {}

		QAbstractItemModel* model_;
		// map of proxy=source QModelIndex's internal pointer to source parent's QModelIndex
		using ProxyToSourceMap = std::map<void*, QPersistentModelIndex>;
		ProxyToSourceMap proxy_to_source_mapping_;
		std::vector<ProxyToSourceMap::iterator> invalidated_mappings_;

		inline void storeMapping(void* src_internal_pointer, const QModelIndex& src_parent) {
			ProxyToSourceMap::value_type pair(src_internal_pointer, src_parent);
			auto it = proxy_to_source_mapping_.insert(std::move(pair)).first;
			Q_ASSERT_X(it->second == src_parent, "FlatMergeProxyModel",
			           "the internal pointer must map to a unique src_parent");
			Q_UNUSED(it)
		}

		// collect all invalidated mappings, return true if we are affected at all
		bool rowsAboutToBeRemoved(const QModelIndex& src_parent, int start, int end) {
			std::vector<void*> pointers;
			pointers.reserve(end - start);
			for (int row = start; row != end; ++row)
				pointers.emplace_back(model_->index(row, 0, src_parent).internalPointer());

			Q_ASSERT(invalidated_mappings_.empty());
			for (auto it = proxy_to_source_mapping_.begin(); it != proxy_to_source_mapping_.end(); ++it) {
				QModelIndex current = it->second;
				if (src_parent == current) {  // it is on affected level
					if (std::find(pointers.begin(), pointers.end(), it->first) != pointers.end())
						invalidated_mappings_.push_back(it);
				} else {  // not on affected level, check parents
					while (current.isValid()) {
						QModelIndex current_parent = current.parent();
						if (current_parent == src_parent) {  // current on affected level
							if (current.row() >= start && current.row() < end)
								invalidated_mappings_.push_back(it);
							break;
						}
						current = current_parent;
					}
				}
			}
			return !invalidated_mappings_.empty();
		}
		bool rowsRemoved() {
			bool affected = !invalidated_mappings_.empty();
			// remove invalidated mappings
			for (auto it : invalidated_mappings_)
				proxy_to_source_mapping_.erase(it);
			invalidated_mappings_.clear();
			return affected;
		}
	};

	// top-level items
	std::vector<ModelData> data_;

public:
	FlatMergeProxyModelPrivate(FlatMergeProxyModel* model) : q_ptr(model) {}

	std::vector<ModelData>::iterator find(const QObject* model) {
		Q_ASSERT(model);
		return std::find_if(data_.begin(), data_.end(), [model](const auto& data) { return data.model_ == model; });
	}

	int accumulatedRowCount(std::vector<ModelData>::const_iterator start,
	                        std::vector<ModelData>::const_iterator end) const {
		return std::accumulate(start, end, 0, [](int acc, const auto& d) { return acc + d.model_->rowCount(); });
	}
	int rowOffset(QObject* model) {
		int result = 0;
		for (const ModelData& d : data_) {
			if (d.model_ == model)
				return result;
			result += d.model_->rowCount();
		}
		Q_ASSERT(false);
		return 0;
	}

	// retrieve the source_index corresponding to proxy_index
	QModelIndex mapToSource(const QModelIndex& proxy_index, ModelData*& data) const {
		Q_ASSERT(proxy_index.isValid());
		Q_ASSERT(proxy_index.model() == q_ptr);

		int prev_rows = 0;  // accumulated rows from previous models
		for (const ModelData& d : data_) {
			// internal_pointer points to source parent
			auto it = d.proxy_to_source_mapping_.find(proxy_index.internalPointer());
			if (it != d.proxy_to_source_mapping_.end()) {
				data = const_cast<ModelData*>(&d);
				const QModelIndex& src_index = it->second;
				int row = proxy_index.row();

				if (!src_index.isValid())  // top-level item of embedded model
					row -= prev_rows;  // need to reduce row by number of previous' models rows

				return d.model_->index(row, proxy_index.column(), src_index);
			}
			prev_rows += d.model_->rowCount();
		}
		Q_ASSERT(false);
		return QModelIndex();
	}

	QModelIndex mapFromSource(const QModelIndex& src, ModelData* data = nullptr) const {
		if (!src.isValid())
			return QModelIndex();

		QModelIndex src_parent = src.parent();
		int prev_rows = 0;
		if (!src_parent.isValid()) {  // src is top-level item
			auto it = data_.begin(), end = data_.end();
			for (; it != end; ++it) {
				if (it->model_ == src.model()) {
					data = &const_cast<ModelData&>(*it);
					break;
				}
				prev_rows += it->model_->rowCount();
			}
			Q_ASSERT(it != end);
		}

		// store source index in mapping: easy, if we already know the correspondig model (coming top-down)
		if (data)
			data->storeMapping(src.internalPointer(), src_parent);
		// coming bottom-up, we need to climb the tree until we reach root and can lookup the model
		else
			mapSourceIndexes(src, data);

		// use internal pointer from src index, top-level items need prev_rows to be added to row index
		return q_ptr->createIndex(src.row() + prev_rows, src.column(), src.internalPointer());
	}

	void mapSourceIndexes(const QModelIndex& src, ModelData*& data) const {
		Q_ASSERT(src.isValid());

		const QModelIndex& src_parent = src.parent();
		if (!src_parent.isValid()) {  // reached root
			// figure out corresponding ModelData from src.model()
			for (const ModelData& d : data_) {
				if (d.model_ == src.model()) {
					data = const_cast<ModelData*>(&d);
					data->storeMapping(src.internalPointer(), src_parent);
					return;
				}
			}
			Q_ASSERT(false);  // src should be part of our model!
		}

		// recursively climb the tree
		mapSourceIndexes(src_parent, data);
		// now data should be well-defined
		data->storeMapping(src.internalPointer(), src_parent);
	}

	// remove model referenced by it, call indicates that onRemoveModel() should be called
	bool removeModel(std::vector<ModelData>::iterator it, bool call);

private:
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceDestroyed(QObject* model);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsAboutToBeInserted(const QModelIndex& parent, int start, int end);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsInserted(const QModelIndex& parent, int start, int end);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsRemoved(const QModelIndex& parent, int start, int end);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsAboutToBeMoved(const QModelIndex& sourceParent, int sourceStart, int sourceEnd,
	                                 const QModelIndex& destParent, int dest);
	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceRowsMoved(const QModelIndex& sourceParent, int sourceStart, int sourceEnd,
	                        const QModelIndex& destParent, int dest);

	// NOLINTNEXTLINE(readability-identifier-naming)
	void _q_sourceDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight, const QVector<int>& roles);
};

FlatMergeProxyModel::FlatMergeProxyModel(QObject* parent) : QAbstractItemModel(parent) {
	d_ptr = new FlatMergeProxyModelPrivate(this);
}

FlatMergeProxyModel::~FlatMergeProxyModel() {
	delete d_ptr;
}

size_t FlatMergeProxyModel::modelCount() const {
	return d_ptr->data_.size();
}

bool FlatMergeProxyModel::isToplevel(const QModelIndex& index) const {
	return index.isValid() && !index.parent().isValid();
}

int FlatMergeProxyModel::rowCount(const QModelIndex& parent) const {
	if (parent.column() > 0)
		return 0;

	if (!parent.isValid())  // root
		return d_ptr->accumulatedRowCount(d_ptr->data_.begin(), d_ptr->data_.end());

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_parent = d_ptr->mapToSource(parent, data);
	return data->model_->rowCount(src_parent);
}

int FlatMergeProxyModel::columnCount(const QModelIndex& parent) const {
	if (parent.column() > 0 || d_ptr->data_.empty())
		return 0;

	if (!parent.isValid())  // root
		return d_ptr->data_[0].model_->columnCount(parent);

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_parent = d_ptr->mapToSource(parent, data);
	return data->model_->columnCount(src_parent);
}

QModelIndex FlatMergeProxyModel::index(int row, int column, const QModelIndex& parent) const {
	if (row < 0 || column < 0 || column >= columnCount(parent))
		return QModelIndex();

	if (!parent.isValid()) {  // top-level items
		int prev_rows = 0;
		for (FlatMergeProxyModelPrivate::ModelData& d : const_cast<FlatMergeProxyModelPrivate*>(d_ptr)->data_) {
			int rows = d.model_->rowCount();
			if (row - prev_rows < rows) {
				const QModelIndex& src_index = d.model_->index(row - prev_rows, column, QModelIndex());
				// for top-level item, internal pointer refers to model
				d.storeMapping(src_index.internalPointer(), QModelIndex());
				return createIndex(row, column, src_index.internalPointer());
			}
			prev_rows += rows;
		}
		return QModelIndex();  // row is too large
	}

	// other items need to refer to operation on source model
	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_parent = d_ptr->mapToSource(parent, data);
	return d_ptr->mapFromSource(data->model_->index(row, column, src_parent), data);
}

QModelIndex FlatMergeProxyModel::parent(const QModelIndex& child) const {
	if (!child.isValid())
		return QModelIndex();

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_parent = d_ptr->mapToSource(child, data).parent();
	return d_ptr->mapFromSource(src_parent, data);
}

QModelIndex FlatMergeProxyModel::mapToSource(const QModelIndex& proxy_index) const {
	FlatMergeProxyModelPrivate::ModelData* data;
	return d_ptr->mapToSource(proxy_index, data);
}

QModelIndex FlatMergeProxyModel::mapFromSource(const QModelIndex& src_index) const {
	return d_ptr->mapFromSource(src_index, nullptr);
}

Qt::ItemFlags FlatMergeProxyModel::flags(const QModelIndex& index) const {
	if (!index.isValid())
		return QAbstractItemModel::flags(index);

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_index = d_ptr->mapToSource(index, data);
	return data->model_->flags(src_index);
}

QVariant FlatMergeProxyModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Horizontal && !d_ptr->data_.empty())
		// return headerData of first sub model, assuming they are all the same
		return d_ptr->data_[0].model_->headerData(section, orientation, role);

	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant FlatMergeProxyModel::data(const QModelIndex& index, int role) const {
	Q_ASSERT(index.isValid());

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_index = d_ptr->mapToSource(index, data);
	return data->model_->data(src_index, role);
}

bool FlatMergeProxyModel::setData(const QModelIndex& index, const QVariant& value, int role) {
	Q_ASSERT(index.isValid());

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_index = d_ptr->mapToSource(index, data);
	Q_ASSERT(data->model_ == src_index.model());
	return data->model_->setData(src_index, value, role);
}

void FlatMergeProxyModel::setMimeTypes(const QStringList& mime_types) {
	d_ptr->mime_types_ = mime_types;
}

QStringList FlatMergeProxyModel::mimeTypes() const {
	return d_ptr->mime_types_;
}

bool FlatMergeProxyModel::dropMimeData(const QMimeData* mime, Qt::DropAction action, int row, int column,
                                       const QModelIndex& parent) {
	if (!parent.isValid())
		return false;

	// propagate to corresponding child model
	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	QModelIndex src_parent = d_ptr->mapToSource(parent, data);
	return data->model_->dropMimeData(mime, action, row, column, src_parent);
}

bool FlatMergeProxyModel::removeRows(int row, int count, const QModelIndex& parent) {
	if (!parent.isValid()) {  // top-level items: remove embedded models
		if (row < 0 || row + count > rowCount())
			return false;

		bool some_success = false;
		for (auto it = d_ptr->data_.begin(), end = d_ptr->data_.end(); it != end;) {
			if (count <= 0)
				break;

			int rows = it->model_->rowCount();
			if (row == 0 && count >= rows) {
				// all rows affected: remove whole model
				some_success |= d_ptr->removeModel(it, true);
				count -= rows;
				continue;
			} else if (row < rows) {
				// some rows affected: forward to model
				int remove_count = std::min(count, rows - row);
				some_success |= it->model_->removeRows(row, remove_count);
				count -= remove_count;  // reduce count of to-be-removed rows
				row = 0;  // next sub model starts with row 0 again
			} else {
				// it->model_ not affected
				row -= rows;
			}
			++it;
		}
		Q_ASSERT(count == 0);
		return some_success;
	} else {
		FlatMergeProxyModelPrivate::ModelData* data = nullptr;
		QModelIndex src_parent = d_ptr->mapToSource(parent, data);
		return data->model_->removeRows(row, count, src_parent);
	}
}

bool FlatMergeProxyModel::insertModel(QAbstractItemModel* model, int pos) {
	if (!model || model == this || model->rowCount() == 0)
		return false;  // invalid model
	if (!d_ptr->data_.empty() && model->columnCount() != columnCount())
		return false;  // all models must have same column count

	// limit pos to range [0, modelCount()]
	if (pos > 0 && pos > static_cast<int>(modelCount()))
		pos = modelCount();
	if (pos < 0)
		pos = modelCount() + std::max<int>(pos + 1, -modelCount());
	Q_ASSERT(pos >= 0 && pos <= static_cast<int>(modelCount()));
	auto it = d_ptr->data_.begin();
	std::advance(it, pos);

	int row = 0;
	for (const auto& d : d_ptr->data_) {
		if (d.model_ == model)
			return false;  // model can only inserted once
		// accumulate rowCount() of models before pos
		if (--pos >= 0)
			row += d.model_->rowCount();
	}

	beginInsertRows(QModelIndex(), row, row + model->rowCount() - 1);
	d_ptr->data_.insert(it, FlatMergeProxyModelPrivate::ModelData(model));
	endInsertRows();

	connect(model, SIGNAL(destroyed(QObject*)), this, SLOT(_q_sourceDestroyed(QObject*)));
	connect(model, SIGNAL(rowsAboutToBeInserted(QModelIndex, int, int)), this,
	        SLOT(_q_sourceRowsAboutToBeInserted(QModelIndex, int, int)));
	connect(model, SIGNAL(rowsInserted(QModelIndex, int, int)), this,
	        SLOT(_q_sourceRowsInserted(QModelIndex, int, int)));
	connect(model, SIGNAL(rowsAboutToBeRemoved(QModelIndex, int, int)), this,
	        SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex, int, int)));
	connect(model, SIGNAL(rowsRemoved(QModelIndex, int, int)), this, SLOT(_q_sourceRowsRemoved(QModelIndex, int, int)));
	connect(model, SIGNAL(rowsAboutToBeMoved(QModelIndex, int, int, QModelIndex, int)), this,
	        SLOT(_q_sourceRowsAboutToBeMoved(QModelIndex, int, int, QModelIndex, int)));
	connect(model, SIGNAL(rowsMoved(QModelIndex, int, int, QModelIndex, int)), this,
	        SLOT(_q_sourceRowsMoved(QModelIndex, int, int, QModelIndex, int)));
	connect(model, SIGNAL(dataChanged(QModelIndex, QModelIndex, QVector<int>)), this,
	        SLOT(_q_sourceDataChanged(QModelIndex, QModelIndex, QVector<int>)));

	return true;
}

std::pair<QAbstractItemModel*, QModelIndex> FlatMergeProxyModel::getModel(const QModelIndex& index) const {
	if (!index.isValid())
		return std::make_pair(nullptr, QModelIndex());

	FlatMergeProxyModelPrivate::ModelData* data = nullptr;
	const QModelIndex& src_index = d_ptr->mapToSource(index, data);

	Q_ASSERT(data);
	// NOLINTNEXTLINE(clang-analyzer-core.NonNullParamChecker)
	return std::make_pair(data->model_, src_index);
}

bool FlatMergeProxyModel::removeModel(QAbstractItemModel* model) {
	return d_ptr->removeModel(d_ptr->find(model), true);
}

bool FlatMergeProxyModel::removeModel(int pos) {
	// limit pos to range [0, modelCount())
	if (pos < 0)
		pos = modelCount() + pos + 1;
	if (pos < 0)
		return false;
	if (pos >= static_cast<int>(modelCount()))
		return false;
	Q_ASSERT(pos >= 0 && pos < static_cast<int>(modelCount()));

	auto it = d_ptr->data_.begin();
	std::advance(it, pos);
	return d_ptr->removeModel(it, true);
}

void FlatMergeProxyModel::onRemoveModel(QAbstractItemModel* model) {
	disconnect(model, SIGNAL(destroyed(QObject*)), this, SLOT(_q_sourceDestroyed(QObject*)));
	disconnect(model, SIGNAL(rowsAboutToBeInserted(QModelIndex, int, int)), this,
	           SLOT(_q_sourceRowsAboutToBeInserted(QModelIndex, int, int)));
	disconnect(model, SIGNAL(rowsInserted(QModelIndex, int, int)), this,
	           SLOT(_q_sourceRowsInserted(QModelIndex, int, int)));
	disconnect(model, SIGNAL(rowsAboutToBeRemoved(QModelIndex, int, int)), this,
	           SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex, int, int)));
	disconnect(model, SIGNAL(rowsRemoved(QModelIndex, int, int)), this,
	           SLOT(_q_sourceRowsRemoved(QModelIndex, int, int)));
	disconnect(model, SIGNAL(rowsAboutToBeMoved(QModelIndex, int, int, QModelIndex, int)), this,
	           SLOT(_q_sourceRowsAboutToBeMoved(QModelIndex, int, int, QModelIndex, int)));
	disconnect(model, SIGNAL(rowsMoved(QModelIndex, int, int, QModelIndex, int)), this,
	           SLOT(_q_sourceRowsMoved(QModelIndex, int, int, QModelIndex, int)));
	disconnect(model, SIGNAL(dataChanged(QModelIndex, QModelIndex, QVector<int>)), this,
	           SLOT(_q_sourceDataChanged(QModelIndex, QModelIndex, QVector<int>)));
}

bool FlatMergeProxyModelPrivate::removeModel(std::vector<ModelData>::iterator it, bool call) {
	if (it == data_.end())
		return false;

	int row = accumulatedRowCount(data_.begin(), it);
	q_ptr->beginRemoveRows(QModelIndex(), row, row + it->model_->rowCount() - 1);
	if (call)
		q_ptr->onRemoveModel(it->model_);
	it = data_.erase(it);
	q_ptr->endRemoveRows();
	return true;
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceDestroyed(QObject* model) {
	removeModel(find(model), false);
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsAboutToBeInserted(const QModelIndex& parent, int start, int end) {
	int offset = parent.isValid() ? 0 : rowOffset(q_ptr->sender());
	q_ptr->beginInsertRows(mapFromSource(parent), start + offset, end + offset);
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsAboutToBeMoved(const QModelIndex& sourceParent, int sourceStart,
                                                             int sourceEnd, const QModelIndex& destParent, int dest) {
	int source_offset = sourceParent.isValid() ? 0 : rowOffset(q_ptr->sender());
	int dest_offset = destParent.isValid() ? 0 : rowOffset(q_ptr->sender());
	q_ptr->beginMoveRows(mapFromSource(sourceParent), sourceStart + source_offset, sourceEnd + source_offset,
	                     mapFromSource(destParent), dest + dest_offset);
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end) {
	auto it = find(q_ptr->sender());
	Q_ASSERT(it != data_.end());
	if (it->rowsAboutToBeRemoved(parent, start, end + 1)) {
		int offset = parent.isValid() ? 0 : rowOffset(q_ptr->sender());
		q_ptr->beginRemoveRows(mapFromSource(parent), start + offset, end + offset);
	}
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsInserted(const QModelIndex& parent, int start, int end) {
	Q_UNUSED(parent)
	Q_UNUSED(start)
	Q_UNUSED(end)
	q_ptr->endInsertRows();
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsMoved(const QModelIndex& sourceParent, int sourceStart, int sourceEnd,
                                                    const QModelIndex& destParent, int dest) {
	Q_UNUSED(sourceParent)
	Q_UNUSED(sourceStart)
	Q_UNUSED(sourceEnd)
	Q_UNUSED(destParent)
	Q_UNUSED(dest)
	q_ptr->endMoveRows();
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceRowsRemoved(const QModelIndex& parent, int start, int end) {
	Q_UNUSED(parent)
	Q_UNUSED(start)
	Q_UNUSED(end)

	auto it = find(q_ptr->sender());
	Q_ASSERT(it != data_.end());
	if (it->rowsRemoved())
		q_ptr->endRemoveRows();
}

// NOLINTNEXTLINE(readability-identifier-naming)
void FlatMergeProxyModelPrivate::_q_sourceDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
                                                      const QVector<int>& roles) {
	q_ptr->dataChanged(mapFromSource(topLeft), mapFromSource(bottomRight), roles);
}
}  // namespace utils
}  // namespace moveit_rviz_plugin

#include "moc_flat_merge_proxy_model.cpp"
