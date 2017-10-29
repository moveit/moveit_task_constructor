#pragma once

#include <QAbstractItemModel>
#include <QItemSelection>
#include <QStringList>
#include <QVector>

namespace moveit { namespace tools {

class CompositeProxyItemModelPrivate;

/** This model allows composition from various individual models:
 *  At any point in the tree structure of the model another sub model can
 *  be linked in ("mounted").
 */
class CompositeProxyItemModel : public QAbstractItemModel
{
	Q_OBJECT

public:
	CompositeProxyItemModel(QObject *parent = nullptr);
	CompositeProxyItemModel(QAbstractItemModel *root_model, QObject *parent);
	~CompositeProxyItemModel();

	void setHorizontalHeaderLabels(const QStringList &labels = QStringList());
	bool setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role) override;
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

	/// Attach a new sub model at given parent index. Previous children of parent will be hidden by the new model.
	void attachModel(QModelIndex parent, QAbstractItemModel *sub_model,
	                 const QModelIndex &src_root_index = QModelIndex(),
	                 const QVector<int> &column_mapping = QVector<int>());
	/// Detach model mounted at given parent index.
	void detachModel(QModelIndex parent);

	/// Checks whether a different sub model is used for children (might be nullptr as well)
	bool overridesSubModel(const QModelIndex &index) const;
	/// Checks whether a valid (non-nullptr) sub model is used for children
	bool mountsSubModel(const QModelIndex &index) const;
	/// return model used for children of this index
	QAbstractItemModel *subModel(const QModelIndex &index);
	const QAbstractItemModel *subModel(const QModelIndex &index) const { return const_cast<CompositeProxyItemModel*>(this)->subModel(index); }

	/// Maps the index of this model onto the index of the underlying source model
	QModelIndex mapToSource(const QModelIndex &index) const;
	/// Similar to mapToSource(), but returns the mount index, if index is a mounting point
	QModelIndex mapToSourceDeep(const QModelIndex &index) const;
	/// Maps an index of a source model onto corresponding indexes in this model
	/// As a source model might have been used multiple times, this mapping is 1-to-many.
	QModelIndexList mapFromSource(const QModelIndex &src_index, const QAbstractItemModel* src_model = nullptr) const;

	QItemSelection mapSelectionToSource(const QItemSelection &selection) const;
	QItemSelection mapSelectionFromSource(const QItemSelection &source_selection) const;

	bool hasChildren(const QModelIndex &index) const override;
	bool canFetchMore(const QModelIndex &index) const override;
	void fetchMore(const QModelIndex &index) override;

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;

	QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex &index) const override;

	Qt::ItemFlags flags(const QModelIndex & index) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	QMap<int, QVariant> itemData(const QModelIndex &index) const override;
	bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
	bool setItemData(const QModelIndex &index, const QMap<int, QVariant> &roles) override;

	QStringList mimeTypes() const override;
	QMimeData *mimeData(const QModelIndexList &indexes) const override;
	bool canDropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent) const override;
	bool dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent) override;

	bool insertRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
	bool insertColumns(int column, int count, const QModelIndex &parent = QModelIndex()) override;
	bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
	bool removeColumns(int column, int count, const QModelIndex &parent = QModelIndex()) override;

	QModelIndex buddy(const QModelIndex &index) const override;
	QModelIndexList match(const QModelIndex &start, int role,
	                      const QVariant &value, int hits = 1,
	                      Qt::MatchFlags flags = Qt::MatchFlags(Qt::MatchStartsWith|Qt::MatchWrap)) const override;

	void sort(int column, Qt::SortOrder order) override;
	QSize span(const QModelIndex &index) const override;

private:
	CompositeProxyItemModelPrivate *d_ptr;
	Q_DECLARE_PRIVATE(CompositeProxyItemModel)
	Q_DISABLE_COPY(CompositeProxyItemModel)

	Q_PRIVATE_SLOT(d_func(), void _q_sourceModelDestroyed())
	Q_PRIVATE_SLOT(d_func(), void _q_sourceDataChanged(const QModelIndex &source_top_left, const QModelIndex &source_bottom_right, const QVector<int> &roles))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceLayoutAboutToBeChanged(const QList<QPersistentModelIndex> &sourceParents = QList<QPersistentModelIndex>(),
	                                                              QAbstractItemModel::LayoutChangeHint hint = QAbstractItemModel::NoLayoutChangeHint))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceLayoutChanged(const QList<QPersistentModelIndex> &sourceParents = QList<QPersistentModelIndex>(),
	                                                     QAbstractItemModel::LayoutChangeHint hint = QAbstractItemModel::NoLayoutChangeHint))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceHeaderDataChanged(Qt::Orientation orientation, int start, int end))

	Q_PRIVATE_SLOT(d_func(), void _q_sourceAboutToBeReset())
	Q_PRIVATE_SLOT(d_func(), void _q_sourceReset())

	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsInserted(const QModelIndex &source_parent, int start, int end))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeRemoved(const QModelIndex &source_parent, int start, int end))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsRemoved(const QModelIndex &source_parent, int start, int end))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int))

	Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsInserted(const QModelIndex &source_parent, int start, int end))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsRemoved(const QModelIndex &source_parent, int start, int end))
	Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsMoved(QModelIndex,int,int,QModelIndex,int))
};

} }
