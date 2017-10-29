#include "composite_proxy_model.h"

#include <assert.h>
#include <QDebug>
#include <QSize>

#define NOT_IMPLEMENTED {throw std::runtime_error(std::string(__FUNCTION__) + " not yet implemented");}

namespace moveit { namespace tools {

/// An empty model that is used to prune a parent model
class EmptyItemModel : public QAbstractItemModel
{
public:
    explicit EmptyItemModel(QObject *parent = 0) : QAbstractItemModel(parent) {}
    QModelIndex index(int, int, const QModelIndex &) const override { return QModelIndex(); }
    QModelIndex parent(const QModelIndex &) const override { return QModelIndex(); }
    int rowCount(const QModelIndex &) const override { return 0; }
    int columnCount(const QModelIndex &) const override { return 0; }
    bool hasChildren(const QModelIndex &) const override { return false; }
    QVariant data(const QModelIndex &, int) const override { return QVariant(); }
};
Q_GLOBAL_STATIC(EmptyItemModel, qEmptyModel)


/*  QModelIndexes of a model need to refer to their model. Hence we cannot directly
 *  use the existing QModelIndexes of the sub models, but rather need to shadow
 *  the tree structure of the composite model with an own data structure.
 *  To this end we use a tree structure composed from Node*.
 *  The map \a mounts stores all mounted sub models together with their mount points.
 *  This is required to map back from source indexes to proxy indexes.
 */
class CompositeProxyItemModelPrivate {
	Q_DECLARE_PUBLIC(CompositeProxyItemModel)

	/// The tree structure of Nodes represents the (lazily mapped) structure of the composite model.
	/// 1) It mimics and unifies the tree structure of the underlying data models,
	///    particularly providing the important back link (parent) from one model index to its parent.
	/// 2) It implicitly stores the "mount" points of sub models:
	///    If a new sub model is mounted, then src_index.model() != sub_model. In that case
	///    sub_model is to be used for children of src_index.
	///    Otherwise src_index.model() == sub_model and we stay within the same model.
	///
	/// Our ModelIndexes store a pointer to this data structure in their internalPointer(), i.e.
	/// the internalPointer() of a ModelIndex points to the Node* representing this item.
	/// This is in contrast to many other approaches that store a pointer
	struct Node;

	/// MountMap maps mounted models onto mounting information
	struct MountInfo {
		QAbstractItemModel* model;
		QPersistentModelIndex root_index; // root source index to retrieve data from
		Node* mount_node; // node in our hierarchy that actually mounts the model
		QVector<int> column_mapping; // map from visible columns to model columns

		explicit MountInfo(QAbstractItemModel* const model, const QModelIndex &index, Node* mount_point)
		    : model(model), root_index(index), mount_node(mount_point) {}
	};
	/// It is a multi hash, because it's possible to mount the same model at different locations
	typedef QMultiHash<const QAbstractItemModel*, MountInfo> MountMap;

	struct Node {
		Node* const parent;
		QVector<Node*> children; // already fetched items of sub_model
		mutable uint available_src_rows; // number of (currently) available rows from source model, equals sub_model->rowCount()

		QModelIndex src_index;   // index of this node in the source model
		MountMap::iterator mount; // model's mount point


		// root constructor
		Node() : parent(nullptr), available_src_rows(0), src_index(QModelIndex()) {}
		// normal constructor
		Node(Node* parent, const QModelIndex& src_index)
		    : parent(parent), available_src_rows(0), src_index(src_index)
		{
			assert(parent != nullptr);
			assert(src_index.column() == 0);
			mount = parent->mount;
		}

		~Node()
		{
			clearChildren();
		}

		void clearChildren() {
			qDeleteAll(children);
			children.clear();
			available_src_rows = 0;
		}

		void fetchMore(const QModelIndex& src_parent, uint new_size) {
			children.reserve(new_size);
			for (uint row = children.size(); row != new_size; ++row)
				children.push_back(new Node(this, model()->index(row, 0, src_parent)));
		}

		inline bool overridesSubModel() const {
			return mount->model != src_index.model();
		}

		int mapColumn(int proxy_column) const {
			return mount->column_mapping.isEmpty() ? proxy_column : mount->column_mapping.at(proxy_column);
		}

		int columnCount() const {
			int mapping_columns = mount->column_mapping.size();
			int model_columns = model()->columnCount(queryIndex());
			return mapping_columns == 0 ? model_columns : mapping_columns;
		}

		inline int rowCount() const {
			return children.size();
		}

		inline QAbstractItemModel* model() const {
			return mount->model;
		}

		inline const QModelIndex& queryIndex() const {
			if (overridesSubModel()) // if mounts a sub model, use root index stored in this mount
				return mount->root_index;
			else
				return src_index;
		}

		// retrieve source index for given row and column
		inline QModelIndex sourceIndex(int row, int column) const {
			assert(row >= 0 && column >= 0);
			if ((uint)row >= available_src_rows || column >= columnCount())
				return QModelIndex(); // row/column index out-of-range

			int mapped_column = mapColumn(column);
			if (mapped_column < 0)
				return QModelIndex();

			return model()->index(row, mapped_column, queryIndex());
		}
	};

	CompositeProxyItemModel* q_ptr;
	Node root;
	MountMap mounts;
	QVector<QString> header_labels;


	CompositeProxyItemModelPrivate(CompositeProxyItemModel* q_ptr) : q_ptr(q_ptr) {
		// mount qEmptyModel() at root
		createMountAt(&root, qEmptyModel());
	}
	~CompositeProxyItemModelPrivate() {}

	void createMountAt(Node* node, QAbstractItemModel *sub_model,
	                   const QModelIndex &src_root_index = QModelIndex(),
	                   const QVector<int> column_mapping = QVector<int>())
	{
		MountInfo mi(sub_model, src_root_index, node);

		int columns = sub_model->columnCount(src_root_index);
		mi.column_mapping.reserve(column_mapping.size());
		for (int col : column_mapping) {
			if (col >= columns) {
				qWarning() << "invalid column" << col << "in model of" << columns << "columns";
				col = -1;
			}
			mi.column_mapping << col;
		}
		node->mount = mounts.insert(mi.model, mi);
	}

	// extract Node* from model index' internalPointer()
	inline Node* node(const QModelIndex &proxy_index)
	{
		if (!proxy_index.isValid()) return &root;
		assert(proxy_index.model() == q_ptr);
		Node* n = static_cast<Node*>(proxy_index.internalPointer());
		assert(n);
		return n;
	}
	const Node* node(const QModelIndex &index) const {
		return const_cast<CompositeProxyItemModelPrivate*>(this)->node(index);
	}

	// retrieve proxy index corresponding to the given node
	QModelIndex proxyIndex(const Node* node) const {
		if (node == &root) return QModelIndex();
		assert(node->parent->children.contains(const_cast<Node*>(node)));
		return q_ptr->createIndex(node->parent->children.indexOf(const_cast<Node*>(node)), 0, const_cast<Node*>(node));
	}

	// return if further rows (more than rowCount()) can be fetched
	bool canFetchMore(const Node* node) const {
		node->available_src_rows = node->model()->rowCount(node->queryIndex());
		return node->available_src_rows > (uint)node->rowCount() // not yet shadowed all source indexes
		        || node->model()->canFetchMore(node->queryIndex()); // source model might have more items too
	}
	// try to fetch next MAX_FETCH_ROWS rows
	void fetchMore(Node* node) const {
		static const uint MAX_FETCH_ROWS = 100;
		QModelIndex query_index = node->queryIndex();
		node->available_src_rows = node->model()->rowCount(query_index);
		if (node->available_src_rows > (uint)node->rowCount() + MAX_FETCH_ROWS); // no need to fetch more from source model
		else if (node->model()->canFetchMore(query_index)) {
			// fetch more rows from source
			node->model()->fetchMore(query_index);
			// update number of available rows
			node->available_src_rows = node->model()->rowCount(query_index);
		}
		uint new_size = node->rowCount() + qMin(MAX_FETCH_ROWS, node->available_src_rows - node->rowCount());
		if (new_size > (uint)node->rowCount()) {
			q_ptr->beginInsertRows(proxyIndex(node), node->rowCount(), new_size-1);
			node->fetchMore(query_index, new_size);
			q_ptr->endInsertRows();
		}
	}

	// return number of already fetched rows
	int rowCount(const QModelIndex &proxy_index) const {
		return node(proxy_index)->rowCount();
	}

	int columnCount(const QModelIndex &proxy_index) const {
		const Node *node = this->node(proxy_index);
		// column count is defined by num of header labels or by root model
		int root_column_count = header_labels.size();
		if (root_column_count == 0)
			root_column_count = root.columnCount();
		if (node == &root) return root_column_count;

		// for non-root elements, the minimum of root_column_count and actual column count is returned
		return qMin(root_column_count, node->columnCount());
	}

	// recursively climp up from src_index to src_root_index (or NIL)
	QModelIndex mapFromSource(const QModelIndex& src_index,
	                          const QModelIndex& src_root_index,
	                          const QModelIndex& proxy_mount_index,
	                          const QVector<int>& column_mapping) {
		// if src_index == src_root_index, not considering column()
		if (src_index.model() == src_root_index.model() &&
			src_index.row() == src_root_index.row() &&
			src_index.internalId() == src_root_index.internalId())
			return proxy_mount_index;
		assert(src_index.isValid());

		int mapped_column = column_mapping.isEmpty() ? src_index.column() : column_mapping.indexOf(src_index.column());
		if (mapped_column < 0)
			return QModelIndex();

		return q_ptr->index(src_index.row(), mapped_column,
		                    mapFromSource(src_index.parent(), src_root_index, proxy_mount_index, column_mapping));
	}

	void connect_signals(QAbstractItemModel* sub_model) {
		q_ptr->connect(sub_model, SIGNAL(destroyed(QObject*)), q_ptr, SLOT(_q_sourceModelDestroyed()));

		q_ptr->connect(sub_model, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
		               q_ptr, SLOT(_q_sourceDataChanged(QModelIndex,QModelIndex,QVector<int>)));
		q_ptr->connect(sub_model, SIGNAL(headerDataChanged(Qt::Orientation,int,int)),
		               q_ptr, SLOT(_q_sourceHeaderDataChanged(Qt::Orientation,int,int)));

		q_ptr->connect(sub_model, SIGNAL(rowsInserted(QModelIndex,int,int)),
		               q_ptr, SLOT(_q_sourceRowsInserted(QModelIndex,int,int)));
		q_ptr->connect(sub_model, SIGNAL(columnsInserted(QModelIndex,int,int)),
		               q_ptr, SLOT(_q_sourceColumnsInserted(QModelIndex,int,int)));

		q_ptr->connect(sub_model, SIGNAL(rowsAboutToBeRemoved(QModelIndex,int,int)),
		               q_ptr, SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex,int,int)));
		q_ptr->connect(sub_model, SIGNAL(rowsRemoved(QModelIndex,int,int)),
		               q_ptr, SLOT(_q_sourceRowsRemoved(QModelIndex,int,int)));
		q_ptr->connect(sub_model, SIGNAL(columnsRemoved(QModelIndex,int,int)),
		               q_ptr, SLOT(_q_sourceColumnsRemoved(QModelIndex,int,int)));

		q_ptr->connect(sub_model, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)),
		               q_ptr, SLOT(_q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int)));
		q_ptr->connect(sub_model, SIGNAL(columnsMoved(QModelIndex,int,int,QModelIndex,int)),
		               q_ptr, SLOT(_q_sourceColumnsMoved(QModelIndex,int,int,QModelIndex,int)));
		q_ptr->connect(sub_model, SIGNAL(layoutAboutToBeChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)),
		               q_ptr, SLOT(_q_sourceLayoutAboutToBeChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)));
		q_ptr->connect(sub_model, SIGNAL(layoutChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)),
		               q_ptr, SLOT(_q_sourceLayoutChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)));
		q_ptr->connect(sub_model, SIGNAL(modelAboutToBeReset()), q_ptr, SLOT(_q_sourceAboutToBeReset()));
		q_ptr->connect(sub_model, SIGNAL(modelReset()), q_ptr, SLOT(_q_sourceReset()));
	}

	void disconnect_signals(QAbstractItemModel* sub_model) {
		// disconnect signals only if all instances of this sub model are gone
		if (mounts.contains(sub_model)) return;

		q_ptr->disconnect(sub_model, SIGNAL(destroyed()), q_ptr, SLOT(_q_sourceModelDestroyed()));

		q_ptr->disconnect(sub_model, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
		                  q_ptr, SLOT(_q_sourceDataChanged(QModelIndex,QModelIndex,QVector<int>)));
		q_ptr->disconnect(sub_model, SIGNAL(headerDataChanged(Qt::Orientation,int,int)),
		                  q_ptr, SLOT(_q_sourceHeaderDataChanged(Qt::Orientation,int,int)));

		q_ptr->disconnect(sub_model, SIGNAL(rowsInserted(QModelIndex,int,int)),
		                  q_ptr, SLOT(_q_sourceRowsInserted(QModelIndex,int,int)));
		q_ptr->disconnect(sub_model, SIGNAL(columnsInserted(QModelIndex,int,int)),
		                  q_ptr, SLOT(_q_sourceColumnsInserted(QModelIndex,int,int)));

		q_ptr->disconnect(sub_model, SIGNAL(rowsAboutToBeRemoved(QModelIndex,int,int)),
		                  q_ptr, SLOT(_q_sourceRowsAboutToBeRemoved(QModelIndex,int,int)));
		q_ptr->disconnect(sub_model, SIGNAL(rowsRemoved(QModelIndex,int,int)),
		                  q_ptr, SLOT(_q_sourceRowsRemoved(QModelIndex,int,int)));
		q_ptr->disconnect(sub_model, SIGNAL(columnsRemoved(QModelIndex,int,int)),
		                  q_ptr, SLOT(_q_sourceColumnsRemoved(QModelIndex,int,int)));

		q_ptr->disconnect(sub_model, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)),
		                  q_ptr, SLOT(_q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int)));
		q_ptr->disconnect(sub_model, SIGNAL(columnsMoved(QModelIndex,int,int,QModelIndex,int)),
		                  q_ptr, SLOT(_q_sourceColumnsMoved(QModelIndex,int,int,QModelIndex,int)));

		q_ptr->disconnect(sub_model, SIGNAL(layoutAboutToBeChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)),
		                  q_ptr, SLOT(_q_sourceLayoutAboutToBeChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)));
		q_ptr->disconnect(sub_model, SIGNAL(layoutChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)),
		                  q_ptr, SLOT(_q_sourceLayoutChanged(QList<QPersistentModelIndex>,QAbstractItemModel::LayoutChangeHint)));

		q_ptr->disconnect(sub_model, SIGNAL(modelAboutToBeReset()), q_ptr, SLOT(_q_sourceAboutToBeReset()));
		q_ptr->disconnect(sub_model, SIGNAL(modelReset()), q_ptr, SLOT(_q_sourceReset()));
	}

	// remove a mount at node if it exists
	void removeExistingMountAt(Node *node, const QAbstractItemModel *suppress_disconnect=nullptr) {
		assert(node->rowCount() == 0); // before all children have to be removed!
		if (node->overridesSubModel()) {
			MountMap::iterator &mount = node->mount;
			assert(mount->mount_node == node);

			QAbstractItemModel *model = mount->model;
			mounts.erase(mount); // remove entry from MountMap
			if (model != suppress_disconnect)
				disconnect_signals(model); // *before* removing signals
		}
	}

	// recursively remove children of node and remove mount points down the tree
	void clearChildrenOf(Node *node, const QAbstractItemModel *suppress_disconnect=nullptr) {
		// first recursively remove currently mapped children
		for (Node *child : node->children) {
			clearChildrenOf(child);

			// If child mounts a model, unmount it
			// Needs to be done *after* removing of children (which also refer to child->mount)
			removeExistingMountAt(child, suppress_disconnect);

			// actually delete child
			delete child;
		}
		node->children.clear();
	}

	// mount empty model (i.e. prune) at parent_node, remove_mount existing mount if neccessary
	// not intended for recursive use
	void detachModelAt(Node* node, const QAbstractItemModel *suppress_disconnect=nullptr) {
		bool do_signal = node->rowCount() > 0;
		if (do_signal)
			Q_EMIT q_ptr->beginRemoveRows(proxyIndex(node), 0, node->rowCount()-1);

		clearChildrenOf(node, suppress_disconnect);
		removeExistingMountAt(node, suppress_disconnect);
		// mount the empty model as mockup model
		createMountAt(node, qEmptyModel());

		if (do_signal)
			Q_EMIT q_ptr->endRemoveRows();
	}

	// check for recursive loops when attaching sub_model at parent_node
	void checkForRecursions(const Node* node, /*const*/ QAbstractItemModel* sub_model) {
		// fast track check: empty model can be mounted everywhere
		if (sub_model == qEmptyModel()) return;

		// traverse tree upwards and check for node's sub_models
		while (node->parent) {
			node = node->parent;
			if (node->model() == sub_model)
				throw std::runtime_error("CompositeProxyItemModel: recursive loop detected");
		}
	}


	QAbstractItemModel* commonSubModel(const QModelIndexList &proxy_indexes) const {
		NOT_IMPLEMENTED
	}
	void _q_sourceModelDestroyed()
	{
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		// unmount all occurences of src_model
		for (auto info : mounts.values(src_model)) {
			assert(info.model == src_model);
			detachModelAt(info.mount_node, src_model);
		}
	}

	void _q_sourceDataChanged(const QModelIndex &source_top_left, const QModelIndex &source_bottom_right, const QVector<int> &roles = QVector<int>()) {
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		QModelIndexList top_lefts = q_ptr->mapFromSource(source_top_left, src_model);
		QModelIndexList bottom_rights = top_lefts;
		if (source_top_left != source_bottom_right) // only perform expensive computation once
			bottom_rights = q_ptr->mapFromSource(source_bottom_right, src_model);

		assert(top_lefts.size() == bottom_rights.size());
		for (auto it_top_left = top_lefts.begin(), it_bottom_right = bottom_rights.begin(), end = top_lefts.end();
		     it_top_left != end; ++it_top_left, ++it_bottom_right) {
			assert(it_top_left->model() == it_bottom_right->model());
			assert(it_top_left->parent() == it_bottom_right->parent());
			Q_EMIT q_ptr->dataChanged(*it_top_left, *it_bottom_right, roles);
		}
	}

	void _q_sourceHeaderDataChanged(Qt::Orientation orientation, int start, int end) {
		if (orientation == Qt::Horizontal && header_labels.size() == 0)
			Q_EMIT q_ptr->headerDataChanged(orientation, start, end);
	}

	void _q_sourceAboutToBeReset() {
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		if (src_model == root.model()) {
			q_ptr->beginResetModel();
			clearChildrenOf(&root);
		} else { // reset of sub models requires to remove (and later insert) rows
			for (auto mount : mounts.values(src_model)) {
				Node* node = mount.mount_node;
				if (node->rowCount() == 0)
					continue;
				q_ptr->beginRemoveRows(proxyIndex(node), 0, node->rowCount()-1);
				clearChildrenOf(node);
				q_ptr->endRemoveRows();
			}
		}
	}

	void _q_sourceReset() {
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		if (src_model == root.model())
			q_ptr->endResetModel();
		else; // lazy fetch doesn't require to add rows here
	}

	void _q_sourceRowsInserted(const QModelIndex &src_parent, int start, int end) {
		assert(start >= 0 && end >= start);

		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		const QModelIndexList& parents = q_ptr->mapFromSource(src_parent, src_model);

		for (const auto parent : parents) {
			Node* parent_node = node(parent);
			assert(parent_node->model() == src_model);
			auto &children = parent_node->children;
			assert(start <= children.size());

			q_ptr->beginInsertRows(parent, start, end);
			// prepare insertion
			children.reserve(children.size() + end-start+1);
			parent_node->available_src_rows += end-start+1;
			assert(parent_node->available_src_rows == (uint)src_model->rowCount(src_parent));
			// insert new nodes
			for (int r=start; r <= end; ++r)
				children.insert(r, new Node(parent_node, src_model->index(r, 0, src_parent)));
			// update moved nodes
			for (int r=end+1, end=children.size(); r != end; ++r)
				children.at(r)->src_index = src_model->index(r, 0, src_parent);
			q_ptr->endInsertRows();
		}
	}

	void _q_sourceRowsAboutToBeRemoved(const QModelIndex &src_parent, int start, int end) {
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		const QModelIndexList& parents = q_ptr->mapFromSource(src_parent, src_model);

		for (const auto &parent : parents) {
			q_ptr->beginRemoveRows(parent, start, end);
		}
	}

	void _q_sourceRowsRemoved(const QModelIndex &src_parent, int start, int end) {
		assert(start >= 0 && end >= start);

		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		const QModelIndexList& parents = q_ptr->mapFromSource(src_parent, src_model);

		for (const auto &parent : parents) {
			Node* parent_node = node(parent);
			assert(parent_node->model() == src_model);
			auto &children = parent_node->children;
			assert(end < children.size());

			parent_node->available_src_rows -= end-start+1;
			assert(parent_node->available_src_rows == (uint)src_model->rowCount(src_parent));
			// remove nodes
			for (int r=start; r <= end; ++r) {
				Node *child = children.at(r);
				clearChildrenOf(child);
				removeExistingMountAt(child);
				delete child;
			}
			children.remove(start, end-start+1);

			// update remaining nodes
			for (int r=start, end=children.size(); r != end; ++r)
				children.at(r)->src_index = src_model->index(r, 0, src_parent);

			q_ptr->endRemoveRows();
		}
	}

	void _q_sourceRowsMoved(const QModelIndex &src_parent, int start, int end,
	                        const QModelIndex &dest_parent, int dest) {
		assert(start >= 0 && end >= start);

		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		const QModelIndexList& parents = q_ptr->mapFromSource(src_parent, src_model);

		if (src_parent != dest_parent)
			NOT_IMPLEMENTED;

		// move within same parent
		assert(start > dest || end <= dest);
		qDebug() << "moving rows is untested!";
		for (const auto parent : parents) {
			Node* parent_node = node(parent);
			assert(parent_node->model() == src_model);
			auto &children = parent_node->children;
			assert(end < children.size());

			q_ptr->beginMoveRows(parent, start, end, parent, dest);

			// backup to-be-moved section
			int count = end - start + 1;
			auto move_section = children.mid(start, count);
			// move affected section (++++)
			auto o = children.begin(), d = children.begin();
			if (end < dest) { // [-----S....E++++D----] -> [-----++++ES....D----]
				std::copy(o + end, o + dest, o + start);
				d = o + dest - count;
			} else { // [-----D++++S....E----] -> [-----S....D++++E----]
				std::copy(o + dest, o + start, o + dest + count);
				d = o + dest;
			}
			// restore move_section
			std::copy(move_section.constBegin(), move_section.constData(), d);

			// update all (TODO: only affected) nodes
			for (int r=0, end=children.size(); r != end; ++r)
				children.at(r)->src_index = src_model->index(r, 0, src_parent);

			q_ptr->endMoveRows();
		}
	}

	void _q_sourceColumnsInserted(const QModelIndex &source_parent, int start, int end) NOT_IMPLEMENTED

	void _q_sourceColumnsRemoved(const QModelIndex &source_parent, int start, int end) NOT_IMPLEMENTED

	void _q_sourceColumnsMoved(const QModelIndex &sourceParent, int sourceStart, int sourceEnd,
	                           const QModelIndex &destParent, int dest) NOT_IMPLEMENTED


	// this pair of functions is called when the source model changed layout of items, e.g. due to sorting
	void _q_sourceLayoutAboutToBeChanged(QList<QPersistentModelIndex> source_parents = QList<QPersistentModelIndex>(),
	                                     QAbstractItemModel::LayoutChangeHint hint = QAbstractItemModel::NoLayoutChangeHint)
	{
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		if (source_parents.isEmpty()) { // if no specific source parents are provided, use the mount roots
			for (const auto &mount : mounts.values())
			source_parents << mount.root_index;
		}
		// create a list of proxy_parents that are affected by source_parents
		// the will reported upstream as changed
		QList<QPersistentModelIndex> proxy_parents;
		proxy_parents.reserve(source_parents.size());
		for (const QPersistentModelIndex &source_parent : source_parents) {
			const QModelIndexList mapped_proxy_parents = q_ptr->mapFromSource(source_parent, src_model);
			for (const QModelIndex &proxy_parent : mapped_proxy_parents)
				proxy_parents << proxy_parent;
		}

		Q_EMIT q_ptr->layoutAboutToBeChanged(proxy_parents, hint);
	}

	void _q_sourceLayoutChanged(QList<QPersistentModelIndex> source_parents = QList<QPersistentModelIndex>(),
	                            QAbstractItemModel::LayoutChangeHint hint = QAbstractItemModel::NoLayoutChangeHint)
	{
		const QAbstractItemModel* src_model = static_cast<const QAbstractItemModel*>(q_ptr->sender());
		if (source_parents.isEmpty()) { // if no specific source parents are provided, use the mount roots
			for (const auto &mount : mounts.values()) {
				source_parents << mount.root_index;
				update_node_tree(mount.mount_node);
			}
		}
		// prepare an updated list of parents to report upstream
		QList<QPersistentModelIndex> proxy_parents;
		proxy_parents.reserve(source_parents.size());
		for (const QPersistentModelIndex &source_index : source_parents) {
			const QModelIndexList proxy_indexes = q_ptr->mapFromSource(source_index, src_model);
			for (const QModelIndex &proxy_index : proxy_indexes)
				proxy_parents << proxy_index;
		}

		// update model's persistent indexes
		for (const QPersistentModelIndex &old_index : q_ptr->persistentIndexList()) {
			Node* index_node = node(old_index);
			Node* parent_node = index_node->parent;
			int row = parent_node->children.indexOf(index_node);
			q_ptr->changePersistentIndex(old_index, q_ptr->createIndex(row, 0, index_node));
		}

		Q_EMIT q_ptr->layoutChanged(proxy_parents, hint);
	}

	// reorder children of index to match new order in source model
	void update_node_tree(Node *parent) {
		if (parent->children.isEmpty()) // nothing to do
			return;

		QVector<Node*> permutation; // new order of children
		permutation.resize(parent->rowCount());
		for (int row = 0, end = parent->rowCount(); row != end; ++row) {
			Node *child = parent->children.at(row);
			assert(row == child->src_index.row());
			const QModelIndex &new_src_index = child->model()->index(row, 0, child->src_index.parent());
			assert(new_src_index.isValid() && new_src_index.row() < end);
			permutation[new_src_index.row()] = child;
		}

		// store new order
		std::swap(parent->children, permutation);

		// recursively proceed until next mount point
		for (int row = 0, end = parent->rowCount(); row != end; ++row) {
			Node *child = parent->children.at(row);
			if (!child->overridesSubModel())
				update_node_tree(child);
		}
	}
};

CompositeProxyItemModel::CompositeProxyItemModel(QObject *parent) : QAbstractItemModel(parent)
{
	d_ptr = new CompositeProxyItemModelPrivate(this);
}

CompositeProxyItemModel::CompositeProxyItemModel(QAbstractItemModel *root_model, QObject *parent)
    : CompositeProxyItemModel(parent)
{
	attachModel(QModelIndex(), root_model);
}

CompositeProxyItemModel::~CompositeProxyItemModel()
{
	delete d_ptr;
}

void CompositeProxyItemModel::setHorizontalHeaderLabels(const QStringList &labels)
{
	const CompositeProxyItemModelPrivate::Node &root = d_ptr->root;
	d_ptr->header_labels.resize(qMax(labels.size(), root.columnCount()));
	std::copy(labels.constBegin(), labels.constEnd(), d_ptr->header_labels.begin());
	Q_EMIT headerDataChanged(Qt::Horizontal, 0, columnCount());
}
bool CompositeProxyItemModel::setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role) {

	if (orientation != Qt::Horizontal) // only accept changes to horizontal header
		return false;

	if (section < 0 || section >= columnCount()) // sanity check
		return false;

	if (d_ptr->header_labels.isEmpty()) {
		int mapped_column = d_ptr->root.mapColumn(section);
		if (mapped_column < 0)
			return false;
		return d_ptr->root.model()->setHeaderData(mapped_column, orientation, value, role);
	} else {
		if (role != Qt::DisplayRole)
			return false;
		d_ptr->header_labels[section] = value.toString();
		Q_EMIT headerDataChanged(Qt::Horizontal, section, section+1);
		return true;
	}
}

QVariant CompositeProxyItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal) {
		// when defined, header labels take precedence over root model's header
		if (d_ptr->header_labels.size()) {
			if (role == Qt::DisplayRole)
				return d_ptr->header_labels.value(section);
		} else {
			int mapped_column = d_ptr->root.mapColumn(section);
			if (mapped_column < 0)
				return QVariant();
			return d_ptr->root.model()->headerData(mapped_column, orientation, role);
		}
	} else if (orientation == Qt::Vertical && role == Qt::DisplayRole)
		return section+1;
	return QVariant();
}

void CompositeProxyItemModel::attachModel(QModelIndex parent,
                                          QAbstractItemModel* sub_model,
                                          const QModelIndex &src_root_index,
                                          const QVector<int> &column_mapping) {
	// ensure that parent index refers to first column
	if (parent.isValid() && parent.column() != 0) {
		qWarning() << "CompositeProxyItemModel: model will be attached in column 0, instead of " << parent.column();
		parent = parent.model()->sibling(parent.row(), 0, parent);
	}

	CompositeProxyItemModelPrivate::Node *parent_node = d_ptr->node(parent);
	if (sub_model == nullptr) sub_model = qEmptyModel();

	if (parent_node->overridesSubModel() && parent_node->model() == sub_model)
		return; // nothing to do (already mounted)

	// avoid recursive loops
	d_ptr->checkForRecursions(parent_node, sub_model);
	// detach old sub model
	d_ptr->detachModelAt(parent_node);
	// and attach new sub model
	d_ptr->createMountAt(parent_node, sub_model, src_root_index, column_mapping);
	if (sub_model == qEmptyModel()) return;

	parent_node->available_src_rows = parent_node->model()->rowCount(parent_node->queryIndex());
	d_ptr->connect_signals(sub_model);
}

void CompositeProxyItemModel::detachModel(QModelIndex parent) {
	// ensure that parent index refers to first column
	if (parent.isValid() && parent.column() != 0) {
		qWarning() << "CompositeProxyItemModel: model will be attached in column 0, instead of " << parent.column();
		parent = parent.model()->sibling(parent.row(), 0, parent);
	}
	CompositeProxyItemModelPrivate::Node *parent_node = d_ptr->node(parent);

	// detach old sub model
	d_ptr->detachModelAt(parent_node);
}

bool CompositeProxyItemModel::overridesSubModel(const QModelIndex &index) const {
	return index.column() <= 0 && d_ptr->node(index)->overridesSubModel();
}

bool CompositeProxyItemModel::mountsSubModel(const QModelIndex &index) const
{
	return overridesSubModel(index) && d_ptr->node(index)->model() != qEmptyModel();
}

QAbstractItemModel* CompositeProxyItemModel::subModel(const QModelIndex &index)
{
	return d_ptr->node(index)->model();
}

// Attention: A mounting point always has two model indexes: the one in the parent model
// and the one (the root_index) in the mounted model. Here we return the index
// in the parent model, as this is the one to retrieve the data from.
// The sister method mapToSourceDeep() maps to the subModel's index.
QModelIndex CompositeProxyItemModel::mapToSource(const QModelIndex &index) const {
	if (!index.isValid())
		return QModelIndex();
	if (index.model() != this) {
		qWarning() << "CompositeProxyItemModel: index from wrong model passed to mapToSource";
		Q_ASSERT(!"CompositeProxyItemModel: index from wrong model passed to mapToSource");
		return QModelIndex();
	}

	// row + column indexes are mapped 1:1, we only need to choose the correct parent index
	return d_ptr->node(index)->parent->sourceIndex(index.row(), index.column());
}

QModelIndex CompositeProxyItemModel::mapToSourceDeep(const QModelIndex &index) const {
	if (mountsSubModel(index))
		return d_ptr->node(index)->mount->root_index;
	else
		return mapToSource(index);
}

// A sub model might be mounted at different locations with different root indexes.
// Hence, we need to return the matching proxy index for each of them.
// As source index might be NIL, we also need to provide the corresponding source model.
QModelIndexList CompositeProxyItemModel::mapFromSource(const QModelIndex &src_index,
                                                       const QAbstractItemModel* src_model) const
{
	QModelIndexList indexes;
	if (src_index.isValid()) {
		assert(src_model == nullptr || src_model == src_index.model());
		src_model = src_index.model();
	} else if (src_model == nullptr)
		return indexes;

	for (const auto &info :	d_ptr->mounts.values(src_model)) {
		assert(info.model == src_model);
		QModelIndex proxy_index = d_ptr->mapFromSource(src_index, info.root_index,
		                                               d_ptr->proxyIndex(info.mount_node),
		                                               info.column_mapping);
		if (src_index.isValid() && !proxy_index.isValid())
			continue;
		indexes << proxy_index;
	}
	return indexes;
}

QItemSelection CompositeProxyItemModel::mapSelectionToSource(const QItemSelection &selection) const {
	QItemSelection source_selection;
	for (const auto &index : selection.indexes()) {
		if (!index.isValid()) // ignore invalid indexes: selection indexes need to be valid
			continue;
		const QModelIndex source_index = mapToSource(index);
		if (!source_index.isValid())
			continue;
		source_selection << QItemSelectionRange(source_index);
	}
	return source_selection;
}

QItemSelection CompositeProxyItemModel::mapSelectionFromSource(const QItemSelection &source_selection) const {
	QItemSelection selection;
	for (const auto &source_index : source_selection.indexes()) {
		if (!source_index.isValid()) // ignore invalid indexes: selection indexes need to be valid
			continue;
		for (const auto &index : mapFromSource(source_index, source_index.model()))
			selection << QItemSelectionRange(index);
	}
	return selection;
}

bool CompositeProxyItemModel::hasChildren(const QModelIndex &index) const
{
	const CompositeProxyItemModelPrivate::Node* node = d_ptr->node(index);
	return node->model()->hasChildren(node->queryIndex());
}

bool CompositeProxyItemModel::canFetchMore(const QModelIndex &index) const
{
	return d_ptr->canFetchMore(d_ptr->node(index));
}

void CompositeProxyItemModel::fetchMore(const QModelIndex &index)
{
	d_ptr->fetchMore(d_ptr->node(index));
}

int CompositeProxyItemModel::rowCount(const QModelIndex &parent) const
{
	if (parent.column() > 0) return 0;
	return d_ptr->rowCount(parent);
}

int CompositeProxyItemModel::columnCount(const QModelIndex &parent) const
{
	if (parent.column() > 0) return 0;
	return d_ptr->columnCount(parent);
}

QModelIndex CompositeProxyItemModel::index(int row, int column, const QModelIndex &parent) const
{
#if _WIN32
	if (row < 0 || row >= rowCount(parent) || column < 0 || column >= columnCount())
		return QModelIndex();
#endif
	assert(row >= 0 && row < rowCount(parent));
	assert(column >= 0 && column < columnCount());

	// sub model might have fewer columns than main model
	if (column >= columnCount(parent))
		return QModelIndex();

	const CompositeProxyItemModelPrivate::Node *parent_node = d_ptr->node(parent);
	const CompositeProxyItemModelPrivate::Node *index_node = parent_node->children.at(row);

	return createIndex(row, column, const_cast<CompositeProxyItemModelPrivate::Node*>(index_node));
}

QModelIndex CompositeProxyItemModel::parent(const QModelIndex &index) const
{
	return d_ptr->proxyIndex(d_ptr->node(index)->parent);
}

Qt::ItemFlags CompositeProxyItemModel::flags(const QModelIndex &index) const
{
	Qt::ItemFlags flags = QAbstractItemModel::flags(index); // default flags
	const QModelIndex& src_index = mapToSource(index);
	if (src_index.isValid())
		flags |= src_index.model()->flags(src_index); // flags of visible item

	CompositeProxyItemModelPrivate::Node *node = d_ptr->node(index);
	if (node->overridesSubModel()) // merge flags of sub model's root index (needed for drag-n-drop)
		flags |= node->model()->flags(node->queryIndex());
	return flags & ~Qt::ItemNeverHasChildren;
}

QVariant CompositeProxyItemModel::data(const QModelIndex &index, int role) const
{
	const QModelIndex& src_index = mapToSource(index);
	if (src_index.isValid())
		return src_index.model()->data(src_index, role);
	return QVariant();
}

QMap<int, QVariant> CompositeProxyItemModel::itemData(const QModelIndex &index) const
{
	const QModelIndex& src_index = mapToSource(index);
	if (src_index.isValid())
		return src_index.model()->itemData(src_index);
	return QMap<int, QVariant>();
}

bool CompositeProxyItemModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	QModelIndex src_index = mapToSource(index);
	if (src_index.isValid())
		return const_cast<QAbstractItemModel*>(src_index.model())->setData(src_index, value, role);
	return false;
}

bool CompositeProxyItemModel::setItemData(const QModelIndex &index, const QMap< int, QVariant >& roles)
{
	const QModelIndex& src_index = mapToSource(index);
	if (src_index.isValid())
		return const_cast<QAbstractItemModel*>(src_index.model())->setItemData(src_index, roles);
	return false;
}


QStringList CompositeProxyItemModel::mimeTypes() const
{
	// retrieve supported mime types of all mounted models
	QStringList result;
	for (auto it = d_ptr->mounts.begin(), end = d_ptr->mounts.end(); it != end; ++it)
		result += it.key()->mimeTypes();
	result.removeDuplicates();
	return result;
}

QMimeData* CompositeProxyItemModel::mimeData(const QModelIndexList &indexes) const
{
	// we can only provide mime information if indexes originate from same source model
	QAbstractItemModel *model = d_ptr->commonSubModel(indexes);
	if (!model)
		return nullptr;

	QSet<QModelIndex> unique_source_indexes;
	for (const QModelIndex &index : indexes)
		unique_source_indexes += mapToSource(index);
	return model->mimeData(unique_source_indexes.toList());
}

bool CompositeProxyItemModel::canDropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent) const
{
	CompositeProxyItemModelPrivate::Node *parent_node = d_ptr->node(parent);
	bool result = parent_node->model()->canDropMimeData(data, action, row, column, parent_node->queryIndex());
	return result;
}

bool CompositeProxyItemModel::dropMimeData(const QMimeData *data, Qt::DropAction action,
                                           int row, int column, const QModelIndex &parent) {
	CompositeProxyItemModelPrivate::Node *parent_node = d_ptr->node(parent);
	return parent_node->model()->dropMimeData(data, action, row, column, parent_node->queryIndex());
}


bool CompositeProxyItemModel::insertRows(int row, int count, const QModelIndex &parent)
{
	auto node = d_ptr->node(parent);
	return node->model()->insertRows(row, count, node->queryIndex());
}

bool CompositeProxyItemModel::removeRows(int row, int count, const QModelIndex &parent)
{
	auto node = d_ptr->node(parent);
	return node->model()->removeRows(row, count, node->queryIndex());
}

bool CompositeProxyItemModel::insertColumns(int column, int count, const QModelIndex &parent)
{
	auto node = d_ptr->node(parent);
	return node->model()->insertColumns(column, count, node->queryIndex());
}

bool CompositeProxyItemModel::removeColumns(int column, int count, const QModelIndex &parent)
{
	auto node = d_ptr->node(parent);
	return node->model()->removeColumns(column, count, node->queryIndex());
}

QModelIndex CompositeProxyItemModel::buddy(const QModelIndex &index) const
{
	if (!index.isValid())
		return QModelIndex();
	QModelIndex src_index = mapToSource(index);
	QModelIndex buddy = src_index.model()->buddy(src_index);
	if (src_index == buddy)
		return index;
	QModelIndexList buddies = mapFromSource(buddy, subModel(index));
	if (buddies.contains(index))
		return index;

	assert(buddies.isEmpty() == false);
	return buddies.first();
}

QModelIndexList CompositeProxyItemModel::match(const QModelIndex &start, int role, const QVariant &value,
                                               int hits, Qt::MatchFlags flags) const
{
	NOT_IMPLEMENTED
}

void CompositeProxyItemModel::sort(int column, Qt::SortOrder order)
{
	// forward to all sub models
	for (const auto &mount : d_ptr->mounts)
		mount.model->sort(column, order);
}

QSize CompositeProxyItemModel::span(const QModelIndex &index) const
{
	const QModelIndex& src_index = mapToSource(index);
	if (src_index.isValid())
		return src_index.model()->span(src_index);
	return QSize();
}

// this needs to be included for CMAKE_AUTOMOC
#include "moc_composite_proxy_model.cpp"

} }
