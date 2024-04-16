#include <utils/flat_merge_proxy_model.h>
#include <utils/tree_merge_proxy_model.h>

#include <QStandardItemModel>
#include <gtest/gtest.h>

using namespace moveit_rviz_plugin::utils;

void createStandardItems(QStandardItem* parent, int rows, int columns, int depth, int d_rows = 0) {
	if (depth <= 0 || rows <= 0 || columns <= 0)
		return;

	for (int row = 0; row < rows; ++row) {
		QList<QStandardItem*> items;
		for (int column = 0; column < columns; ++column)
			items << new QStandardItem(QString("%2: %0, %1").arg(row).arg(column).arg(depth));
		parent->appendRow(items);
		createStandardItems(items.first(), rows + d_rows, columns, depth - 1, d_rows);
	}
}

QStandardItemModel* createStandardModel(QObject* parent, int rows, int columns, int depth, int d_rows = 0) {
	QStandardItemModel* model = new QStandardItemModel(parent);
	createStandardItems(model->invisibleRootItem(), rows, columns, depth, d_rows);
	return model;
}

// tell gtest how to print QVariant
::std::ostream& operator<<(::std::ostream& os, const QVariant& value) {
	return os << value.toString().toStdString();
}

// compare structure and content of proxy and source model
void checkEquality(const QAbstractItemModel* proxy_model, const QModelIndex& proxy_parent,
                   const QAbstractItemModel* source_model, const QModelIndex& source_parent) {
	ASSERT_EQ(proxy_model->rowCount(proxy_parent), source_model->rowCount(source_parent));
	ASSERT_EQ(proxy_model->columnCount(proxy_parent), source_model->columnCount(source_parent));

	for (int r = proxy_model->rowCount(proxy_parent) - 1; r >= 0; --r) {
		for (int c = proxy_model->columnCount(proxy_parent) - 1; c >= 0; --c) {
			QModelIndex proxy_child = proxy_model->index(r, c, proxy_parent);
			QModelIndex source_child = source_model->index(r, c, source_parent);

			// validate equality of data
			if (proxy_child.isValid() && source_child.isValid()) {
				EXPECT_EQ(proxy_model->data(proxy_child), source_model->data(source_child));
				EXPECT_EQ(proxy_child.row(), source_child.row());
				EXPECT_EQ(proxy_child.column(), source_child.column());
				EXPECT_EQ(proxy_child.internalPointer(), source_child.internalPointer());
			}

			// recursively check children
			checkEquality(proxy_model, proxy_child, source_model, source_child);
		}
	}
}

template <class T>
void modifySourceModel(QAbstractItemModel* src, T* proxy, const QModelIndex& src_index) {
	bool called = false;
	const char* new_value = "foo";

	auto con =
	    proxy->connect(proxy, &T::dataChanged,
	                   [&called, &src_index, new_value](const QModelIndex& topLeft, const QModelIndex& bottomRight) {
		                   EXPECT_EQ(topLeft.row(), src_index.row());
		                   EXPECT_EQ(topLeft.column(), src_index.column());

		                   EXPECT_EQ(bottomRight.row(), src_index.row());
		                   EXPECT_EQ(bottomRight.column(), src_index.column());

		                   EXPECT_STREQ(topLeft.data().toString().toLatin1().data(), new_value);
		                   called = true;
	                   });

	EXPECT_TRUE(src->setData(src_index, new_value));

	proxy->disconnect(con);
	EXPECT_TRUE(called);
}

TEST(TreeMergeModel, basics) {
	TreeMergeProxyModel tree;

	EXPECT_EQ(tree.parent(QModelIndex()), QModelIndex());
	EXPECT_EQ(tree.rowCount(), 0);
	EXPECT_EQ(tree.columnCount(), 0);

	QAbstractItemModel* m = createStandardModel(&tree, 2, 3, 2);
	tree.insertModel("M1", m);
	EXPECT_FALSE(tree.insertModel("M1", m));  // a model may only inserted once

	EXPECT_EQ(tree.rowCount(), 1);
	EXPECT_EQ(tree.columnCount(), 3);
	EXPECT_TRUE(tree.index(0, 0).flags() & Qt::ItemIsSelectable);
	EXPECT_FALSE(tree.index(0, 1).flags() & Qt::ItemIsSelectable);
	EXPECT_FALSE(tree.index(1, 0).isValid());

	QModelIndex idx = tree.index(0, 0);
	ASSERT_TRUE(idx.isValid());
	EXPECT_EQ(idx.row(), 0);
	EXPECT_EQ(idx.column(), 0);
	EXPECT_EQ(idx.model(), &tree);
	EXPECT_EQ(idx.internalPointer(), &tree);

	EXPECT_FALSE(idx.parent().isValid());
	EXPECT_STREQ(idx.data().toString().toUtf8().data(), "M1");

	checkEquality(&tree, idx, m, QModelIndex());

	// add second model
	m = createStandardModel(&tree, 3, 3, 2);
	tree.insertModel("M2", m);

	// modify underlying model before accessing anything else
	modifySourceModel(m, &tree, m->index(1, 1));
	modifySourceModel(m, &tree, m->index(0, 2, m->index(1, 0)));

	EXPECT_EQ(tree.rowCount(), 2);
	EXPECT_EQ(tree.columnCount(), 3);

	idx = tree.index(1, 0);
	ASSERT_TRUE(idx.isValid());
	EXPECT_EQ(idx.row(), 1);
	EXPECT_EQ(idx.column(), 0);
	EXPECT_EQ(idx.model(), &tree);
	EXPECT_EQ(idx.internalPointer(), &tree);

	EXPECT_FALSE(idx.parent().isValid());
	EXPECT_STREQ(idx.data().toString().toUtf8().data(), "M2");

	checkEquality(&tree, idx, m, QModelIndex());

	// remove middle, top-level row of second model
	EXPECT_EQ(tree.rowCount(idx), 3);
	m->removeRows(1, 1);
	EXPECT_EQ(tree.rowCount(), 2);
	EXPECT_EQ(tree.rowCount(idx), 2);

	// remove first model
	ASSERT_TRUE(tree.removeModel(0));
	EXPECT_EQ(tree.rowCount(), 1);
	checkEquality(&tree, tree.index(0, 0), m, QModelIndex());

	// remove model by pointer
	EXPECT_FALSE(tree.removeModel(nullptr));
	ASSERT_TRUE(tree.removeModel(m));
	EXPECT_EQ(tree.rowCount(), 0);
}

TEST(FlatMergeModel, basics) {
	FlatMergeProxyModel flat;

	EXPECT_EQ(flat.parent(QModelIndex()), QModelIndex());
	EXPECT_EQ(flat.rowCount(), 0);
	EXPECT_EQ(flat.columnCount(), 0);

	QAbstractItemModel* m1 = createStandardModel(&flat, 2, 3, 2);
	flat.insertModel(m1);
	EXPECT_FALSE(flat.insertModel(m1));  // a model may only be inserted once

	// modify underlying model before accessing anything else
	modifySourceModel(m1, &flat, m1->index(1, 1));
	modifySourceModel(m1, &flat, m1->index(0, 2, m1->index(1, 0)));

	EXPECT_EQ(flat.rowCount(), 2);
	EXPECT_EQ(flat.columnCount(), 3);

	QModelIndex idx = flat.index(0, 0);
	ASSERT_TRUE(idx.isValid());
	EXPECT_EQ(idx.row(), 0);
	EXPECT_EQ(idx.column(), 0);
	EXPECT_EQ(idx.model(), &flat);

	EXPECT_FALSE(idx.parent().isValid());

	checkEquality(&flat, QModelIndex(), m1, QModelIndex());

	// add second model
	QAbstractItemModel* m2 = createStandardModel(&flat, 3, 3, 1);
	flat.insertModel(m2);

	EXPECT_EQ(flat.rowCount(), 2 + 3);
	EXPECT_EQ(flat.columnCount(), 3);
	EXPECT_EQ(flat.index(2, 0).data(), m2->index(0, 0).data());

	EXPECT_EQ(flat.getModel(flat.index(1, 1)), std::make_pair(m1, m1->index(1, 1)));
	EXPECT_EQ(flat.getModel(flat.index(2, 2)), std::make_pair(m2, m2->index(0, 2)));
}

class FlatMergeModelRemove : public ::testing::Test
{
protected:
	FlatMergeProxyModel flat;
	void SetUp() override {
		flat.insertModel(createStandardModel(&flat, 1, 3, 2));
		flat.insertModel(createStandardModel(&flat, 2, 3, 2));
		flat.insertModel(createStandardModel(&flat, 3, 3, 2));
	}
};

TEST_F(FlatMergeModelRemove, remove1) {
	ASSERT_EQ(flat.rowCount(), 1 + 2 + 3);
	EXPECT_TRUE(flat.removeRows(0, 1));
	EXPECT_EQ(flat.rowCount(), 2 + 3);
}

TEST_F(FlatMergeModelRemove, remove2) {
	ASSERT_EQ(flat.rowCount(), 1 + 2 + 3);
	EXPECT_TRUE(flat.removeRows(0, 2));
	EXPECT_EQ(flat.rowCount(), 1 + 3);
}

TEST_F(FlatMergeModelRemove, remove3) {
	ASSERT_EQ(flat.rowCount(), 1 + 2 + 3);
	EXPECT_TRUE(flat.removeRows(0, 3));
	EXPECT_EQ(flat.rowCount(), 3);
}

TEST_F(FlatMergeModelRemove, remove4) {
	ASSERT_EQ(flat.rowCount(), 1 + 2 + 3);
	EXPECT_TRUE(flat.removeRows(2, 2));
	EXPECT_EQ(flat.rowCount(), 1 + 1 + 2);
}
