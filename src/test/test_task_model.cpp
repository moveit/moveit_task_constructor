#include <task_model.h>
#include <gtest/gtest.h>
#include <initializer_list>

class TaskModelTest : public ::testing::Test {
protected:
	moveit_rviz_plugin::TaskModel model;
	int children = 0;
	int num_inserts = 0;
	int num_updates = 0;

	moveit_task_constructor::Task genMsg(const std::string &name) {
		moveit_task_constructor::Task t;
		t.id = name;
		uint id = 0;

		moveit_task_constructor::Stage root;
		root.parent_id = id;
		root.id = ++id;
		root.name = name;
		t.stages.push_back(root);

		for (int i = 0; i != children; ++i) {
			moveit_task_constructor::Stage child;
			child.parent_id = root.id;
			child.id = ++id;
			child.name = std::to_string(i);
			t.stages.push_back(child);
		}
		return t;
	}

	void validate(const std::initializer_list<const char*> &expected) {
		// validate root index
		ASSERT_EQ(model.rowCount(), expected.size());
		EXPECT_EQ(model.columnCount(), 3);
		EXPECT_EQ(model.parent(QModelIndex()), QModelIndex());

		// validate first-level items
		auto it = expected.begin();
		for (size_t row = 0; row < expected.size(); ++row, ++it) {
			QModelIndex idx = model.index(row, 0);
			EXPECT_EQ(idx.row(), row);
			EXPECT_EQ(idx.column(), 0);
			EXPECT_EQ(idx.model(), &model);
			EXPECT_EQ(model.parent(idx), QModelIndex());

			// validate data
			EXPECT_STREQ(model.data(idx).toByteArray().constData(), *it);
			EXPECT_EQ(model.data(model.index(row, 1)).toUInt(), 0);
			EXPECT_EQ(model.data(model.index(row, 2)).toUInt(), 0);

			// validate children
			ASSERT_EQ(model.rowCount(idx), children);
			EXPECT_EQ(model.columnCount(idx), 3);
			EXPECT_EQ(model.rowCount(model.index(row, 1)), 0);

			// validate second-level items
			for (int child = 0; child < children; ++child) {
				QModelIndex childIdx = model.index(child, 0, idx);
				EXPECT_EQ(childIdx.row(), child);
				EXPECT_EQ(childIdx.column(), 0);
				EXPECT_EQ(childIdx.model(), &model);
				EXPECT_EQ(model.parent(childIdx), idx);

				EXPECT_EQ(model.data(childIdx).toString().toStdString(), std::to_string(child));
				EXPECT_EQ(model.rowCount(childIdx), 0);
				EXPECT_EQ(model.columnCount(childIdx), 3);
			}
		}
	}

	void populateAndValidate() {
		{ SCOPED_TRACE("empty"); validate({}); }
		EXPECT_EQ(num_inserts, 0);
		EXPECT_EQ(num_updates, 0);

		for (int i = 0; i < 2; ++i) {
			SCOPED_TRACE("first i=" + std::to_string(i));
			num_inserts = 0;
			num_updates = 0;
			model.processTaskMessage(genMsg("first"));

			if (i == 0) EXPECT_EQ(num_inserts, 1);  // 1 notify for inserted task
			else EXPECT_EQ(num_inserts, 0);
			EXPECT_EQ(num_updates, 0);

			validate({"first"});
		}
		for (int i = 0; i < 2; ++i) {
			SCOPED_TRACE("second i=" + std::to_string(i));
			num_inserts = 0;
			num_updates = 0;
			model.processTaskMessage(genMsg("second"));  // 1 notify for inserted task

			if (i == 0) EXPECT_EQ(num_inserts, 1);
			else EXPECT_EQ(num_inserts, 0);
			EXPECT_EQ(num_updates, 0);

			validate({"first", "second"});
		}
	}

	void SetUp() {
		QObject::connect(&model, &QAbstractItemModel::rowsAboutToBeInserted,
		                 [this](){ ++num_inserts; });
		QObject::connect(&model, &QAbstractItemModel::dataChanged,
		                 [this](){ ++num_updates; });
	}
	void TearDown() {}
};

TEST_F(TaskModelTest, noChildren) {
	children = 0;
	populateAndValidate();
}

TEST_F(TaskModelTest, threeChildren) {
	children = 3;
	populateAndValidate();
}

TEST_F(TaskModelTest, visitedPopulate) {
	// first population without children
	children = 0;
	model.processTaskMessage(genMsg("first"));
	validate({"first"}); // validation visits root node
	EXPECT_EQ(num_inserts, 1);

	children = 3;
	num_inserts = 0;
	model.processTaskMessage(genMsg("first"));
	validate({"first"});
	// second population with children should emit insert notifies for them
	EXPECT_EQ(num_inserts, 3);
	EXPECT_EQ(num_updates, 0);
}
