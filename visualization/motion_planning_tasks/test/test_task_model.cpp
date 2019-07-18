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

#include <task_list_model.h>
#include <local_task_model.h>
#include <remote_task_model.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>

#include <ros/init.h>
#include <gtest/gtest.h>
#include <initializer_list>
#include <qcoreapplication.h>

using namespace moveit::task_constructor;

class TaskListModelTest : public ::testing::Test
{
protected:
	moveit_rviz_plugin::TaskListModel model;
	int children = 0;
	int num_inserts = 0;
	int num_updates = 0;

	moveit_task_constructor_msgs::TaskDescription genMsg(const std::string& name) {
		moveit_task_constructor_msgs::TaskDescription t;
		t.id = name;
		uint id = 0, root_id;

		moveit_task_constructor_msgs::StageDescription desc;
		desc.parent_id = id;
		desc.id = root_id = ++id;
		desc.name = name;
		t.stages.push_back(desc);

		for (int i = 0; i != children; ++i) {
			desc.parent_id = root_id;
			desc.id = ++id;
			desc.name = std::to_string(i);
			t.stages.push_back(desc);
		}
		return t;
	}

	void validate(QAbstractItemModel& model, const std::initializer_list<const char*>& expected) {
		// validate root index
		ASSERT_EQ(model.rowCount(), static_cast<int>(expected.size()));
		EXPECT_EQ(model.columnCount(), 3);
		EXPECT_EQ(model.parent(QModelIndex()), QModelIndex());

		// validate first-level items
		auto it = expected.begin();
		for (size_t row = 0; row < expected.size(); ++row, ++it) {
			QModelIndex idx = model.index(row, 0);
			EXPECT_EQ(idx.row(), static_cast<int>(row));
			EXPECT_EQ(idx.column(), 0);
			EXPECT_EQ(idx.model(), &model);
			EXPECT_EQ(model.parent(idx), QModelIndex());

			// validate data
			EXPECT_STREQ(model.data(idx).toByteArray().constData(), *it);
			EXPECT_EQ(model.data(model.index(row, 1)).toUInt(), 0u);
			EXPECT_EQ(model.data(model.index(row, 2)).toUInt(), 0u);

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
		{
			SCOPED_TRACE("empty");
			validate(model, {});
		}
		EXPECT_EQ(num_inserts, 0);
		EXPECT_EQ(num_updates, 0);

		for (int i = 0; i < 2; ++i) {
			SCOPED_TRACE("first i=" + std::to_string(i));
			num_inserts = 0;
			num_updates = 0;
			model.processTaskDescriptionMessage("1", genMsg("first"));

			if (i == 0)
				EXPECT_EQ(num_inserts, 1);  // 1 notify for inserted task
			else
				EXPECT_EQ(num_inserts, 0);
			EXPECT_EQ(num_updates, 0);

			validate(model, { "first" });
		}
		for (int i = 0; i < 2; ++i) {
			SCOPED_TRACE("second i=" + std::to_string(i));
			num_inserts = 0;
			num_updates = 0;
			model.processTaskDescriptionMessage("2", genMsg("second"));  // 1 notify for inserted task

			if (i == 0)
				EXPECT_EQ(num_inserts, 1);
			else
				EXPECT_EQ(num_inserts, 0);
			EXPECT_EQ(num_updates, 0);

			validate(model, { "first", "second" });
		}
	}

	void SetUp() {
		QObject::connect(&model, &QAbstractItemModel::rowsAboutToBeInserted, [this]() { ++num_inserts; });
		QObject::connect(&model, &QAbstractItemModel::dataChanged, [this]() { ++num_updates; });
	}
	void TearDown() {}
};

TEST_F(TaskListModelTest, remoteTaskModel) {
	children = 3;
	planning_scene::PlanningSceneConstPtr scene;
	moveit_rviz_plugin::RemoteTaskModel m(scene, nullptr);
	m.processStageDescriptions(genMsg("first").stages);
	SCOPED_TRACE("first");
	validate(m, { "first" });
}

TEST_F(TaskListModelTest, localTaskModel) {
	int argc = 0;
	char* argv = nullptr;
	ros::init(argc, &argv, "testLocalTaskModel");

	children = 3;
	const char* task_name = "task pipeline";
	moveit_rviz_plugin::LocalTaskModel m(std::make_unique<SerialContainer>(task_name),
	                                     planning_scene::PlanningSceneConstPtr(), nullptr);
	for (int i = 0; i != children; ++i)
		m.add(std::make_unique<stages::CurrentState>(std::to_string(i)));

	{
		SCOPED_TRACE("localTaskModel");
		validate(m, { task_name });
	}
}

TEST_F(TaskListModelTest, noChildren) {
	children = 0;
	populateAndValidate();
}

TEST_F(TaskListModelTest, threeChildren) {
	children = 3;
	populateAndValidate();
}

TEST_F(TaskListModelTest, visitedPopulate) {
	// first population without children
	children = 0;
	model.processTaskDescriptionMessage("1", genMsg("first"));
	validate(model, { "first" });  // validation visits root node
	EXPECT_EQ(num_inserts, 1);

	children = 3;
	num_inserts = 0;
	model.processTaskDescriptionMessage("1", genMsg("first"));
	validate(model, { "first" });
	// second population with children should emit insert notifies for them
	EXPECT_EQ(num_inserts, 3);
	EXPECT_EQ(num_updates, 0);
}

TEST_F(TaskListModelTest, deletion) {
	children = 3;
	model.processTaskDescriptionMessage("1", genMsg("first"));
	auto m = model.getModel(model.index(0, 0)).first;
	int num_deletes = 0;
	QObject::connect(m, &QObject::destroyed, [&num_deletes]() { ++num_deletes; });

	model.removeModel(m);
	// process deleteLater() events
	QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
	// as m is owned by model, m should be destroyed
	// EXPECT_EQ(num_deletes, 1); // TODO: event is not processed, missing event loop?
	EXPECT_EQ(model.rowCount(), 0);
}
