#include <src/remote_task_model.h>
#include <algorithm>
#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

using namespace moveit_rviz_plugin;

class SolutionModelTest : public ::testing::Test
{
protected:
	template <typename T>
	inline std::vector<T> reversed(const std::vector<T>& in) {
		return std::vector<T>(in.rbegin(), in.rend());
	}
	void validateSorting(QAbstractTableModel& model, int sort_column, Qt::SortOrder sort_order,
	                     const std::vector<uint32_t>& expected_id_order) {
		SCOPED_TRACE(qPrintable(QString("sorting in %1 %2 order")
		                            .arg(sort_order == Qt::AscendingOrder ? "ascending" : "descending")
		                            .arg(sort_column == 0 ? "creation" : "cost")));
		model.sort(sort_column, sort_order);
		std::vector<uint32_t> actual_id_order(model.rowCount());
		for (int row = 0; row < model.rowCount(); ++row)
			actual_id_order[row] = model.data(model.index(row, 0), Qt::UserRole).toInt();
		EXPECT_THAT(actual_id_order, ::testing::ElementsAreArray(expected_id_order));
	}
	void processAndValidate(RemoteSolutionModel& model, const std::vector<uint32_t>& success_ids,
	                        const std::vector<uint32_t>& failure_ids) {
		model.processSolutionIDs(success_ids, failure_ids, failure_ids.size(), 0.0);

		std::vector<uint32_t> cost_ordered_ids(success_ids.begin(), success_ids.end());
		std::vector<uint32_t> sorted_failure_ids(failure_ids.begin(), failure_ids.end());
		std::sort(sorted_failure_ids.begin(), sorted_failure_ids.end());
		std::copy(sorted_failure_ids.begin(), sorted_failure_ids.end(), std::back_inserter(cost_ordered_ids));

		std::vector<uint32_t> creation_ordered_ids(cost_ordered_ids.size());
		for (size_t i = 0; i < cost_ordered_ids.size(); ++i)
			creation_ordered_ids[i] = i + 1;

		validateSorting(model, 0, Qt::AscendingOrder, creation_ordered_ids);
		validateSorting(model, 0, Qt::DescendingOrder, reversed(creation_ordered_ids));

		validateSorting(model, 1, Qt::AscendingOrder, cost_ordered_ids);
		validateSorting(model, 1, Qt::DescendingOrder, reversed(cost_ordered_ids));
	}
};

#define processAndValidate(...)                             \
	{                                                        \
		SCOPED_TRACE("processSolutionIDs(" #__VA_ARGS__ ")"); \
		processAndValidate(model, __VA_ARGS__);               \
	}

TEST_F(SolutionModelTest, sorting) {
	RemoteSolutionModel model;
	processAndValidate({ 1, 3 }, { 2 });
	processAndValidate({ 4, 1, 6, 3 }, { 5, 2 });
}
