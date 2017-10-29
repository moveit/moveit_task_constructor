#pragma once

#include <QAbstractItemModel>
#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor/Task.h>
#include <moveit_task_constructor/Solution.h>

#include <memory>

namespace moveit_rviz_plugin {

class LocalTask;
class RemoteTask;

/** The TaskModel collects/provides information about Task instances available in the system.
 *
 *  We distinguish between local and remote task instances.
 *  Local instances are created by insertLocalTask().
 *  Remote instances are discovered via processTaskMessage() / processSolutionMessage().
 */
class TaskModelPrivate;
class TaskModel : public QAbstractItemModel
{
	Q_OBJECT
	Q_DECLARE_PRIVATE(TaskModel)
	TaskModelPrivate* d_ptr;

public:
	TaskModel(QObject *parent = nullptr);
	~TaskModel();

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override { return 3; }

	QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex &index) const override;

	Qt::ItemFlags flags(const QModelIndex & index) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;

	/// process an incoming task message - only call in Qt's main loop
	void processTaskMessage(const moveit_task_constructor::Task &msg);
	/// process an incoming solution message - only call in Qt's main loop
	void processSolutionMessage(const moveit_task_constructor::Solution &msg);

private:
	void processTaskMessage(RemoteTask *root, const moveit_task_constructor::Task &msg);
};
MOVEIT_CLASS_FORWARD(TaskModel)
typedef std::weak_ptr<TaskModel> TaskModelWeakPtr;

}
