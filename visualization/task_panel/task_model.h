#pragma once

#include <QAbstractItemModel>
#include <moveit_task_constructor/Task.h>

#include <memory>

namespace moveit_rviz_plugin {

class RemoteTask;
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

	/// process an incoming task message
	void processTaskMessage(const moveit_task_constructor::Task &msg);

private:
	void processTaskMessage(RemoteTask *root, const moveit_task_constructor::Task &msg);
};

}
