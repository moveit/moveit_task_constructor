#include "task_model.h"
#include "stage_wrapper.h"
#include <moveit_task_constructor/stage.h>
#include <ros/console.h>

namespace moveit_rviz_plugin {

class TaskModelPrivate {
public:
	typedef std::vector<std::unique_ptr<StageWrapperInterface>> task_array_type;
	TaskModel* q_ptr;
	// the vector of all visible tasks in the model
	task_array_type tasks;
	// map from remote task IDs to tasks
	// if task is destroyed remotely, it is marked with flag IS_DESTROYED
	// if task is removed locally from tasks vector, it is marked with a nullptr
	std::map<std::string, RemoteTask*> remote_tasks;

public:
	TaskModelPrivate(TaskModel* q_ptr) : q_ptr(q_ptr) {}

	inline bool indexValid(const QModelIndex &index) const {
		return (index.row() >= 0) && (index.column() >= 0) && (index.model() == q_ptr);
	}

	StageWrapperInterface *node(const QModelIndex &index) const;
	QModelIndex index(StageWrapperInterface *node) const;
};

TaskModel::TaskModel(QObject *parent)
   : QAbstractItemModel(parent)
   , d_ptr (new TaskModelPrivate(this))
{
	ROS_DEBUG_NAMED("TaskModel", "created task model: %p", this);
}

TaskModel::~TaskModel() {
	ROS_DEBUG_NAMED("TaskModel", "destroying task model: %p", this);
	delete d_ptr;
}

int TaskModel::rowCount(const QModelIndex &parent) const
{
	Q_D(const TaskModel);
	if (parent.column() > 0)
		return 0;

	if (!parent.isValid())
		return d_ptr->tasks.size();

	const StageWrapperInterface* parentNode = d->node(parent);
	parentNode->flags |= WAS_VISITED;
	return parentNode->numChildren();
}

QModelIndex TaskModel::index(int row, int column, const QModelIndex &parent) const
{
	Q_D(const TaskModel);
	if (row < 0 || column < 0 || row >= rowCount(parent) || column >= columnCount(parent))
		return QModelIndex();

	StageWrapperInterface* indexNode
	      = (d->indexValid(parent)
	         ? d->node(parent)->child(row)
	         : d->tasks[row].get());
	Q_ASSERT(indexNode);
	return createIndex(row, column, indexNode);
}

inline StageWrapperInterface* TaskModelPrivate::node(const QModelIndex &index) const
{
	Q_ASSERT(index.isValid());
	return static_cast<StageWrapperInterface*>(index.internalPointer());
}

QModelIndex TaskModel::parent(const QModelIndex &index) const
{
	Q_D(const TaskModel);
	if (!d->indexValid(index))
		return QModelIndex();

	StageWrapperInterface* indexNode = d->node(index);
	return d->index(indexNode->parent());
}

QModelIndex TaskModelPrivate::index(StageWrapperInterface* node) const
{
	if (node == nullptr) return QModelIndex();

	// get parent's row
	int row;
	StageWrapperInterface* parentNode = node->parent();
	if (parentNode == nullptr) {
		auto it = std::find_if(tasks.begin(), tasks.end(),
		                       [node](task_array_type::const_reference n) { return n.get() == node; });
		Q_ASSERT(it != tasks.end());
		row = it - tasks.begin();
	} else {
		int end = parentNode->numChildren();
		for (row = 0; row != end; ++row)
			if (parentNode->child(row) == node)
				break;
		Q_ASSERT(row != end);
	}
	return q_ptr->createIndex(row, 0, node);
}

Qt::ItemFlags TaskModel::flags(const QModelIndex &index) const
{
	Q_D(const TaskModel);
	Qt::ItemFlags flags = QAbstractItemModel::flags(index);
	if (index.column() == 0)
		flags |= Qt::ItemIsEditable; // name is editable
	return flags;
}

QVariant TaskModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole) {
		switch (section) {
		case 0: return tr("Name");
		case 1: return tr("# solved");
		case 2: return tr("# failed");
		}
	}
	return QAbstractItemModel::headerData(section, orientation, role);
}

QVariant TaskModel::data(const QModelIndex &index, int role) const
{
	Q_D(const TaskModel);
	if (!d->indexValid(index))
		return QVariant();

	StageWrapperInterface* indexNode = d->node(index);
	switch (role) {
	case Qt::EditRole:
	case Qt::DisplayRole:
		switch (index.column()) {
		case 0: return QString::fromStdString(indexNode->name());
		case 1: return uint(indexNode->numSolved());
		case 2: return uint(indexNode->numFailed());
		}
		break;
	}
	return QVariant();
}

bool TaskModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	Q_D(const TaskModel);
	if (!d->indexValid(index) || index.column() != 0 || role != Qt::EditRole)
		return false;

	StageWrapperInterface* indexNode = d->node(index);
	return indexNode->setName(value.toString().toStdString());
}

// process a task monitoring message:
// update existing RemoteTask, create a new one,
// or (if msg.stages is empty) delete an existing one
void TaskModel::processTaskMessage(const moveit_task_constructor::Task &msg)
{
	Q_D(TaskModel);

	// retrieve existing or insert new remote task for given id
	auto it_inserted = d->remote_tasks.insert(std::make_pair(msg.id, nullptr));
	bool created = it_inserted.second;
	if (created) // create new task, if ID was not known before
		it_inserted.first->second = new RemoteTask();
	RemoteTask *remote_task = it_inserted.first->second;

	// empty stages list indicates, that this remote task is not available anymore
	if (msg.stages.empty()) {
		// task was already deleted locally, now we can remove it from remote_tasks
		if (!remote_task) d->remote_tasks.erase(it_inserted.first);
		// task is still in use, mark it as destroyed
		else remote_task->flags |= IS_DESTROYED;
	}
	if (!remote_task) return; // task is not in use anymore
	processTaskMessage(remote_task, msg);

	// notify model about newly created task
	if (created) {
		ROS_DEBUG_NAMED("TaskModel", "received new Task: %s", msg.id.c_str());
		int row = rowCount();
		beginInsertRows(QModelIndex(), row, row);
		d->tasks.push_back(std::unique_ptr<RemoteTask>(remote_task));
		endInsertRows();
	}
}

typedef moveit_task_constructor::Stage StageMsg;
static std::map<StageFlag, moveit::task_constructor::InterfaceFlags>
remote_stage_flags = {
   { READS_START, moveit::task_constructor::READS_START },
   { READS_END,moveit::task_constructor::READS_END },
   { WRITES_NEXT_START, moveit::task_constructor::WRITES_NEXT_START },
   { WRITES_PREV_END, moveit::task_constructor::WRITES_PREV_END },
};

void TaskModel::processTaskMessage(RemoteTask* root, const moveit_task_constructor::Task &msg)
{
	Q_D(TaskModel);

	// iterate over msg.stages and create new nodes where needed
	for (const StageMsg &s : msg.stages) {
		RemoteStage *parent = nullptr;
		// find parent for any non-root stage
		if (s.parent_id != 0) {
			auto parent_it = root->id_to_stage.find(s.parent_id);
			if (parent_it == root->id_to_stage.end()) {
				ROS_ERROR_NAMED("TaskModel", "No parent found for stage %d (%s)", s.id, s.name.c_str());
				continue;
			}
			parent = parent_it->second;
		}

		auto it_inserted = root->id_to_stage.insert(std::make_pair(s.id, s.parent_id == 0 ? root : nullptr));
		RemoteStage*& stage = it_inserted.first->second;
		if (!stage) // if just inserted, create new stage
			stage = new RemoteStage(parent);
		else
			assert (stage->parent() == parent || stage == root);

		// set content of stage
		bool changed = stage->setName(s.name);
		StageFlags old = stage->flags;
		for (const auto &pair : remote_stage_flags) {
			if (s.id & pair.second)
				stage->flags |= pair.first;
			else
				stage->flags &= ~pair.first;
		}
		changed |= (stage->flags != old);

		// notify model about changes to stage (if neccessary)
		if (changed && (stage->flags & WAS_VISITED)) {
			QModelIndex idx = d->index(stage);
			dataChanged(idx, idx);
		}

		// insert newly created stage into model (root insertion is handled externally)
		if (it_inserted.second && stage != root) {
			bool notify = parent->flags & WAS_VISITED;
			QModelIndex parentIdx = notify ? d->index(parent) : QModelIndex();
			int row = rowCount(parentIdx);
			if (notify) beginInsertRows(parentIdx, row, row);
			parent->push_back(std::unique_ptr<RemoteStage>(stage));
			if (notify) endInsertRows();
		}
	}
}

void TaskModel::processSolutionMessage(const moveit_task_constructor::Solution &msg)
{
	// TODO
}

}
