#pragma once
#include <rviz/panel.h>

namespace moveit_rviz_plugin {

/** The TaskPanel displays information about manipulation tasks in the system.
 *  Subscribing to task_monitoring and task_solution topics, it collects information
 *  about running tasks and their solutions and allows to inspect both,
 *  successful solutions and failed solution attempts.
 */
class TaskPanelPrivate;
class TaskPanel: public rviz::Panel
{
	Q_OBJECT
	Q_DECLARE_PRIVATE(TaskPanel)
	TaskPanelPrivate *d_ptr;

public:
	TaskPanel(QWidget* parent = 0);

	void onInitialize() override;

	void load(const rviz::Config& config) override;
	void save(rviz::Config config) const override;

private:
	Q_PRIVATE_SLOT(d_func(), void _q_changedTaskMonitorTopic())
};

}
