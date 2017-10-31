#pragma once

#include <rviz/factory.h>
#include <QStandardItemModel>

namespace moveit_rviz_plugin {

class FactoryModel : public QStandardItemModel
{
	void fillTree(rviz::Factory *factory);

public:
	FactoryModel(rviz::Factory *factory, QObject *parent = nullptr);
};

}
