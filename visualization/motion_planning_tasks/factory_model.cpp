#include "factory_model.h"
#include <rviz/load_resource.h>

namespace moveit_rviz_plugin {

FactoryModel::FactoryModel(rviz::Factory *factory, QObject *parent)
   : QStandardItemModel(parent)
{
	setHorizontalHeaderLabels({tr("Name")});
	fillTree(factory);
}

void FactoryModel::fillTree(rviz::Factory *factory)
{
	QIcon default_package_icon = rviz::loadPixmap( "package://rviz/icons/default_package_icon.png" );

	QStringList classes = factory->getDeclaredClassIds();
	classes.sort();

	// Map from package names to the corresponding top-level tree widget items.
	std::map<QString, QStandardItem*> package_items;

	for(const QString& lookup_name : classes)
	{
		QString package = factory->getClassPackage(lookup_name);

		QStandardItem* package_item;
		auto mi = package_items.find(package);
		if(mi == package_items.end())
		{
			package_item = new QStandardItem(default_package_icon, package);
			package_items[package] = package_item;
			appendRow(package_item);
		}
		else
		{
			package_item = mi->second;
		}
		QStandardItem* class_item = new QStandardItem(factory->getIcon(lookup_name),
		                                              factory->getClassName(lookup_name));
		class_item->setWhatsThis(factory->getClassDescription(lookup_name));
		class_item->setData(lookup_name);
		class_item->setDragEnabled(true);
		package_item->appendRow(class_item);
	}
}

}
