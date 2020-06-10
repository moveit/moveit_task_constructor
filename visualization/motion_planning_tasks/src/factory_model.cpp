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

#include "factory_model.h"
#include <rviz/load_resource.h>
#include <QMimeData>
#include <QSet>

namespace moveit_rviz_plugin {

FactoryModel::FactoryModel(rviz::Factory& factory, const QString& mime_type, QObject* parent)
  : QStandardItemModel(parent), mime_type_(mime_type) {
	setHorizontalHeaderLabels({ tr("Name") });
	fillTree(factory);
}

void FactoryModel::fillTree(rviz::Factory& factory) {
	QIcon default_package_icon = rviz::loadPixmap("package://rviz/icons/default_package_icon.png");

	QStringList classes = factory.getDeclaredClassIds();
	classes.sort();

	// Map from package names to the corresponding top-level tree widget items.
	std::map<QString, QStandardItem*> package_items;

	for (const QString& lookup_name : classes) {
		QString package = factory.getClassPackage(lookup_name);

		QStandardItem* package_item;
		auto mi = package_items.find(package);
		if (mi == package_items.end()) {
			package_item = new QStandardItem(default_package_icon, package);
			package_items[package] = package_item;
			appendRow(package_item);
		} else {
			package_item = mi->second;
		}
		QStandardItem* class_item = new QStandardItem(factory.getIcon(lookup_name), factory.getClassName(lookup_name));
		class_item->setWhatsThis(factory.getClassDescription(lookup_name));
		class_item->setData(lookup_name, Qt::UserRole);
		class_item->setDragEnabled(true);
		package_item->appendRow(class_item);
	}
}

QStringList FactoryModel::mimeTypes() const {
	return { mime_type_ };
}

QMimeData* FactoryModel::mimeData(const QModelIndexList& indexes) const {
	QSet<int> rows_considered;
	QMimeData* mime_data = new QMimeData();
	for (const auto& index : indexes) {
		if (rows_considered.contains(index.row()))
			continue;
		// mime data is lookup_name
		mime_data->setData(mime_type_, index.data(Qt::UserRole).toByteArray());
	}
	return mime_data;
}
}  // namespace moveit_rviz_plugin
