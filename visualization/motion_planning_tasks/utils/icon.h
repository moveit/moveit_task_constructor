/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is adapted from Qt Creator (replacing theme stuff by QColor)
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 as published by the Free Software
** Foundation with exceptions as appearing in the file LICENSE.GPL3-EXCEPT
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
****************************************************************************/

#pragma once

#include <QPair>
#include <QVector>

class QColor;
class QIcon;
class QPixmap;
class QString;

namespace moveit_rviz_plugin {
namespace utils {

typedef QPair<QString, QColor> IconMaskAndColor;

// Returns a recolored icon with shadow and custom disabled state for a
// series of grayscalemask|Theme::Color mask pairs
class Icon : public QVector<IconMaskAndColor>
{
public:
	enum IconStyleOption
	{
		None = 0,
		Tint = 1,
		DropShadow = 2,
		PunchEdges = 4,

		ToolBarStyle = Tint | DropShadow | PunchEdges,
		MenuTintedStyle = Tint | PunchEdges
	};

	Q_DECLARE_FLAGS(IconStyleOptions, IconStyleOption)

	Icon();
	Icon(std::initializer_list<IconMaskAndColor> args, IconStyleOptions style = ToolBarStyle);
	Icon(const QString& imageFileName);
	Icon(const Icon& other) = default;

	QIcon icon() const;
	// Same as icon() but without disabled state.
	QPixmap pixmap() const;

	// Try to avoid it. it is just there for special API cases in Qt Creator
	// where icons are still defined as filename.
	QString imageFileName() const;

	// Returns either the classic or a themed icon depending on
	// the current Theme::FlatModeIcons flag.
	static QIcon sideBarIcon(const Icon& classic, const Icon& flat);
	// Like sideBarIcon plus added action mode for the flat icon
	static QIcon modeIcon(const Icon& classic, const Icon& flat, const Icon& flatActive);

	// Combined icon pixmaps in Normal and Disabled states from several QIcons
	static QIcon combinedIcon(const QList<QIcon>& icons);

private:
	IconStyleOptions m_style = None;
};
}
}

Q_DECLARE_OPERATORS_FOR_FLAGS(moveit_rviz_plugin::utils::Icon::IconStyleOptions)
