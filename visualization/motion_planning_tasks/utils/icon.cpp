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

#include "icon.h"

#include <QApplication>
#include <QIcon>
#include <QImage>
#include <QMetaEnum>
#include <QPainter>
#include <QPaintEngine>
#include <QWidget>

namespace moveit_rviz_plugin {
namespace utils {

static const qreal PUNCH_EDGE_WIDTH = 0.5;
static const qreal PUNCH_EDGE_INTENSITY = 0.6;

static QPixmap maskToColorAndAlpha(const QPixmap& mask, const QColor& color) {
	QImage result(mask.toImage().convertToFormat(QImage::Format_ARGB32));
	result.setDevicePixelRatio(mask.devicePixelRatio());
	QRgb* bits_start = reinterpret_cast<QRgb*>(result.bits());
	const QRgb* bits_end = bits_start + result.width() * result.height();
	const QRgb tint = color.rgb() & 0x00ffffff;
	const QRgb alpha = QRgb(color.alpha());
	for (QRgb* pixel = bits_start; pixel < bits_end; ++pixel) {
		QRgb pixel_alpha = (((~*pixel) & 0xff) * alpha) >> 8;
		*pixel = (pixel_alpha << 24) | tint;
	}
	return QPixmap::fromImage(result);
}

using MaskAndColor = QPair<QPixmap, QColor>;
using MasksAndColors = QList<MaskAndColor>;
static MasksAndColors masksAndColors(const Icon& icon, int /*dpr*/) {
	MasksAndColors result;
	for (const IconMaskAndColor& i : icon) {
		const QString& file_name = i.first;
		const QColor color = i.second;
		result.append(qMakePair(QPixmap(file_name), color));
	}
	return result;
}

static void smearPixmap(QPainter* painter, const QPixmap& pixmap, qreal radius) {
	const qreal nagative = -radius - 0.01;  // Workaround for QPainter rounding behavior
	const qreal positive = radius;
	painter->drawPixmap(QPointF(nagative, nagative), pixmap);
	painter->drawPixmap(QPointF(0, nagative), pixmap);
	painter->drawPixmap(QPointF(positive, nagative), pixmap);
	painter->drawPixmap(QPointF(positive, 0), pixmap);
	painter->drawPixmap(QPointF(positive, positive), pixmap);
	painter->drawPixmap(QPointF(0, positive), pixmap);
	painter->drawPixmap(QPointF(nagative, positive), pixmap);
	painter->drawPixmap(QPointF(nagative, 0), pixmap);
}

static QPixmap combinedMask(const MasksAndColors& masks, Icon::IconStyleOptions style) {
	if (masks.count() == 1)
		return masks.first().first;

	QPixmap result(masks.first().first);
	QPainter p(&result);
	p.setCompositionMode(QPainter::CompositionMode_Darken);
	auto mask_image = masks.constBegin();
	mask_image++;
	for (; mask_image != masks.constEnd(); ++mask_image) {
		if (style & Icon::PUNCH_EDGES) {
			p.save();
			p.setOpacity(PUNCH_EDGE_INTENSITY);
			p.setCompositionMode(QPainter::CompositionMode_Lighten);
			smearPixmap(&p, maskToColorAndAlpha((*mask_image).first, Qt::white), PUNCH_EDGE_WIDTH);
			p.restore();
		}
		p.drawPixmap(0, 0, (*mask_image).first);
	}
	p.end();
	return result;
}

static QPixmap masksToIcon(const MasksAndColors& masks, const QPixmap& combinedMask, Icon::IconStyleOptions style) {
	QPixmap result(combinedMask.size());
	result.setDevicePixelRatio(combinedMask.devicePixelRatio());
	result.fill(Qt::transparent);
	QPainter p(&result);

	for (MasksAndColors::const_iterator mask_image = masks.constBegin(); mask_image != masks.constEnd(); ++mask_image) {
		if (style & Icon::PUNCH_EDGES && mask_image != masks.constBegin()) {
			// Punch a transparent outline around an overlay.
			p.save();
			p.setOpacity(PUNCH_EDGE_INTENSITY);
			p.setCompositionMode(QPainter::CompositionMode_DestinationOut);
			smearPixmap(&p, maskToColorAndAlpha((*mask_image).first, Qt::white), PUNCH_EDGE_WIDTH);
			p.restore();
		}
		p.drawPixmap(0, 0, maskToColorAndAlpha((*mask_image).first, (*mask_image).second));
	}

	if (style & Icon::DROP_SHADOW) {
		const QPixmap shadow_mask = maskToColorAndAlpha(combinedMask, Qt::black);
		p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
		p.setOpacity(0.05);
		p.drawPixmap(QPointF(0, -0.501), shadow_mask);
		p.drawPixmap(QPointF(-0.501, 0), shadow_mask);
		p.drawPixmap(QPointF(0.5, 0), shadow_mask);
		p.drawPixmap(QPointF(0.5, 0.5), shadow_mask);
		p.drawPixmap(QPointF(-0.501, 0.5), shadow_mask);
		p.setOpacity(0.2);
		p.drawPixmap(0, 1, shadow_mask);
	}

	p.end();

	return result;
}

static QPixmap combinedPlainPixmaps(const QVector<IconMaskAndColor>& images) {
	QPixmap result(images.first().first);
	auto pixmap = images.constBegin();
	pixmap++;
	for (; pixmap != images.constEnd(); ++pixmap) {
		const QPixmap overlay((*pixmap).first);
		result.paintEngine()->painter()->drawPixmap(0, 0, overlay);
	}
	return result;
}

Icon::Icon() {}

Icon::Icon(std::initializer_list<IconMaskAndColor> args, Icon::IconStyleOptions style)
  : QVector<IconMaskAndColor>(args), m_style(style) {}

Icon::Icon(const QString& imageFileName) : m_style(NONE) {
	append({ imageFileName, QColor() });
}

QIcon Icon::icon() const {
	if (isEmpty()) {
		return QIcon();
	} else if (m_style == NONE) {
		return QIcon(combinedPlainPixmaps(*this));
	} else {
		QIcon result;
		const int max_dpr = qRound(qApp->devicePixelRatio());
		for (int dpr = 1; dpr <= max_dpr; dpr++) {
			const MasksAndColors masks = masksAndColors(*this, dpr);
			const QPixmap combined_mask = combinedMask(masks, m_style);
			result.addPixmap(masksToIcon(masks, combined_mask, m_style));

			const QColor disabled_color = QColor::fromRgba(0x60a4a6a8);
			result.addPixmap(maskToColorAndAlpha(combined_mask, disabled_color), QIcon::Disabled);
		}
		return result;
	}
}

QPixmap Icon::pixmap() const {
	if (isEmpty()) {
		return QPixmap();
	} else if (m_style == NONE) {
		return combinedPlainPixmaps(*this);
	} else {
		const MasksAndColors masks = masksAndColors(*this, qRound(qApp->devicePixelRatio()));
		const QPixmap combined_mask = combinedMask(masks, m_style);
		return masksToIcon(masks, combined_mask, m_style);
	}
}

QString Icon::imageFileName() const {
	return first().first;
}

QIcon Icon::combinedIcon(const QList<QIcon>& icons) {
	QIcon result;
	QWindow* window = QApplication::allWidgets().first()->windowHandle();
	for (const QIcon& icon : icons)
		for (const QIcon::Mode mode : { QIcon::Disabled, QIcon::Normal })
			for (const QSize& size : icon.availableSizes(mode))
				result.addPixmap(icon.pixmap(window, size, mode), mode);
	return result;
}
}  // namespace utils
}  // namespace moveit_rviz_plugin
