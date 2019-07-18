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

static const qreal PunchEdgeWidth = 0.5;
static const qreal PunchEdgeIntensity = 0.6;

static QPixmap maskToColorAndAlpha(const QPixmap& mask, const QColor& color) {
	QImage result(mask.toImage().convertToFormat(QImage::Format_ARGB32));
	result.setDevicePixelRatio(mask.devicePixelRatio());
	QRgb* bitsStart = reinterpret_cast<QRgb*>(result.bits());
	const QRgb* bitsEnd = bitsStart + result.width() * result.height();
	const QRgb tint = color.rgb() & 0x00ffffff;
	const QRgb alpha = QRgb(color.alpha());
	for (QRgb* pixel = bitsStart; pixel < bitsEnd; ++pixel) {
		QRgb pixelAlpha = (((~*pixel) & 0xff) * alpha) >> 8;
		*pixel = (pixelAlpha << 24) | tint;
	}
	return QPixmap::fromImage(result);
}

typedef QPair<QPixmap, QColor> MaskAndColor;
typedef QList<MaskAndColor> MasksAndColors;
static MasksAndColors masksAndColors(const Icon& icon, int dpr) {
	MasksAndColors result;
	for (const IconMaskAndColor& i : icon) {
		const QString& fileName = i.first;
		const QColor color = i.second;
		result.append(qMakePair(QPixmap(fileName), color));
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
	auto maskImage = masks.constBegin();
	maskImage++;
	for (; maskImage != masks.constEnd(); ++maskImage) {
		if (style & Icon::PunchEdges) {
			p.save();
			p.setOpacity(PunchEdgeIntensity);
			p.setCompositionMode(QPainter::CompositionMode_Lighten);
			smearPixmap(&p, maskToColorAndAlpha((*maskImage).first, Qt::white), PunchEdgeWidth);
			p.restore();
		}
		p.drawPixmap(0, 0, (*maskImage).first);
	}
	p.end();
	return result;
}

static QPixmap masksToIcon(const MasksAndColors& masks, const QPixmap& combinedMask, Icon::IconStyleOptions style) {
	QPixmap result(combinedMask.size());
	result.setDevicePixelRatio(combinedMask.devicePixelRatio());
	result.fill(Qt::transparent);
	QPainter p(&result);

	for (MasksAndColors::const_iterator maskImage = masks.constBegin(); maskImage != masks.constEnd(); ++maskImage) {
		if (style & Icon::PunchEdges && maskImage != masks.constBegin()) {
			// Punch a transparent outline around an overlay.
			p.save();
			p.setOpacity(PunchEdgeIntensity);
			p.setCompositionMode(QPainter::CompositionMode_DestinationOut);
			smearPixmap(&p, maskToColorAndAlpha((*maskImage).first, Qt::white), PunchEdgeWidth);
			p.restore();
		}
		p.drawPixmap(0, 0, maskToColorAndAlpha((*maskImage).first, (*maskImage).second));
	}

	if (style & Icon::DropShadow) {
		const QPixmap shadowMask = maskToColorAndAlpha(combinedMask, Qt::black);
		p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
		p.setOpacity(0.05);
		p.drawPixmap(QPointF(0, -0.501), shadowMask);
		p.drawPixmap(QPointF(-0.501, 0), shadowMask);
		p.drawPixmap(QPointF(0.5, 0), shadowMask);
		p.drawPixmap(QPointF(0.5, 0.5), shadowMask);
		p.drawPixmap(QPointF(-0.501, 0.5), shadowMask);
		p.setOpacity(0.2);
		p.drawPixmap(0, 1, shadowMask);
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

Icon::Icon(const QString& imageFileName) : m_style(None) {
	append({ imageFileName, QColor() });
}

QIcon Icon::icon() const {
	if (isEmpty()) {
		return QIcon();
	} else if (m_style == None) {
		return QIcon(combinedPlainPixmaps(*this));
	} else {
		QIcon result;
		const int maxDpr = qRound(qApp->devicePixelRatio());
		for (int dpr = 1; dpr <= maxDpr; dpr++) {
			const MasksAndColors masks = masksAndColors(*this, dpr);
			const QPixmap combined_mask = combinedMask(masks, m_style);
			result.addPixmap(masksToIcon(masks, combined_mask, m_style));

			const QColor disabledColor = QColor::fromRgba(0x60a4a6a8);
			result.addPixmap(maskToColorAndAlpha(combined_mask, disabledColor), QIcon::Disabled);
		}
		return result;
	}
}

QPixmap Icon::pixmap() const {
	if (isEmpty()) {
		return QPixmap();
	} else if (m_style == None) {
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

QIcon Icon::sideBarIcon(const Icon& classic, const Icon& flat) {
	return flat.icon();
}

QIcon Icon::modeIcon(const Icon& classic, const Icon& flat, const Icon& flatActive) {
	QIcon result = sideBarIcon(classic, flat);
	result.addPixmap(flatActive.pixmap(), QIcon::Active);
	return result;
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
}
}
