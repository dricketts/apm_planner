/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of widget controlling one MAV
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include "QsLog.h"
#include <QString>
#include <QTimer>
#include <QLabel>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>

#include <QProcess>
#include <QPalette>

#include "UASControlWidget.h"
#include <UASManager.h>
#include <UAS.h>
#include "QGC.h"

UASControlWidget::UASControlWidget(QWidget *parent) : QWidget(parent),
    m_uas(0),
    m_uasMode(0),
    m_engineOn(false)
{
    ui.setupUi(this);

    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));
    ui.modeComboBox->clear();
    ui.modeComboBox->insertItem(0, UAS::getShortModeTextFor(MAV_MODE_PREFLIGHT).remove(0, 2), MAV_MODE_PREFLIGHT);
    ui.modeComboBox->insertItem(1, UAS::getShortModeTextFor((MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)).remove(0, 2), (MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
    ui.modeComboBox->insertItem(2, UAS::getShortModeTextFor(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED).remove(0, 2), MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
    ui.modeComboBox->insertItem(3, UAS::getShortModeTextFor((MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED)).remove(0, 2), (MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED));
    ui.modeComboBox->insertItem(4, UAS::getShortModeTextFor((MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED)).remove(0, 2), (MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED));
    ui.modeComboBox->insertItem(5, UAS::getShortModeTextFor((MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_TEST_ENABLED)).remove(0, 2), (MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_TEST_ENABLED));
    connect(ui.modeComboBox, SIGNAL(activated(int)), this, SLOT(setMode(int)));
    connect(ui.setModeButton, SIGNAL(clicked()), this, SLOT(transmitMode()));

    m_uasMode = ui.modeComboBox->itemData(ui.modeComboBox->currentIndex()).toInt();

    ui.modeComboBox->setCurrentIndex(0);

    ui.gridLayout->setAlignment(Qt::AlignTop);

}

void UASControlWidget::setUAS(UASInterface* uas)
{
    if (m_uas != 0)
    {
        UASInterface* oldUAS = UASManager::instance()->getUASForId(this->m_uas);
        disconnect(ui.controlButton, SIGNAL(clicked()), oldUAS, SLOT(armSystem()));
        disconnect(ui.liftoffButton, SIGNAL(clicked()), oldUAS, SLOT(launch()));
        disconnect(ui.landButton, SIGNAL(clicked()), oldUAS, SLOT(home()));
        disconnect(ui.shutdownButton, SIGNAL(clicked()), oldUAS, SLOT(shutdown()));
        //connect(ui.setHomeButton, SIGNAL(clicked()), uas, SLOT(setLocalOriginAtCurrentGPSPosition()));
        disconnect(uas, SIGNAL(modeChanged(int,QString,QString)), this, SLOT(updateMode(int,QString,QString)));
        disconnect(uas, SIGNAL(statusChanged(int)), this, SLOT(updateState(int)));
        disconnect(uas, SIGNAL(shimStatusChanged(int)), this, SLOT(updateState(int)));
	//        disconnect(uas, SIGNAL(broadcastShimParams(bool, bool, float, float, float, float float, uint32_t)),
	//   this, SLOT(displayShimParams(bool, bool, float, float, float, float, float, uint32_t)));
	//        disconnect(ui.shimButton, SIGNAL(clicked()), oldUAS, SLOT(enableShim()));
        disconnect(ui.paramSetButton, SIGNAL(clicked()), oldUAS, SLOT(updateShimParams()));
    }
    if (uas == 0)
    {
        m_uas = 0;
        return;
    }

    // Connect user interface controls
    connect(ui.controlButton, SIGNAL(clicked()), this, SLOT(cycleContextButton()));
    connect(ui.liftoffButton, SIGNAL(clicked()), uas, SLOT(launch()));
    connect(ui.landButton, SIGNAL(clicked()), uas, SLOT(home()));
    connect(ui.shutdownButton, SIGNAL(clicked()), uas, SLOT(shutdown()));
    //connect(ui.setHomeButton, SIGNAL(clicked()), uas, SLOT(setLocalOriginAtCurrentGPSPosition()));
    connect(uas, SIGNAL(modeChanged(int,QString,QString)), this, SLOT(updateMode(int,QString,QString)));
    connect(uas, SIGNAL(statusChanged(int)), this, SLOT(updateState(int)));
    connect(uas, SIGNAL(shimStatusChanged(bool)), this, SLOT(updateShimStatus(bool)));
    //    connect(uas, SIGNAL(broadcastShimParams(bool, bool, float, float, float, float float, uint32_t)),
    //    this, SLOT(displayShimParams(bool, bool, float, float, float, float, float, uint32_t)));

    //    connect(ui.shimButton, SIGNAL(clicked()), this, SLOT(toggleShim()));
    connect(ui.paramSetButton, SIGNAL(clicked()), this, SLOT(updateShimParams()));

    ui.controlStatusLabel->setText(tr("Connected to ") + uas->getUASName());

    m_uas = uas->getUASID();
    setBackgroundColor(uas->getColor());
}

UASControlWidget::~UASControlWidget()
{

}

void UASControlWidget::updateStatemachine()
{

    if (m_engineOn)
    {
        ui.controlButton->setText(tr("DISARM SYSTEM"));
    }
    else
    {
        ui.controlButton->setText(tr("ARM SYSTEM"));
    }
//     if (m_shimOn)
//     {
//         ui.shimButton->setText(tr("DISABLE SHIM"));
//     }
//     else
//     {
//         ui.shimButton->setText(tr("ENABLE SHIM"));
//     }

}

/**
 * Set the background color based on the MAV color. If the MAV is selected as the
 * currently actively controlled system, the frame color is highlighted
 */
void UASControlWidget::setBackgroundColor(QColor color)
{
    // UAS color
    QColor uasColor = color;
    QString colorstyle;
    QString borderColor = "#4A4A4F";
    borderColor = "#FA4A4F";
    uasColor = uasColor.darker(900);
    colorstyle = colorstyle.sprintf("QLabel { border-radius: 3px; padding: 0px; margin: 0px; background-color: #%02X%02X%02X; border: 0px solid %s; }",
                                    uasColor.red(), uasColor.green(), uasColor.blue(), borderColor.toStdString().c_str());
    setStyleSheet(colorstyle);
    QPalette palette = this->palette();
    palette.setBrush(QPalette::Window, QBrush(uasColor));
    setPalette(palette);
    setAutoFillBackground(true);
}


void UASControlWidget::updateMode(int uas,QString mode,QString description)
{
    Q_UNUSED(uas);
    Q_UNUSED(mode);
    Q_UNUSED(description);
}

void UASControlWidget::updateShimStatus(bool enabled)
{
  m_shimOn = enabled;
//   if (m_shimOn)
//   {
//     ui.shimButton->setText(tr("DISABLE SHIM"));
//   }
//   else
//   {
//     ui.shimButton->setText(tr("ENABLE SHIM"));
//   }

}

void UASControlWidget::updateState(int state)
{
    switch (state)
    {
    case (int)MAV_STATE_ACTIVE:
        m_engineOn = true;
        ui.controlButton->setText(tr("DISARM SYSTEM"));
        break;
    case (int)MAV_STATE_STANDBY:
        m_engineOn = false;
        ui.controlButton->setText(tr("ARM SYSTEM"));
        break;
    }
}

/**
 * Called by the button
 */
void UASControlWidget::setMode(int mode)
{
    // Adapt context button mode
    m_uasMode = ui.modeComboBox->itemData(mode).toInt();
    ui.modeComboBox->blockSignals(true);
    ui.modeComboBox->setCurrentIndex(mode);
    ui.modeComboBox->blockSignals(false);

    emit changedMode(mode);
}

void UASControlWidget::transmitMode()
{
    UASInterface* mav = UASManager::instance()->getUASForId(this->m_uas);
    if (mav)
    {
        // include armed state
        if (m_engineOn)
            m_uasMode |= MAV_MODE_FLAG_SAFETY_ARMED;
        else
            m_uasMode &= ~MAV_MODE_FLAG_SAFETY_ARMED;

        mav->setMode(m_uasMode);
        QString mode = ui.modeComboBox->currentText();

        ui.lastActionLabel->setText(QString("Sent new mode %1 to %2").arg(mode).arg(mav->getUASName()));
    }
}

// void UASControlWidget::toggleShim()
// {
//      UAS* mav = dynamic_cast<UAS*>(UASManager::instance()->getUASForId(this->m_uas));
//      if (mav)
//      {
//        if (m_shimOn)
//        {
// 	 mav->disableShim();
//        } else {
// 	 mav->enableShim();
//        }
//      }

//      // Update state now and in several intervals when MAV might have changed state
//      updateStatemachine();

//      QTimer::singleShot(50, this, SLOT(updateStatemachine()));
//      QTimer::singleShot(200, this, SLOT(updateStatemachine()));
// }

// update our shim parameters
void UASControlWidget::updateShimParams()
{
  QString val;
  QFile file;
  file.setFileName("/Users/danielricketts/Development/UAV/apm_planner/src/ui/uas/test1.json");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  val = file.readAll();
  file.close();
  QLOG_DEBUG() << val;
  QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
  QJsonObject params = d.object();
  bool smooth = params["smooth"].toBool();
  QLOG_DEBUG() << smooth;
  uint8_t lookahead = params["lookahead"].toInt();
  QLOG_DEBUG() << lookahead;
  float roll_lb = params["roll_lb"].toDouble();
  QLOG_DEBUG() << roll_lb;
  int16_t abraking = params["abraking"].toInt();
  QLOG_DEBUG() << abraking;
  uint16_t mid_throttle = params["mid_throttle"].toInt();
  QLOG_DEBUG() << mid_throttle;
  
  QJsonObject box1 = params["box1"].toObject();
  int16_t y_ub1 = box1["y_ub"].toInt();
  QLOG_DEBUG() << y_ub1;
  int16_t y_lb1 = box1["y_lb"].toInt();
  QLOG_DEBUG() << y_lb1;
  int16_t vy_ub1 = box1["vy_ub"].toInt();
  QLOG_DEBUG() << vy_ub1;
  int16_t x_ub1 = box1["x_ub"].toInt();
  QLOG_DEBUG() << x_ub1;
  int16_t x_lb1 = box1["x_lb"].toInt();
  QLOG_DEBUG() << x_lb1;
  int16_t vx_ub1 = box1["vx_ub"].toInt();
  QLOG_DEBUG() << vx_ub1;
  
  QJsonObject box2 = params["box2"].toObject();
  int16_t y_ub2 = box2["y_ub"].toInt();
  QLOG_DEBUG() << y_ub2;
  int16_t y_lb2 = box2["y_lb"].toInt();
  QLOG_DEBUG() << y_lb2;
  int16_t vy_ub2 = box2["vy_ub"].toInt();
  QLOG_DEBUG() << vy_ub2;
  int16_t x_ub2 = box2["x_ub"].toInt();
  QLOG_DEBUG() << x_ub2;
  int16_t x_lb2 = box2["x_lb"].toInt();
  QLOG_DEBUG() << x_lb2;
  int16_t vx_ub2 = box2["vx_ub"].toInt();
  QLOG_DEBUG() << vx_ub2;
  
  QJsonObject box3 = params["box3"].toObject();
  int16_t y_ub3 = box3["y_ub"].toInt();
  QLOG_DEBUG() << y_ub3;
  int16_t y_lb3 = box3["y_lb"].toInt();
  QLOG_DEBUG() << y_lb3;
  int16_t vy_ub3 = box3["vy_ub"].toInt();
  QLOG_DEBUG() << vy_ub3;
  int16_t x_ub3 = box3["x_ub"].toInt();
  QLOG_DEBUG() << x_ub3;
  int16_t x_lb3 = box3["x_lb"].toInt();
  QLOG_DEBUG() << x_lb3;
  int16_t vx_ub3 = box3["vx_ub"].toInt();
  QLOG_DEBUG() << vx_ub3;
  
  QJsonObject box4 = params["box4"].toObject();
  int16_t y_ub4 = box4["y_ub"].toInt();
  QLOG_DEBUG() << y_ub4;
  int16_t y_lb4 = box4["y_lb"].toInt();
  QLOG_DEBUG() << y_lb4;
  int16_t vy_ub4 = box4["vy_ub"].toInt();
  QLOG_DEBUG() << vy_ub4;
  int16_t x_ub4 = box4["x_ub"].toInt();
  QLOG_DEBUG() << x_ub4;
  int16_t x_lb4 = box4["x_lb"].toInt();
  QLOG_DEBUG() << x_lb4;
  int16_t vx_ub4 = box4["vx_ub"].toInt();
  QLOG_DEBUG() << vx_ub4;
       
  UAS* mav = dynamic_cast<UAS*>(UASManager::instance()->getUASForId(this->m_uas));
  if (mav)
    {
       mav->setShimParams(smooth,
			  lookahead,
			  roll_lb,
			  abraking,
			  mid_throttle,
			  
			  y_ub1,
			  y_lb1,
			  vy_ub1,
			  x_ub1,
			  x_lb1,
			  vx_ub1,

			  y_ub2,
			  y_lb2,
			  vy_ub2,
			  x_ub2,
			  x_lb2,
			  vx_ub2,

			  y_ub3,
			  y_lb3,
			  vy_ub3,
			  x_ub3,
			  x_lb3,
			  vx_ub3,

			  y_ub4,
			  y_lb4,
			  vy_ub4,
			  x_ub4,
			  x_lb4,
			  vx_ub4);

//        mav->setShimParams(
//         ui.smoothIn->isChecked(),
// 	(float)ui.lookaheadIn->value(),
//         (float)ui.h_ubIn->value(),
// 			  (float)ui.h_lbIn->value(),
// 			  (float)ui.hprime_ubIn->value(),
// 			  (float)ui.hprime_lbIn->value(),
// 			  (float)ui.x_ubIn->value(),
//         (float)ui.x_lbIn->value(),
//         (float)ui.xprime_ubIn->value(),
//         (float)ui.xprime_lbIn->value(),
//         (float)ui.roll_lbIn->value(),
//         (float)ui.abrakingIn->value(),
//         (float)ui.hoverIn->value()
// 			  );

     }
}

// void UASControlWidget::displayShimParams(bool before, bool smooth, float ubverified, float ubunverified,
// 					 float amin, float pwm_accel_scale, float hover_throttle, uint16_t smooth_lookahead)
// {
//   QLOG_DEBUG() << "About to display";
//   ui.statusOut->setText((m_shimOn ? "enabled " : "disabled ") +
// 			(before ? "before " : "after ") +
// 			(smooth ? "smooth" : "not smooth"));
//   ui.bound_verOut->display(ubverified);
//   ui.bound_unverOut->display(ubunverified);
//   ui.aminOut->display(amin);
//   ui.pwm_scaleOut->display(pwm_accel_scale);
//   ui.hover_throttleOut->display(hover_throttle);
//   ui.smooth_lookaheadOut->display((int) smooth_lookahead);
// }

void UASControlWidget::cycleContextButton()
{
    UAS* mav = dynamic_cast<UAS*>(UASManager::instance()->getUASForId(this->m_uas));
    if (mav)
    {

        if (!m_engineOn)
        {
            mav->armSystem();
            ui.lastActionLabel->setText(QString("Enabled motors on %1").arg(mav->getUASName()));
        } else {
            mav->disarmSystem();
            ui.lastActionLabel->setText(QString("Disabled motors on %1").arg(mav->getUASName()));
        }
        // Update state now and in several intervals when MAV might have changed state
        updateStatemachine();

        QTimer::singleShot(50, this, SLOT(updateStatemachine()));
        QTimer::singleShot(200, this, SLOT(updateStatemachine()));

    }

}
