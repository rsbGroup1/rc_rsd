/********************************************************************************
** Form generated from reading UI file 'HMIWidget130463.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef HMIWIDGET130463_H
#define HMIWIDGET130463_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HMIWidgetClass
{
public:
    QGroupBox *groupBox;
    QLabel *_labelCameraView;
    QLabel *label_6;
    QGroupBox *groupBox_2;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *_labelOrderStatus;
    QFrame *line;
    QLabel *label_5;
    QWidget *horizontalLayoutWidget_5;
    QHBoxLayout *horizontalLayout_5;
    QRadioButton *_cbAuto;
    QRadioButton *_cbManual;
    QPushButton *_btnEmStop;
    QPushButton *_btnCellReady;
    QPushButton *_btnCellBusy;
    QPushButton *_btnCellError;
    QLabel *label_17;
    QFrame *line_3;
    QLabel *LocalPlanner;
    QFrame *line_4;
    QLabel *label_18;
    QTextBrowser *_txtBrowser;
    QPushButton *_btnStartConv;
    QPushButton *_btnMoveConv;
    QPushButton *_btnStopConv;
    QLabel *label;

    void setupUi(QWidget *HMIWidgetClass)
    {
        if (HMIWidgetClass->objectName().isEmpty())
            HMIWidgetClass->setObjectName(QString::fromUtf8("HMIWidgetClass"));
        HMIWidgetClass->resize(373, 935);
        groupBox = new QGroupBox(HMIWidgetClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 660, 351, 271));
        groupBox->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        _labelCameraView = new QLabel(groupBox);
        _labelCameraView->setObjectName(QString::fromUtf8("_labelCameraView"));
        _labelCameraView->setGeometry(QRect(10, 40, 331, 221));
        _labelCameraView->setAlignment(Qt::AlignCenter);
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 10, 331, 19));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignCenter);
        groupBox_2 = new QGroupBox(HMIWidgetClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 40, 351, 601));
        groupBox_2->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(10, 140, 331, 19));
        label_14->setFont(font);
        label_14->setAlignment(Qt::AlignCenter);
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 170, 101, 17));
        label_15->setFont(font);
        _labelOrderStatus = new QLabel(groupBox_2);
        _labelOrderStatus->setObjectName(QString::fromUtf8("_labelOrderStatus"));
        _labelOrderStatus->setGeometry(QRect(110, 170, 231, 17));
        line = new QFrame(groupBox_2);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(10, 120, 331, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 260, 331, 19));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignCenter);
        horizontalLayoutWidget_5 = new QWidget(groupBox_2);
        horizontalLayoutWidget_5->setObjectName(QString::fromUtf8("horizontalLayoutWidget_5"));
        horizontalLayoutWidget_5->setGeometry(QRect(10, 40, 331, 31));
        horizontalLayout_5 = new QHBoxLayout(horizontalLayoutWidget_5);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        _cbAuto = new QRadioButton(horizontalLayoutWidget_5);
        _cbAuto->setObjectName(QString::fromUtf8("_cbAuto"));
        _cbAuto->setFocusPolicy(Qt::NoFocus);
        _cbAuto->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
"	border: 1px solid darkgray; \n"
"	border-radius: 6px; \n"
"	background-color: white; \n"
"	width: 15px; \n"
"	height: 15px; \n"
"	margin-left: 5px;\n"
"}\n"
"\n"
"QRadioButton::indicator::checked { \n"
"	border: 1px solid darkgray; \n"
"	border-radius: 6px; \n"
"	background-color: #003399; \n"
"	width: 15px; \n"
"	height: 15px; \n"
"	margin-left: 5px;\n"
"}"));
        _cbAuto->setChecked(false);

        horizontalLayout_5->addWidget(_cbAuto);

        _cbManual = new QRadioButton(horizontalLayoutWidget_5);
        _cbManual->setObjectName(QString::fromUtf8("_cbManual"));
        _cbManual->setEnabled(true);
        _cbManual->setFocusPolicy(Qt::NoFocus);
        _cbManual->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
"	border: 1px solid darkgray; \n"
"	border-radius: 6px; \n"
"	background-color: white; \n"
"	width: 15px; \n"
"	height: 15px; \n"
"	margin-left: 5px;\n"
"}\n"
"\n"
"QRadioButton::indicator::checked { \n"
"	border: 1px solid darkgray; \n"
"	border-radius: 6px; \n"
"	background-color: #003399; \n"
"	width: 15px; \n"
"	height: 15px; \n"
"	margin-left: 5px;\n"
"}"));
        _cbManual->setChecked(true);

        horizontalLayout_5->addWidget(_cbManual);

        _btnEmStop = new QPushButton(horizontalLayoutWidget_5);
        _btnEmStop->setObjectName(QString::fromUtf8("_btnEmStop"));
        QFont font1;
        font1.setBold(false);
        font1.setWeight(50);
        _btnEmStop->setFont(font1);
        _btnEmStop->setFocusPolicy(Qt::NoFocus);
        _btnEmStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: red;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));

        horizontalLayout_5->addWidget(_btnEmStop);

        _btnCellReady = new QPushButton(groupBox_2);
        _btnCellReady->setObjectName(QString::fromUtf8("_btnCellReady"));
        _btnCellReady->setGeometry(QRect(50, 200, 92, 31));
        _btnCellReady->setMinimumSize(QSize(92, 0));
        _btnCellReady->setFocusPolicy(Qt::NoFocus);
        _btnCellReady->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));
        _btnCellBusy = new QPushButton(groupBox_2);
        _btnCellBusy->setObjectName(QString::fromUtf8("_btnCellBusy"));
        _btnCellBusy->setGeometry(QRect(150, 200, 92, 31));
        _btnCellBusy->setMinimumSize(QSize(92, 0));
        _btnCellBusy->setFocusPolicy(Qt::NoFocus);
        _btnCellBusy->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
""));
        _btnCellError = new QPushButton(groupBox_2);
        _btnCellError->setObjectName(QString::fromUtf8("_btnCellError"));
        _btnCellError->setGeometry(QRect(250, 200, 92, 31));
        _btnCellError->setMinimumSize(QSize(92, 0));
        _btnCellError->setFocusPolicy(Qt::NoFocus);
        _btnCellError->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));
        label_17 = new QLabel(groupBox_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 200, 61, 31));
        label_17->setFont(font);
        line_3 = new QFrame(groupBox_2);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(10, 240, 331, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        LocalPlanner = new QLabel(groupBox_2);
        LocalPlanner->setObjectName(QString::fromUtf8("LocalPlanner"));
        LocalPlanner->setGeometry(QRect(10, 10, 331, 22));
        LocalPlanner->setFont(font);
        LocalPlanner->setAlignment(Qt::AlignCenter);
        line_4 = new QFrame(groupBox_2);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(10, 390, 331, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        label_18 = new QLabel(groupBox_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 410, 331, 19));
        label_18->setFont(font);
        label_18->setAlignment(Qt::AlignCenter);
        _txtBrowser = new QTextBrowser(groupBox_2);
        _txtBrowser->setObjectName(QString::fromUtf8("_txtBrowser"));
        _txtBrowser->setGeometry(QRect(10, 440, 331, 151));
        _btnStartConv = new QPushButton(groupBox_2);
        _btnStartConv->setObjectName(QString::fromUtf8("_btnStartConv"));
        _btnStartConv->setGeometry(QRect(10, 80, 101, 31));
        _btnStartConv->setMinimumSize(QSize(92, 0));
        _btnStartConv->setFocusPolicy(Qt::NoFocus);
        _btnStartConv->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));
        _btnMoveConv = new QPushButton(groupBox_2);
        _btnMoveConv->setObjectName(QString::fromUtf8("_btnMoveConv"));
        _btnMoveConv->setGeometry(QRect(240, 80, 101, 31));
        _btnMoveConv->setMinimumSize(QSize(92, 0));
        _btnMoveConv->setFocusPolicy(Qt::NoFocus);
        _btnMoveConv->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));
        _btnStopConv = new QPushButton(groupBox_2);
        _btnStopConv->setObjectName(QString::fromUtf8("_btnStopConv"));
        _btnStopConv->setGeometry(QRect(120, 80, 111, 31));
        _btnStopConv->setMinimumSize(QSize(92, 0));
        _btnStopConv->setFocusPolicy(Qt::NoFocus);
        _btnStopConv->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 1px solid #003399;\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 10.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background: qradialgradient(cx: 0.4, cy: -0.1,\n"
"fx: 0.9, fy: -0.1,\n"
"radius: 5.35, stop: 0 #fff, stop: 1 #003399);\n"
"}\n"
""));
        _btnCellReady->raise();
        label_14->raise();
        label_15->raise();
        _labelOrderStatus->raise();
        line->raise();
        label_5->raise();
        horizontalLayoutWidget_5->raise();
        _btnCellBusy->raise();
        _btnCellError->raise();
        label_17->raise();
        line_3->raise();
        LocalPlanner->raise();
        line_4->raise();
        label_18->raise();
        _txtBrowser->raise();
        _btnStartConv->raise();
        _btnMoveConv->raise();
        _btnStopConv->raise();
        label = new QLabel(HMIWidgetClass);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 0, 351, 41));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Purisa"));
        font2.setPointSize(21);
        font2.setBold(false);
        font2.setItalic(true);
        font2.setWeight(50);
        label->setFont(font2);
        label->setStyleSheet(QString::fromUtf8("color:#003399;"));
        label->setAlignment(Qt::AlignCenter);

        retranslateUi(HMIWidgetClass);

        QMetaObject::connectSlotsByName(HMIWidgetClass);
    } // setupUi

    void retranslateUi(QWidget *HMIWidgetClass)
    {
        HMIWidgetClass->setWindowTitle(QApplication::translate("HMIWidgetClass", "HMIWidget", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QString());
        _labelCameraView->setText(QApplication::translate("HMIWidgetClass", "CameraView", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("HMIWidgetClass", "Camera", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QString());
        label_14->setText(QApplication::translate("HMIWidgetClass", "MES Server Communication", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("HMIWidgetClass", "Order status", 0, QApplication::UnicodeUTF8));
        _labelOrderStatus->setText(QApplication::translate("HMIWidgetClass", "5 red, 4 black", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("HMIWidgetClass", "Parameter tweak", 0, QApplication::UnicodeUTF8));
        _cbAuto->setText(QApplication::translate("HMIWidgetClass", "Auto", 0, QApplication::UnicodeUTF8));
        _cbManual->setText(QApplication::translate("HMIWidgetClass", "Manual", 0, QApplication::UnicodeUTF8));
        _btnEmStop->setText(QApplication::translate("HMIWidgetClass", "EMER. STOP", 0, QApplication::UnicodeUTF8));
        _btnCellReady->setText(QApplication::translate("HMIWidgetClass", "Cell Ready", 0, QApplication::UnicodeUTF8));
        _btnCellBusy->setText(QApplication::translate("HMIWidgetClass", "Cell Busy", 0, QApplication::UnicodeUTF8));
        _btnCellError->setText(QApplication::translate("HMIWidgetClass", "Cell Error", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("HMIWidgetClass", "MES", 0, QApplication::UnicodeUTF8));
        LocalPlanner->setText(QApplication::translate("HMIWidgetClass", "Control Mode", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("HMIWidgetClass", "Log messages", 0, QApplication::UnicodeUTF8));
        _btnStartConv->setText(QApplication::translate("HMIWidgetClass", "Start conv", 0, QApplication::UnicodeUTF8));
        _btnMoveConv->setText(QApplication::translate("HMIWidgetClass", "Move conv", 0, QApplication::UnicodeUTF8));
        _btnStopConv->setText(QApplication::translate("HMIWidgetClass", "Stop conv", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("HMIWidgetClass", "RSD HMI", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class HMIWidgetClass: public Ui_HMIWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // HMIWIDGET130463_H

