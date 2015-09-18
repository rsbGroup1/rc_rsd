/********************************************************************************
** Form generated from reading UI file 'HMIWidgetV16379.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef HMIWIDGETV16379_H
#define HMIWIDGETV16379_H

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
#include <QtGui/QSlider>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HMIWidgetClass
{
public:
    QGroupBox *groupBox;
    QLabel *_labelCameraView;
    QLabel *label_6;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_3;
    QSlider *horizontalSlider;
    QLabel *label_7;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_8;
    QSlider *horizontalSlider_2;
    QLabel *label_9;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_10;
    QSlider *horizontalSlider_3;
    QLabel *label_11;
    QWidget *horizontalLayoutWidget_4;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_12;
    QSlider *horizontalSlider_4;
    QLabel *label_13;
    QFrame *line_2;
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
    QRadioButton *_cbDebug;
    QPushButton *_btnCellReady;
    QPushButton *_btnCellBusy;
    QPushButton *_btnCellError;
    QLabel *label_17;
    QFrame *line_3;
    QLabel *LocalPlanner;
    QFrame *line_4;
    QLabel *label_18;
    QTextBrowser *_txtBrowser;
    QLabel *label;

    void setupUi(QWidget *HMIWidgetClass)
    {
        if (HMIWidgetClass->objectName().isEmpty())
            HMIWidgetClass->setObjectName(QString::fromUtf8("HMIWidgetClass"));
        HMIWidgetClass->resize(373, 933);
        groupBox = new QGroupBox(HMIWidgetClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 570, 351, 361));
        groupBox->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        _labelCameraView = new QLabel(groupBox);
        _labelCameraView->setObjectName(QString::fromUtf8("_labelCameraView"));
        _labelCameraView->setGeometry(QRect(10, 130, 331, 221));
        _labelCameraView->setAlignment(Qt::AlignCenter);
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 10, 331, 19));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignCenter);
        horizontalLayoutWidget = new QWidget(groupBox);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 40, 161, 37));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(horizontalLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout->addWidget(label_3);

        horizontalSlider = new QSlider(horizontalLayoutWidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setFocusPolicy(Qt::NoFocus);
        horizontalSlider->setContextMenuPolicy(Qt::DefaultContextMenu);
        horizontalSlider->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #003399;\n"
"height: 8px;\n"
"background: white;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #003399;\n"
"width: 7px;\n"
"margin: -5px 0; \n"
"border-radius: 2px;\n"
"}"));
        horizontalSlider->setMaximum(255);
        horizontalSlider->setValue(150);
        horizontalSlider->setTracking(false);
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(horizontalSlider);

        label_7 = new QLabel(horizontalLayoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout->addWidget(label_7);

        horizontalLayoutWidget_2 = new QWidget(groupBox);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 80, 161, 31));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(horizontalLayoutWidget_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_2->addWidget(label_8);

        horizontalSlider_2 = new QSlider(horizontalLayoutWidget_2);
        horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
        horizontalSlider_2->setFocusPolicy(Qt::NoFocus);
        horizontalSlider_2->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #003399;\n"
"height: 8px;\n"
"background: white;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #003399;\n"
"width: 7px;\n"
"margin: -5px 0; \n"
"border-radius: 2px;\n"
"}"));
        horizontalSlider_2->setMaximum(255);
        horizontalSlider_2->setValue(150);
        horizontalSlider_2->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(horizontalSlider_2);

        label_9 = new QLabel(horizontalLayoutWidget_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_2->addWidget(label_9);

        horizontalLayoutWidget_3 = new QWidget(groupBox);
        horizontalLayoutWidget_3->setObjectName(QString::fromUtf8("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(180, 40, 161, 31));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_10 = new QLabel(horizontalLayoutWidget_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_3->addWidget(label_10);

        horizontalSlider_3 = new QSlider(horizontalLayoutWidget_3);
        horizontalSlider_3->setObjectName(QString::fromUtf8("horizontalSlider_3"));
        horizontalSlider_3->setFocusPolicy(Qt::NoFocus);
        horizontalSlider_3->setContextMenuPolicy(Qt::DefaultContextMenu);
        horizontalSlider_3->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #003399;\n"
"height: 8px;\n"
"background: white;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #003399;\n"
"width: 7px;\n"
"margin: -5px 0; \n"
"border-radius: 2px;\n"
"}"));
        horizontalSlider_3->setMaximum(255);
        horizontalSlider_3->setValue(150);
        horizontalSlider_3->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(horizontalSlider_3);

        label_11 = new QLabel(horizontalLayoutWidget_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_3->addWidget(label_11);

        horizontalLayoutWidget_4 = new QWidget(groupBox);
        horizontalLayoutWidget_4->setObjectName(QString::fromUtf8("horizontalLayoutWidget_4"));
        horizontalLayoutWidget_4->setGeometry(QRect(180, 80, 161, 31));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget_4);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_12 = new QLabel(horizontalLayoutWidget_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_4->addWidget(label_12);

        horizontalSlider_4 = new QSlider(horizontalLayoutWidget_4);
        horizontalSlider_4->setObjectName(QString::fromUtf8("horizontalSlider_4"));
        horizontalSlider_4->setFocusPolicy(Qt::NoFocus);
        horizontalSlider_4->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #003399;\n"
"height: 8px;\n"
"background: white;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #003399;\n"
"width: 7px;\n"
"margin: -5px 0; \n"
"border-radius: 2px;\n"
"}"));
        horizontalSlider_4->setMaximum(255);
        horizontalSlider_4->setValue(150);
        horizontalSlider_4->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(horizontalSlider_4);

        label_13 = new QLabel(horizontalLayoutWidget_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_4->addWidget(label_13);

        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 110, 331, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        groupBox_2 = new QGroupBox(HMIWidgetClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 40, 351, 511));
        groupBox_2->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(10, 90, 331, 19));
        label_14->setFont(font);
        label_14->setAlignment(Qt::AlignCenter);
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 120, 101, 17));
        label_15->setFont(font);
        _labelOrderStatus = new QLabel(groupBox_2);
        _labelOrderStatus->setObjectName(QString::fromUtf8("_labelOrderStatus"));
        _labelOrderStatus->setGeometry(QRect(110, 120, 231, 17));
        line = new QFrame(groupBox_2);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(10, 70, 331, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 210, 331, 19));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignCenter);
        horizontalLayoutWidget_5 = new QWidget(groupBox_2);
        horizontalLayoutWidget_5->setObjectName(QString::fromUtf8("horizontalLayoutWidget_5"));
        horizontalLayoutWidget_5->setGeometry(QRect(10, 40, 331, 24));
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
        _cbAuto->setChecked(true);

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
        _cbManual->setChecked(false);

        horizontalLayout_5->addWidget(_cbManual);

        _cbDebug = new QRadioButton(horizontalLayoutWidget_5);
        _cbDebug->setObjectName(QString::fromUtf8("_cbDebug"));
        _cbDebug->setFocusPolicy(Qt::NoFocus);
        _cbDebug->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
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

        horizontalLayout_5->addWidget(_cbDebug);

        _btnCellReady = new QPushButton(groupBox_2);
        _btnCellReady->setObjectName(QString::fromUtf8("_btnCellReady"));
        _btnCellReady->setGeometry(QRect(50, 150, 92, 31));
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
        _btnCellBusy->setGeometry(QRect(150, 150, 92, 31));
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
        _btnCellError->setGeometry(QRect(250, 150, 92, 31));
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
        label_17->setGeometry(QRect(10, 150, 61, 31));
        label_17->setFont(font);
        line_3 = new QFrame(groupBox_2);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(10, 190, 331, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        LocalPlanner = new QLabel(groupBox_2);
        LocalPlanner->setObjectName(QString::fromUtf8("LocalPlanner"));
        LocalPlanner->setGeometry(QRect(10, 10, 331, 22));
        LocalPlanner->setFont(font);
        LocalPlanner->setAlignment(Qt::AlignCenter);
        line_4 = new QFrame(groupBox_2);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(10, 350, 331, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        label_18 = new QLabel(groupBox_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 370, 331, 19));
        label_18->setFont(font);
        label_18->setAlignment(Qt::AlignCenter);
        _txtBrowser = new QTextBrowser(groupBox_2);
        _txtBrowser->setObjectName(QString::fromUtf8("_txtBrowser"));
        _txtBrowser->setGeometry(QRect(10, 400, 331, 101));
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
        label = new QLabel(HMIWidgetClass);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 0, 351, 41));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Purisa"));
        font1.setPointSize(21);
        font1.setBold(false);
        font1.setItalic(true);
        font1.setWeight(50);
        label->setFont(font1);
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
        label_3->setText(QApplication::translate("HMIWidgetClass", " Hue   ", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("HMIWidgetClass", "150", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("HMIWidgetClass", " Value", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("HMIWidgetClass", "150", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("HMIWidgetClass", " Sat   ", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("HMIWidgetClass", "150", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("HMIWidgetClass", " Svd  ", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("HMIWidgetClass", "150", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QString());
        label_14->setText(QApplication::translate("HMIWidgetClass", "MES Server Communication", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("HMIWidgetClass", "Order status", 0, QApplication::UnicodeUTF8));
        _labelOrderStatus->setText(QApplication::translate("HMIWidgetClass", "5 red, 4 black", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("HMIWidgetClass", "Parameter tweak", 0, QApplication::UnicodeUTF8));
        _cbAuto->setText(QApplication::translate("HMIWidgetClass", "Auto", 0, QApplication::UnicodeUTF8));
        _cbManual->setText(QApplication::translate("HMIWidgetClass", "Manual", 0, QApplication::UnicodeUTF8));
        _cbDebug->setText(QApplication::translate("HMIWidgetClass", "Debug", 0, QApplication::UnicodeUTF8));
        _btnCellReady->setText(QApplication::translate("HMIWidgetClass", "Cell Ready", 0, QApplication::UnicodeUTF8));
        _btnCellBusy->setText(QApplication::translate("HMIWidgetClass", "Cell Busy", 0, QApplication::UnicodeUTF8));
        _btnCellError->setText(QApplication::translate("HMIWidgetClass", "Cell Error", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("HMIWidgetClass", "MES", 0, QApplication::UnicodeUTF8));
        LocalPlanner->setText(QApplication::translate("HMIWidgetClass", "Control Mode", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("HMIWidgetClass", "Log messages", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("HMIWidgetClass", "RSD HMI", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class HMIWidgetClass: public Ui_HMIWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // HMIWIDGETV16379_H

