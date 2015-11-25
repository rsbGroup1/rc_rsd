/********************************************************************************
** Form generated from reading UI file 'HMIWidgetW24784.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef HMIWIDGETW24784_H
#define HMIWIDGETW24784_H

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
    QLabel *label_2;
    QSlider *_sliderSMin;
    QLabel *_labelSMin;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QSlider *_sliderSMax;
    QLabel *_labelSMax;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QSlider *_sliderVMin;
    QLabel *_labelVMin;
    QWidget *horizontalLayoutWidget_4;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_8;
    QSlider *_sliderVMax;
    QLabel *_labelVMax;
    QWidget *horizontalLayoutWidget_7;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_10;
    QSlider *_sliderClose;
    QLabel *_labelClose;
    QWidget *horizontalLayoutWidget_9;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_12;
    QSlider *_sliderBlob;
    QLabel *_labelBlob;
    QFrame *line_2;
    QPushButton *_btnEmStop;
    QLabel *label_20;
    QPushButton *_btnGripperOpen;
    QPushButton *_btnGripperClose;
    QFrame *line_7;
    QWidget *horizontalLayoutWidget_10;
    QHBoxLayout *horizontalLayout_10;
    QRadioButton *_cbLive;
    QRadioButton *_cbVision;
    QGroupBox *groupBox_2;
    QLabel *label_15;
    QLabel *_labelOrderStatus;
    QLabel *label_5;
    QWidget *horizontalLayoutWidget_5;
    QHBoxLayout *horizontalLayout_5;
    QRadioButton *_cbAuto;
    QRadioButton *_cbManual;
    QRadioButton *_cbJog;
    QPushButton *_btnCellDone;
    QPushButton *_btnMrOk;
    QPushButton *_btnClearOrder;
    QFrame *line_3;
    QLabel *LocalPlanner;
    QFrame *line_4;
    QLabel *label_18;
    QTextBrowser *_txtBrowser;
    QLabel *label_7;
    QLabel *_labelSafety;
    QLabel *_labelBricks;
    QPushButton *_btnClearLog;
    QFrame *line_5;
    QLabel *label_16;
    QLabel *label_17;
    QPushButton *_btnConvStartF;
    QPushButton *_btnConvStop;
    QFrame *line_6;
    QLabel *label_19;
    QPushButton *_btnRobotHome;
    QPushButton *_btnRobotReady;
    QPushButton *_btnRobotStop;
    QPushButton *_btnConvStartR;
    QLabel *label;

    void setupUi(QWidget *HMIWidgetClass)
    {
        if (HMIWidgetClass->objectName().isEmpty())
            HMIWidgetClass->setObjectName(QString::fromUtf8("HMIWidgetClass"));
        HMIWidgetClass->resize(739, 791);
        groupBox = new QGroupBox(HMIWidgetClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(380, 60, 351, 721));
        groupBox->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        _labelCameraView = new QLabel(groupBox);
        _labelCameraView->setObjectName(QString::fromUtf8("_labelCameraView"));
        _labelCameraView->setGeometry(QRect(10, 260, 331, 221));
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
        horizontalLayoutWidget->setGeometry(QRect(10, 40, 331, 31));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(horizontalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        _sliderSMin = new QSlider(horizontalLayoutWidget);
        _sliderSMin->setObjectName(QString::fromUtf8("_sliderSMin"));
        _sliderSMin->setEnabled(true);
        _sliderSMin->setFocusPolicy(Qt::NoFocus);
        _sliderSMin->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderSMin->setMaximum(255);
        _sliderSMin->setValue(0);
        _sliderSMin->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(_sliderSMin);

        _labelSMin = new QLabel(horizontalLayoutWidget);
        _labelSMin->setObjectName(QString::fromUtf8("_labelSMin"));

        horizontalLayout->addWidget(_labelSMin);

        horizontalLayoutWidget_2 = new QWidget(groupBox);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 70, 331, 31));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(horizontalLayoutWidget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_2->addWidget(label_3);

        _sliderSMax = new QSlider(horizontalLayoutWidget_2);
        _sliderSMax->setObjectName(QString::fromUtf8("_sliderSMax"));
        _sliderSMax->setFocusPolicy(Qt::NoFocus);
        _sliderSMax->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderSMax->setMaximum(255);
        _sliderSMax->setValue(255);
        _sliderSMax->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(_sliderSMax);

        _labelSMax = new QLabel(horizontalLayoutWidget_2);
        _labelSMax->setObjectName(QString::fromUtf8("_labelSMax"));

        horizontalLayout_2->addWidget(_labelSMax);

        horizontalLayoutWidget_3 = new QWidget(groupBox);
        horizontalLayoutWidget_3->setObjectName(QString::fromUtf8("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(10, 100, 331, 31));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(horizontalLayoutWidget_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_3->addWidget(label_4);

        _sliderVMin = new QSlider(horizontalLayoutWidget_3);
        _sliderVMin->setObjectName(QString::fromUtf8("_sliderVMin"));
        _sliderVMin->setFocusPolicy(Qt::NoFocus);
        _sliderVMin->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderVMin->setMaximum(255);
        _sliderVMin->setValue(50);
        _sliderVMin->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(_sliderVMin);

        _labelVMin = new QLabel(horizontalLayoutWidget_3);
        _labelVMin->setObjectName(QString::fromUtf8("_labelVMin"));

        horizontalLayout_3->addWidget(_labelVMin);

        horizontalLayoutWidget_4 = new QWidget(groupBox);
        horizontalLayoutWidget_4->setObjectName(QString::fromUtf8("horizontalLayoutWidget_4"));
        horizontalLayoutWidget_4->setGeometry(QRect(10, 130, 331, 31));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget_4);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(horizontalLayoutWidget_4);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_4->addWidget(label_8);

        _sliderVMax = new QSlider(horizontalLayoutWidget_4);
        _sliderVMax->setObjectName(QString::fromUtf8("_sliderVMax"));
        _sliderVMax->setFocusPolicy(Qt::NoFocus);
        _sliderVMax->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderVMax->setMaximum(255);
        _sliderVMax->setValue(255);
        _sliderVMax->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(_sliderVMax);

        _labelVMax = new QLabel(horizontalLayoutWidget_4);
        _labelVMax->setObjectName(QString::fromUtf8("_labelVMax"));

        horizontalLayout_4->addWidget(_labelVMax);

        horizontalLayoutWidget_7 = new QWidget(groupBox);
        horizontalLayoutWidget_7->setObjectName(QString::fromUtf8("horizontalLayoutWidget_7"));
        horizontalLayoutWidget_7->setGeometry(QRect(10, 190, 331, 31));
        horizontalLayout_7 = new QHBoxLayout(horizontalLayoutWidget_7);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        label_10 = new QLabel(horizontalLayoutWidget_7);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_7->addWidget(label_10);

        _sliderClose = new QSlider(horizontalLayoutWidget_7);
        _sliderClose->setObjectName(QString::fromUtf8("_sliderClose"));
        _sliderClose->setFocusPolicy(Qt::NoFocus);
        _sliderClose->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderClose->setMaximum(31);
        _sliderClose->setSingleStep(2);
        _sliderClose->setPageStep(4);
        _sliderClose->setValue(5);
        _sliderClose->setOrientation(Qt::Horizontal);

        horizontalLayout_7->addWidget(_sliderClose);

        _labelClose = new QLabel(horizontalLayoutWidget_7);
        _labelClose->setObjectName(QString::fromUtf8("_labelClose"));

        horizontalLayout_7->addWidget(_labelClose);

        horizontalLayoutWidget_9 = new QWidget(groupBox);
        horizontalLayoutWidget_9->setObjectName(QString::fromUtf8("horizontalLayoutWidget_9"));
        horizontalLayoutWidget_9->setGeometry(QRect(10, 160, 331, 31));
        horizontalLayout_9 = new QHBoxLayout(horizontalLayoutWidget_9);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        horizontalLayout_9->setContentsMargins(0, 0, 0, 0);
        label_12 = new QLabel(horizontalLayoutWidget_9);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_9->addWidget(label_12);

        _sliderBlob = new QSlider(horizontalLayoutWidget_9);
        _sliderBlob->setObjectName(QString::fromUtf8("_sliderBlob"));
        _sliderBlob->setFocusPolicy(Qt::NoFocus);
        _sliderBlob->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"border: 1px solid #bbb;\n"
"background: #003399;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"background: #003399;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"background: #fff;\n"
"border: 1px solid #777;\n"
"height: 10px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #eee, stop:1 #ccc);\n"
"border: 1px solid #777;\n"
"width: 13px;\n"
"margin-top: -2px;\n"
"margin-bottom: -2px;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"background: qlineargradient(x1:0, y1:0, x2:1, y2:1,\n"
"    stop:0 #fff, stop:1 #ddd);\n"
"border: 1px solid #444;\n"
"border-radius: 4px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal:disabled {\n"
"background: #bbb;\n"
"border-color: #999;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal:disabled {\n"
"background: #eee;\n"
""
                        "border-color: #999;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:disabled {\n"
"background: #eee;\n"
"border: 1px solid #aaa;\n"
"border-radius: 4px;\n"
"}"));
        _sliderBlob->setMaximum(20000);
        _sliderBlob->setSingleStep(50);
        _sliderBlob->setPageStep(100);
        _sliderBlob->setValue(2500);
        _sliderBlob->setOrientation(Qt::Horizontal);

        horizontalLayout_9->addWidget(_sliderBlob);

        _labelBlob = new QLabel(horizontalLayoutWidget_9);
        _labelBlob->setObjectName(QString::fromUtf8("_labelBlob"));

        horizontalLayout_9->addWidget(_labelBlob);

        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 550, 331, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        _btnEmStop = new QPushButton(groupBox);
        _btnEmStop->setObjectName(QString::fromUtf8("_btnEmStop"));
        _btnEmStop->setGeometry(QRect(15, 668, 321, 41));
        QFont font1;
        font1.setPointSize(17);
        font1.setBold(true);
        font1.setWeight(75);
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
        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(10, 570, 331, 19));
        label_20->setFont(font);
        label_20->setAlignment(Qt::AlignCenter);
        _btnGripperOpen = new QPushButton(groupBox);
        _btnGripperOpen->setObjectName(QString::fromUtf8("_btnGripperOpen"));
        _btnGripperOpen->setGeometry(QRect(20, 600, 141, 31));
        _btnGripperOpen->setMinimumSize(QSize(92, 0));
        _btnGripperOpen->setFocusPolicy(Qt::NoFocus);
        _btnGripperOpen->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnGripperClose = new QPushButton(groupBox);
        _btnGripperClose->setObjectName(QString::fromUtf8("_btnGripperClose"));
        _btnGripperClose->setGeometry(QRect(190, 600, 141, 31));
        _btnGripperClose->setMinimumSize(QSize(92, 0));
        _btnGripperClose->setFocusPolicy(Qt::NoFocus);
        _btnGripperClose->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        line_7 = new QFrame(groupBox);
        line_7->setObjectName(QString::fromUtf8("line_7"));
        line_7->setGeometry(QRect(10, 640, 331, 20));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);
        horizontalLayoutWidget_10 = new QWidget(groupBox);
        horizontalLayoutWidget_10->setObjectName(QString::fromUtf8("horizontalLayoutWidget_10"));
        horizontalLayoutWidget_10->setGeometry(QRect(10, 520, 331, 31));
        horizontalLayout_10 = new QHBoxLayout(horizontalLayoutWidget_10);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        horizontalLayout_10->setContentsMargins(0, 0, 0, 0);
        _cbLive = new QRadioButton(horizontalLayoutWidget_10);
        _cbLive->setObjectName(QString::fromUtf8("_cbLive"));
        _cbLive->setFocusPolicy(Qt::NoFocus);
        _cbLive->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
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
        _cbLive->setChecked(false);

        horizontalLayout_10->addWidget(_cbLive);

        _cbVision = new QRadioButton(horizontalLayoutWidget_10);
        _cbVision->setObjectName(QString::fromUtf8("_cbVision"));
        _cbVision->setEnabled(true);
        _cbVision->setFocusPolicy(Qt::NoFocus);
        _cbVision->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
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
        _cbVision->setChecked(true);

        horizontalLayout_10->addWidget(_cbVision);

        groupBox_2 = new QGroupBox(HMIWidgetClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 60, 351, 721));
        groupBox_2->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: 2px solid #003399;\n"
"    border-radius: 5px;\n"
"}"));
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 90, 101, 17));
        label_15->setFont(font);
        _labelOrderStatus = new QLabel(groupBox_2);
        _labelOrderStatus->setObjectName(QString::fromUtf8("_labelOrderStatus"));
        _labelOrderStatus->setGeometry(QRect(120, 90, 211, 17));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 110, 101, 19));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
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

        _cbJog = new QRadioButton(horizontalLayoutWidget_5);
        _cbJog->setObjectName(QString::fromUtf8("_cbJog"));
        _cbJog->setFocusPolicy(Qt::NoFocus);
        _cbJog->setStyleSheet(QString::fromUtf8("QRadioButton::indicator::unchecked { \n"
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

        horizontalLayout_5->addWidget(_cbJog);

        _btnCellDone = new QPushButton(groupBox_2);
        _btnCellDone->setObjectName(QString::fromUtf8("_btnCellDone"));
        _btnCellDone->setGeometry(QRect(10, 210, 101, 31));
        _btnCellDone->setMinimumSize(QSize(92, 0));
        _btnCellDone->setFocusPolicy(Qt::NoFocus);
        _btnCellDone->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnMrOk = new QPushButton(groupBox_2);
        _btnMrOk->setObjectName(QString::fromUtf8("_btnMrOk"));
        _btnMrOk->setGeometry(QRect(120, 210, 111, 31));
        _btnMrOk->setMinimumSize(QSize(92, 0));
        _btnMrOk->setFocusPolicy(Qt::NoFocus);
        _btnMrOk->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnClearOrder = new QPushButton(groupBox_2);
        _btnClearOrder->setObjectName(QString::fromUtf8("_btnClearOrder"));
        _btnClearOrder->setGeometry(QRect(241, 210, 101, 31));
        _btnClearOrder->setMinimumSize(QSize(92, 0));
        _btnClearOrder->setFocusPolicy(Qt::NoFocus);
        _btnClearOrder->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        line_3 = new QFrame(groupBox_2);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(10, 160, 331, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        LocalPlanner = new QLabel(groupBox_2);
        LocalPlanner->setObjectName(QString::fromUtf8("LocalPlanner"));
        LocalPlanner->setGeometry(QRect(10, 10, 331, 22));
        LocalPlanner->setFont(font);
        LocalPlanner->setAlignment(Qt::AlignCenter);
        line_4 = new QFrame(groupBox_2);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(10, 430, 331, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        label_18 = new QLabel(groupBox_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 450, 331, 31));
        label_18->setFont(font);
        label_18->setAlignment(Qt::AlignCenter);
        _txtBrowser = new QTextBrowser(groupBox_2);
        _txtBrowser->setObjectName(QString::fromUtf8("_txtBrowser"));
        _txtBrowser->setGeometry(QRect(10, 490, 331, 221));
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 130, 121, 19));
        label_7->setFont(font);
        label_7->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        _labelSafety = new QLabel(groupBox_2);
        _labelSafety->setObjectName(QString::fromUtf8("_labelSafety"));
        _labelSafety->setGeometry(QRect(120, 110, 211, 17));
        _labelBricks = new QLabel(groupBox_2);
        _labelBricks->setObjectName(QString::fromUtf8("_labelBricks"));
        _labelBricks->setGeometry(QRect(120, 130, 211, 17));
        _btnClearLog = new QPushButton(groupBox_2);
        _btnClearLog->setObjectName(QString::fromUtf8("_btnClearLog"));
        _btnClearLog->setGeometry(QRect(241, 450, 101, 31));
        _btnClearLog->setMinimumSize(QSize(92, 0));
        _btnClearLog->setFocusPolicy(Qt::NoFocus);
        _btnClearLog->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        line_5 = new QFrame(groupBox_2);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(10, 250, 331, 16));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        label_16 = new QLabel(groupBox_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(10, 180, 331, 19));
        label_16->setFont(font);
        label_16->setAlignment(Qt::AlignCenter);
        label_17 = new QLabel(groupBox_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 360, 331, 19));
        label_17->setFont(font);
        label_17->setAlignment(Qt::AlignCenter);
        _btnConvStartF = new QPushButton(groupBox_2);
        _btnConvStartF->setObjectName(QString::fromUtf8("_btnConvStartF"));
        _btnConvStartF->setGeometry(QRect(10, 390, 101, 31));
        _btnConvStartF->setMinimumSize(QSize(92, 0));
        _btnConvStartF->setFocusPolicy(Qt::NoFocus);
        _btnConvStartF->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnConvStop = new QPushButton(groupBox_2);
        _btnConvStop->setObjectName(QString::fromUtf8("_btnConvStop"));
        _btnConvStop->setGeometry(QRect(240, 390, 101, 31));
        _btnConvStop->setMinimumSize(QSize(92, 0));
        _btnConvStop->setFocusPolicy(Qt::NoFocus);
        _btnConvStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        line_6 = new QFrame(groupBox_2);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setGeometry(QRect(10, 340, 331, 16));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        label_19 = new QLabel(groupBox_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(10, 270, 331, 19));
        label_19->setFont(font);
        label_19->setAlignment(Qt::AlignCenter);
        _btnRobotHome = new QPushButton(groupBox_2);
        _btnRobotHome->setObjectName(QString::fromUtf8("_btnRobotHome"));
        _btnRobotHome->setGeometry(QRect(10, 300, 101, 31));
        _btnRobotHome->setMinimumSize(QSize(92, 0));
        _btnRobotHome->setFocusPolicy(Qt::NoFocus);
        _btnRobotHome->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnRobotReady = new QPushButton(groupBox_2);
        _btnRobotReady->setObjectName(QString::fromUtf8("_btnRobotReady"));
        _btnRobotReady->setGeometry(QRect(120, 300, 111, 31));
        _btnRobotReady->setMinimumSize(QSize(92, 0));
        _btnRobotReady->setFocusPolicy(Qt::NoFocus);
        _btnRobotReady->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnRobotStop = new QPushButton(groupBox_2);
        _btnRobotStop->setObjectName(QString::fromUtf8("_btnRobotStop"));
        _btnRobotStop->setGeometry(QRect(240, 300, 101, 31));
        _btnRobotStop->setMinimumSize(QSize(92, 0));
        _btnRobotStop->setFocusPolicy(Qt::NoFocus);
        _btnRobotStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnConvStartR = new QPushButton(groupBox_2);
        _btnConvStartR->setObjectName(QString::fromUtf8("_btnConvStartR"));
        _btnConvStartR->setGeometry(QRect(120, 390, 111, 31));
        _btnConvStartR->setMinimumSize(QSize(92, 0));
        _btnConvStartR->setFocusPolicy(Qt::NoFocus);
        _btnConvStartR->setStyleSheet(QString::fromUtf8("QPushButton {\n"
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
        _btnCellDone->raise();
        label_15->raise();
        _labelOrderStatus->raise();
        label_5->raise();
        horizontalLayoutWidget_5->raise();
        _btnMrOk->raise();
        _btnClearOrder->raise();
        line_3->raise();
        LocalPlanner->raise();
        line_4->raise();
        label_18->raise();
        _txtBrowser->raise();
        label_7->raise();
        _labelSafety->raise();
        _labelBricks->raise();
        _btnClearLog->raise();
        line_5->raise();
        label_16->raise();
        label_17->raise();
        _btnConvStartF->raise();
        _btnConvStop->raise();
        line_6->raise();
        label_19->raise();
        _btnRobotHome->raise();
        _btnRobotReady->raise();
        _btnRobotStop->raise();
        _btnConvStartR->raise();
        label = new QLabel(HMIWidgetClass);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 721, 41));
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
        label_2->setText(QApplication::translate("HMIWidgetClass", "Saturation min   ", 0, QApplication::UnicodeUTF8));
        _labelSMin->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("HMIWidgetClass", "Saturation max  ", 0, QApplication::UnicodeUTF8));
        _labelSMax->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("HMIWidgetClass", "Value min             ", 0, QApplication::UnicodeUTF8));
        _labelVMin->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("HMIWidgetClass", "Value max            ", 0, QApplication::UnicodeUTF8));
        _labelVMax->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("HMIWidgetClass", "Close kernel size", 0, QApplication::UnicodeUTF8));
        _labelClose->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("HMIWidgetClass", "Min blob size        ", 0, QApplication::UnicodeUTF8));
        _labelBlob->setText(QApplication::translate("HMIWidgetClass", "1000", 0, QApplication::UnicodeUTF8));
        _btnEmStop->setText(QApplication::translate("HMIWidgetClass", "EMERGENCY STOP", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("HMIWidgetClass", "Gripper control", 0, QApplication::UnicodeUTF8));
        _btnGripperOpen->setText(QApplication::translate("HMIWidgetClass", "Open gripper", 0, QApplication::UnicodeUTF8));
        _btnGripperClose->setText(QApplication::translate("HMIWidgetClass", "Close gripper", 0, QApplication::UnicodeUTF8));
        _cbLive->setText(QApplication::translate("HMIWidgetClass", "Live feed ", 0, QApplication::UnicodeUTF8));
        _cbVision->setText(QApplication::translate("HMIWidgetClass", "Vision feed", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QString());
        label_15->setText(QApplication::translate("HMIWidgetClass", "Order status", 0, QApplication::UnicodeUTF8));
        _labelOrderStatus->setText(QApplication::translate("HMIWidgetClass", "No order..", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("HMIWidgetClass", "Safety status", 0, QApplication::UnicodeUTF8));
        _cbAuto->setText(QApplication::translate("HMIWidgetClass", "Auto", 0, QApplication::UnicodeUTF8));
        _cbManual->setText(QApplication::translate("HMIWidgetClass", "Manual", 0, QApplication::UnicodeUTF8));
        _cbJog->setText(QApplication::translate("HMIWidgetClass", "Jog", 0, QApplication::UnicodeUTF8));
        _btnCellDone->setText(QApplication::translate("HMIWidgetClass", "Cell Done", 0, QApplication::UnicodeUTF8));
        _btnMrOk->setText(QApplication::translate("HMIWidgetClass", "MR Ok (Debug)", 0, QApplication::UnicodeUTF8));
        _btnClearOrder->setText(QApplication::translate("HMIWidgetClass", "Clear Order", 0, QApplication::UnicodeUTF8));
        LocalPlanner->setText(QApplication::translate("HMIWidgetClass", "Control mode and status", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("HMIWidgetClass", "Log messages", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("HMIWidgetClass", "Bricks on conv", 0, QApplication::UnicodeUTF8));
        _labelSafety->setText(QApplication::translate("HMIWidgetClass", "Ok", 0, QApplication::UnicodeUTF8));
        _labelBricks->setText(QApplication::translate("HMIWidgetClass", "None", 0, QApplication::UnicodeUTF8));
        _btnClearLog->setText(QApplication::translate("HMIWidgetClass", "Clear", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("HMIWidgetClass", "MES Server feedback", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("HMIWidgetClass", "Conveyer control", 0, QApplication::UnicodeUTF8));
        _btnConvStartF->setText(QApplication::translate("HMIWidgetClass", "Start conv (f)", 0, QApplication::UnicodeUTF8));
        _btnConvStop->setText(QApplication::translate("HMIWidgetClass", "Stop conv", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("HMIWidgetClass", "Robot control", 0, QApplication::UnicodeUTF8));
        _btnRobotHome->setText(QApplication::translate("HMIWidgetClass", "Home robot", 0, QApplication::UnicodeUTF8));
        _btnRobotReady->setText(QApplication::translate("HMIWidgetClass", "Ready robot", 0, QApplication::UnicodeUTF8));
        _btnRobotStop->setText(QApplication::translate("HMIWidgetClass", "Stop robot", 0, QApplication::UnicodeUTF8));
        _btnConvStartR->setText(QApplication::translate("HMIWidgetClass", "Start conv (r)", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("HMIWidgetClass", "RSD HMI", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class HMIWidgetClass: public Ui_HMIWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // HMIWIDGETW24784_H

