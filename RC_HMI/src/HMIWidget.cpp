// Includes
#include "HMIWidget.hpp"

// Functions
HMIWidget::HMIWidget(QWidget *parent): QWidget(parent)
{
    // Setup UI
	setupUi(this);

    // Connect UI elements
    connect(_btnCellBusy, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnCellError, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnCellReady, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnEmStop, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnConvStartR, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnConvStartF, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnConvStop, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnRobotHome, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnRobotReady, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnRobotStop, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnGripperClose, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnGripperOpen, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_btnClearLog, SIGNAL(released()), this, SLOT(eventBtn()));
    connect(_cbAuto, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));
    connect(_cbManual, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));
    connect(_cbLive, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));
    connect(_cbVision, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));
    connect(_sliderArea, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderBlob, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderClose, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderVMax, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderVMin, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderSMax, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));
    connect(_sliderSMin, SIGNAL(sliderReleased()), this, SLOT(eventSlider()));

    // Set default text etc.
    _txtBrowser->setText("");
    _labelOrderStatus->setText("Pending order..");
    _labelBricks->setText("None");
    _labelBricks->setStyleSheet("QLabel {color : red; }");
    _labelSafety->setText("Ok");
    _labelSafety->setStyleSheet("QLabel {color : green }");
    _cbManual->setChecked(true);
    _safety = false;
    _anyBricks = false;
    _liveFeed = true;
    _manualJog = false;

    // Update labels
    _labelArea->setText(QString::number(_sliderArea->value()));
    _labelBlob->setText(QString::number(_sliderBlob->value()));
    _labelClose->setText(QString::number(_sliderClose->value()));
    _labelVMax->setText(QString::number(_sliderVMax->value()));
    _labelVMin->setText(QString::number(_sliderVMin->value()));
    _labelSMax->setText(QString::number(_sliderSMax->value()));
    _labelSMin->setText(QString::number(_sliderSMin->value()));

    // Init camera timer
    _imageShowTimer = new QTimer(this);
    connect(_imageShowTimer, SIGNAL(timeout()), this, SLOT(imageQueueHandler()));
    _imageShowTimer->start(IMAGE_SHOW_FREQUENCY);
}

HMIWidget::~HMIWidget()
{
    //
}

void HMIWidget::initialize(rw::models::WorkCell::Ptr workcell, rws::RobWorkStudio* rws)
{
    // Run once var
    static bool runOnce = true;

    // Get devices (robot and gripper)
    const std::vector<rw::common::Ptr<rw::models::Device> > &devices = workcell->getDevices();

    if(runOnce && !devices.empty())
    {
        runOnce = false;

         // Load in objects
        _rwWorkCell = workcell;
        _rws = rws;
        _state = _rws->getState();

        // Get device name
        _devicePG70 = dynamic_cast<rw::models::TreeDevice*>(devices[1].get());
        _deviceKuka = dynamic_cast<rw::models::SerialDevice*>(devices[0].get());
        std::cout << "Loaded device " << _deviceKuka->getName() << " and " << _devicePG70->getName() << std::endl;

        // Check if the device is of DoF as desired
        if(_deviceKuka->getQ(_state).size() != 6)
        {
            std::cout << "Device is not a KukaKr6 robot" << std::endl;
            exit(0);
        }

        // Calculate q idle
        rw::math::Q q(6,0,0,0,0,0,0);
        _deviceKuka->setQ(q, _state);

        // Find Brick Frame
        rw::kinematics::Frame *brickFrame = _rwWorkCell->findFrame("Brick");

        // Find toolcenter Frame
        rw::kinematics::Frame *cameraFrame = _rwWorkCell->findFrame("Camera");

        // Make new invkin object here
        rw::invkin::JacobianIKSolver inverseKin(_deviceKuka, cameraFrame, _state);
        inverseKin.setEnableInterpolation(true);
        inverseKin.setCheckJointLimits(true);

        // Load transformations
        rw::math::Transform3D<> _w2brick = rw::kinematics::Kinematics::worldTframe(brickFrame, _state);
        rw::math::Transform3D<> _w2base = _deviceKuka->worldTbase(_state);

        // Calculate idle Q
        rw::math::Transform3D<> posOffset(rw::math::Vector3D<>(0, 0, 0.3));
        rw::math::Transform3D<> w2brickoffset = _w2brick * posOffset;
        rw::math::Transform3D<> transform((inverse(_w2base)*w2brickoffset).P(), rw::math::RPY<>(M_PI, -18*DEGREETORAD, 0));
        std::vector<rw::math::Q> qVec = inverseKin.solve(transform, _state);
        if(qVec.empty())
        {
            _consoleQueue.enqueue("Idle Q: Error in inverse kinematics!");
            return;
        }
        _qIdle = qVec[0];

        // Setup ROS arugments
        char** argv = NULL;
        int argc = 0;

        // Init ROS
        ros::init(argc, argv, "rc_hmi");
        _nodeHandle = new ros::NodeHandle;
        ros::NodeHandle pNh(ros::this_node::getName() + "/");

        // Topic names
        std::string liveImageSub, visionImageSub, kukaService, PG70Service, consoleSub, convService, mesPub, mesSub, anyBricksSub, safetySub, statusPub, visionPub, getBricksService;
        pNh.param<std::string>("live_image_sub", liveImageSub, "/RC_Camera/image_rect_color");
        pNh.param<std::string>("vision_image_sub", visionImageSub, "/rcVision/image");
        pNh.param<std::string>("KukaCmdServiceName", kukaService, "/KukaNode");
        pNh.param<std::string>("PG70CmdServiceName", PG70Service, "/PG70/PG70");
        pNh.param<std::string>("console_sub", consoleSub, "/rcHMI/console");
        pNh.param<std::string>("convServiceName", convService, "/rcPLC");
        pNh.param<std::string>("anyBricks_sub", anyBricksSub, "/rcVision/anyBricks");
        pNh.param<std::string>("getBricks_service", getBricksService, "/rcVision/getBricks");
        pNh.param<std::string>("safety_sub", safetySub, "/KukaNode/GetSafety");
        pNh.param<std::string>("mesPub", mesPub, "/rcMESServer/msgToServer");
        pNh.param<std::string>("mesSub", mesSub, "/rcMESServer/msgFromServer");
        pNh.param<std::string>("visionParamPub", visionPub, "/rcHMI/visionParam");

        // Create service calls
        _serviceKukaSetConf = _nodeHandle->serviceClient<rc_hmi::setConfiguration>(kukaService + "/SetConfiguration");
        _serviceKukaGetConf = _nodeHandle->serviceClient<rc_hmi::getConfiguration>(kukaService + "/GetConfiguration");
        _serviceKukaStop = _nodeHandle->serviceClient<rc_hmi::stopRobot>(kukaService + "/StopRobot");
        _serviceKukaGetSafety = _nodeHandle->serviceClient<rc_hmi::getSafety>(kukaService + "/GetSafety");
        _servicePG70Close = _nodeHandle->serviceClient<rc_hmi::Close>(PG70Service + "/close");
        _servicePG70Stop = _nodeHandle->serviceClient<rc_hmi::Stop>(PG70Service + "/stop");
        _servicePG70Open = _nodeHandle->serviceClient<rc_hmi::Open>(PG70Service + "/open");
        _serviceConvChange = _nodeHandle->serviceClient<rc_hmi::ChangeDirection>(convService + "/ChangeDirection");
        _serviceConvStop = _nodeHandle->serviceClient<rc_hmi::StopConv>(convService + "/StopConv");
        _serviceConvStart = _nodeHandle->serviceClient<rc_hmi::StartConv>(convService + "/StartConv");
        _serviceGetBricks = _nodeHandle->serviceClient<rc_hmi::getBricks>(getBricksService);

        // Publishers
        _mesMessagePub = _nodeHandle->advertise<std_msgs::String>(mesPub, 100);
        _hmiStatusPub = _nodeHandle->advertise<std_msgs::String>(statusPub, 100);
        _visionParamPub = _nodeHandle->advertise<std_msgs::String>(visionPub, 100);

        // Subscribers
        _anyBrickSub = _nodeHandle->subscribe(anyBricksSub, 10, &HMIWidget::anyBrickCallback, this);
        _mesMessageSub = _nodeHandle->subscribe(mesSub, 10, &HMIWidget::mesRecCallback, this);
        _consoleSub = _nodeHandle->subscribe(consoleSub, 100, &HMIWidget::consoleCallback, this);
        _itImg = new image_transport::ImageTransport(*_nodeHandle);
        _subLiveImg = _itImg->subscribe(liveImageSub, 1, &HMIWidget::liveImageCallback, this);
        _subVisionImg = _itImg->subscribe(visionImageSub, 1, &HMIWidget::visionImageCallback, this);

        // Start new threads
        boost::thread rosSpin(&HMIWidget::startROSThread, this);
    }
}

void HMIWidget::startROSThread()
{
    // ROS Spin (handle callbacks etc)
    ros::spin();
}

void HMIWidget::consoleCallback(std_msgs::String msg)
{
    _consoleQueue.enqueue(QString::fromUtf8(msg.data.data(), msg.data.size()));
}

void HMIWidget::liveImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    boost::unique_lock<boost::mutex> lock(_liveFeedMutex);

    // Push to queue
    if(_liveFeed)
        _imageQueue.enqueue(msg);
}

void HMIWidget::visionImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    boost::unique_lock<boost::mutex> lock(_liveFeedMutex);

    // Push to queue
    if(_liveFeed == false)
        _imageQueue.enqueue(msg);
}

void HMIWidget::imageQueueHandler()
{
    if(_imageQueue.size() > 0)
    {
        // Get from queue
        const sensor_msgs::ImageConstPtr msg = _imageQueue.dequeue();

        // Convert to OpenCV
        cv::Mat cvImg = cv_bridge::toCvShare(msg, "rgb8")->image;

        // Determine format
        QImage::Format format;
        if(cvImg.type() == CV_8UC1)     // 8-bits unsigned, NO. OF CHANNELS=1
            format = QImage::Format_Indexed8;
        else if(cvImg.type()==CV_8UC3)  // 8-bits unsigned, NO. OF CHANNELS=3
            format = QImage::Format_RGB888;
        else
        {
            std::cerr << "ERROR: Mat could not be converted to QImage." << std::endl;
            return;
        }

        // Show image in label
        QImage qImg(cvImg.data, cvImg.cols, cvImg.rows, cvImg.step, format);
        QPixmap qPix = QPixmap::fromImage(qImg);
        qPix = qPix.scaled(_labelCameraView->size(), Qt::IgnoreAspectRatio, Qt::FastTransformation);
        _labelCameraView->setPixmap(qPix);
    }

    // Check if anything needs to be written to log
    if(_consoleQueue.size())
        writeToLog(_consoleQueue.dequeue());

    static bool oldAnyBricks = false, oldSafety = false;

    // Update labels
    _labelBricksMutex.lock();
    if(_anyBricks != oldAnyBricks)
    {
        if(_anyBricks)
        {
            _labelBricks->setText("Yes!");
            _labelBricks->setStyleSheet("QLabel {color : green; }");
        }
        else
        {
            _labelBricks->setText("None!");
            _labelBricks->setStyleSheet("QLabel {color : red; }");
        }

        oldAnyBricks = _anyBricks;
    }
    _labelBricksMutex.unlock();

    // Get safety
    rc_hmi::getSafety obj;
    _serviceKukaGetSafety.call(obj);
    _safety = obj.response.safetyBreached;

    // Update label
    if(_safety != oldSafety)
    {
        if(_safety)
        {
            emergencyStop();
            _labelSafety->setText("Not Ok!");
            _labelSafety->setStyleSheet("QLabel {color : red; }");
        }
        else
        {
            _labelSafety->setText("Ok!");
            _labelSafety->setStyleSheet("QLabel {color : green; }");
        }

        oldSafety = _safety;
    }

    // Fetch robot position and update HMI
    // Create setConfiguration service
    rc_hmi::getConfiguration getQObj;

    // Call service
    if(!_serviceKukaGetConf.call(getQObj))
       _consoleQueue.enqueue("Failed to call the 'serviceKukaGetConfiguration'");

    // Get information
    rw::math::Q q(6);
    for(unsigned int i=0; i<6; i++)
       q(i) = getQObj.response.q[i]*DEGREETORAD;

    _manualJog = false;
    _deviceKuka->setQ(q, _state);
    _rws->setState(_state);

    // Get vision
    boost::unique_lock<boost::mutex> lock(_liveFeedMutex);
    if(_liveFeed == false)
    {
        rc_hmi::getBricks obj;
        _serviceGetBricks.call(obj);
    }
}

void HMIWidget::anyBrickCallback(std_msgs::Bool msg)
{
    rw::math::Q qCurrent = _deviceKuka->getQ(_state);
    boost::unique_lock<boost::mutex> lock(_labelBricksMutex);

    if((qCurrent-_qIdle).norm2() < 0.01 && msg.data)
        _anyBricks = true;
    else
        _anyBricks = false;
}

void HMIWidget::mesRecCallback(std_msgs::String msg)
{
    _consoleQueue.enqueue(QString::fromStdString("MES: " + msg.data));
}

void HMIWidget::stateChangedListener(const rw::kinematics::State &state)
{
    static bool runOnce = false;

    // Update new state
    _state = state;

    // Run once
    if(runOnce && _manualJog)
    {
        // Move robot
        // Get Q
        rw::math::Q qRobot = _deviceKuka->getQ(_state);

        // Create setConfiguration service
        rc_hmi::setConfiguration setQObj;

        // Fill out information
        for(int i = 0; i<6; i++)
           setQObj.request.q[i] = qRobot(i);

        // Call service
        if(!_serviceKukaSetConf.call(setQObj))
           _consoleQueue.enqueue("Failed to call the 'serviceKukaSetConfiguration'");

        _manualJog = false;
    }

    runOnce = !runOnce;
}

void HMIWidget::eventSlider()
{
    QObject *obj = sender();

    std_msgs::String msg;

    if(obj == _sliderArea)
    {
        _labelArea->setText(QString::number(_sliderArea->value()));
        msg.data = "area|" + SSTR(_sliderArea->value());
    }
    else if(obj == _sliderBlob)
    {
        _labelBlob->setText(QString::number(_sliderBlob->value()));
        msg.data = "blob|" + SSTR(_sliderBlob->value());
    }
    else if(obj == _sliderClose)
    {
        _labelClose->setText(QString::number(_sliderClose->value()));
        msg.data = "close|" + SSTR(_sliderClose->value());
    }
    else if(obj == _sliderVMax)
    {
        _labelVMax->setText(QString::number(_sliderVMax->value()));
        msg.data = "vmax|" + SSTR(_sliderVMax->value());
    }
    else if(obj == _sliderVMin)
    {
        _labelVMin->setText(QString::number(_sliderVMin->value()));
        msg.data = "vmin|" + SSTR(_sliderVMin->value());
    }
    else if(obj == _sliderSMax)
    {
        _labelSMax->setText(QString::number(_sliderSMax->value()));
        msg.data = "smax|" + SSTR(_sliderSMax->value());
    }
    else if(obj == _sliderSMin)
    {
        _labelSMin->setText(QString::number(_sliderSMin->value()));
        msg.data = "smin|" + SSTR(_sliderSMin->value());
    }

    _visionParamPub.publish(msg);
}

void HMIWidget::eventCb(bool input)
{
    static bool runOnce = false;

    if(runOnce)
    {
        QObject *obj = sender();

        if(obj == _cbAuto || obj == _cbManual)
        {
            std_msgs::String msg;

            if(_cbAuto->isChecked())
            {
                // Ready robot
                _manualJog = true;
                _deviceKuka->setQ(_qIdle, _state);
                _rws->setState(_state);

                msg.data = "start";

                _btnConvStartR->setEnabled(false);
                _btnConvStartF->setEnabled(false);
                _btnConvStop->setEnabled(false);
                _btnRobotHome->setEnabled(false);
                _btnRobotReady->setEnabled(false);
                _btnRobotStop->setEnabled(false);
                _btnGripperClose->setEnabled(false);
                _btnGripperOpen->setEnabled(false);
            }
            else if(_cbManual->isChecked())
            {
                msg.data = "stop";

                _btnConvStartF->setEnabled(true);
                _btnConvStartR->setEnabled(true);
                _btnConvStop->setEnabled(true);
                _btnRobotHome->setEnabled(true);
                _btnRobotReady->setEnabled(true);
                _btnRobotStop->setEnabled(true);
                _btnGripperClose->setEnabled(true);
                _btnGripperOpen->setEnabled(true);

                emergencyStop();
            }

            _hmiStatusPub.publish(msg);
        }
        else if(obj == _cbLive || obj == _cbVision)
        {
            boost::unique_lock<boost::mutex> lock(_liveFeedMutex);

            if(_cbLive->isChecked())
                _liveFeed = true;
            else if(_cbVision->isChecked())
                _liveFeed = false;
        }
    }

    runOnce = !runOnce;
}

void HMIWidget::emergencyStop()
{
    // Stop robot
    rc_hmi::stopRobot stopObj;
    if(!_serviceKukaStop.call(stopObj))
        _consoleQueue.enqueue("Failed to call the 'serviceKukaStopRobot'");

    // Stop gripper
    rc_hmi::Stop stopObjPG70;
    if(!_servicePG70Stop.call(stopObjPG70))
        _consoleQueue.enqueue("Failed to call the 'servicePG70Stop'");

    // Stop conveyer
    rc_hmi::StopConv obj;
    if(!_serviceConvStop.call(obj))
        _consoleQueue.enqueue("Failed to call the 'serviceStopConveyer'");
}

void HMIWidget::eventBtn()
{
    QObject *obj = sender();

    if(obj == _btnCellReady)
    {
        _consoleQueue.enqueue("HMI: Cell Ready!");
        std_msgs::String msg;
        msg.data = "ready";
        _mesMessagePub.publish(msg);
    }
    if(obj == _btnCellBusy)
    {
        _consoleQueue.enqueue("HMI: Cell Busy!");
        std_msgs::String msg;
        msg.data = "busy";
        _mesMessagePub.publish(msg);
    }
    else if(obj == _btnCellError)
    {
        _consoleQueue.enqueue("HMI: Cell Error!");
        std_msgs::String msg;
        msg.data = "error";
        _mesMessagePub.publish(msg);
    }
    else if(obj == _btnEmStop)
    {
        emergencyStop();

        _cbAuto->setChecked(false);
        _cbManual->setChecked(true);
    }
    else if(obj == _btnClearLog)
    {
        _txtBrowser->setText("");
    }

    if(_cbManual->isChecked())
    {
        if(obj == _btnConvStartR)
        {
            // Move conveyer 1 step forward
            rc_hmi::StartConv obj;
            obj.request.direction = true;
            if(!_serviceConvStart.call(obj))
                _consoleQueue.enqueue("Failed to call the 'serviceStartConveyer'");

        }
        else if(obj == _btnConvStartF)
        {
            // Start conveyer
            rc_hmi::StartConv obj;
            obj.request.direction = false;
            if(!_serviceConvStart.call(obj))
                _consoleQueue.enqueue("Failed to call the 'serviceStartConveyer'");

        }
        else if(obj == _btnConvStop)
        {
            // Stop conveyer
            rc_hmi::StopConv obj;
            if(!_serviceConvStop.call(obj))
                _consoleQueue.enqueue("Failed to call the 'serviceStopConveyer'");
        }

        else if(obj == _btnRobotHome)
        {
            // Home robot
            _manualJog = true;
            rw::math::Q q(6,0,0,0,0,0,0);
            _deviceKuka->setQ(q, _state);
            _rws->setState(_state);
        }
        else if(obj == _btnRobotReady)
        {
            // Ready robot
            _manualJog = true;
            _deviceKuka->setQ(_qIdle, _state);
            _rws->setState(_state);
        }
        else if(obj == _btnRobotStop)
        {
            // Stop robot
            rc_hmi::stopRobot stopObj;
            if(!_serviceKukaStop.call(stopObj))
                _consoleQueue.enqueue("Failed to call the 'serviceKukaStopRobot'");
        }
        else if(obj == _btnGripperClose)
        {
            // Close gripper
            rc_hmi::Close clsoeObj;
            clsoeObj.request.power = 10.0;
            if(!_servicePG70Close.call(clsoeObj))
                _consoleQueue.enqueue("Failed to call the 'servicePG70Close'");
        }
        else if(obj == _btnGripperOpen)
        {
            // Open gripper
            rc_hmi::Open openObj;
            openObj.request.power = 10.0;
            if(!_servicePG70Open.call(openObj))
                _consoleQueue.enqueue("Failed to call the 'servicePG70Open'");
        }
    }
}

void HMIWidget::writeToLog(QString text)
{
    QTime time;
    _txtBrowser->setText(time.currentTime().toString() + ": " + text + "\n" + _txtBrowser->toPlainText());
}

bool HMIWidget::msgBoxHelper(QString text, QString informativeText)
{
    // QT Popup box
    QMessageBox msgBox;
    msgBox.setText(text);
    msgBox.setInformativeText(informativeText);
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Ok);

    switch(msgBox.exec())
    {
        case QMessageBox::Ok:
            return true;

        default:
            return false;
    }
}

bool HMIWidget::openWorkCell()
{
	QMessageBox::information(this, "Open", "Open WorkCell");
	return true;
}

void HMIWidget::callback()
{
	QCoreApplication::processEvents();
}
