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
    connect(_cbAuto, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));
    connect(_cbManual, SIGNAL(toggled(bool)), this, SLOT(eventCb(bool)));

    // Set default text etc.
    _txtBrowser->setText("");
    _labelOrderStatus->setText("Pending order..");
    _cbAuto->setChecked(true);

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

        // Setup ROS arugments
        char** argv = NULL;
        int argc = 0;

        // Init ROS
        ros::init(argc, argv, "RSD_HMI_Node");

        // Setup ROS service clients and topic subscriptions
        _nodeHandle = new ros::NodeHandle;

        // Topic names
        std::string imageSub, kukaService, PG70Service;
        _nodeHandle->param<std::string>("/RC_HMI/HMI/image_sub", imageSub, "/rcCamera/image");
        _nodeHandle->param<std::string>("/RC_HMI/HMI/KukaCmdServiceName", kukaService, "/KukaNode");
        _nodeHandle->param<std::string>("/RC_HMI/HMI/PG70CmdServiceName", PG70Service, "/PG70/PG70");

        _serviceKukaSetConf = _nodeHandle->serviceClient<rc_hmi::setConfiguration>(kukaService + "/SetConfiguration");
        _serviceKukaGetConf = _nodeHandle->serviceClient<rc_hmi::getConfiguration>(kukaService + "/GetConfiguration");
        _serviceKukaStop = _nodeHandle->serviceClient<rc_hmi::stopRobot>(kukaService + "/StopRobot");
        _servicePG70Move = _nodeHandle->serviceClient<rc_hmi::Move>(PG70Service + "/Move");
        _servicePG70Stop = _nodeHandle->serviceClient<rc_hmi::Stop>(PG70Service + "/Stop");

        _itImg = new image_transport::ImageTransport(*_nodeHandle);
        _subImg = _itImg->subscribe(imageSub, 1, &HMIWidget::imageCallback, this);

        // Start new threads
        boost::thread rosSpin(&HMIWidget::startROSThread, this);
    }
}

void HMIWidget::startROSThread()
{
    // ROS Spin (handle callbacks etc)
    ros::spin();
}

void HMIWidget::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Push to queue
    _imageQueue.enqueue(msg);
}

void HMIWidget::imageQueueHandler()
{
    if(_imageQueue.size() > 0)
    {
        // Get from queue
        const sensor_msgs::ImageConstPtr msg = _imageQueue.dequeue();

        // Convert to OpenCV
        cv::Mat cvImg = cv_bridge::toCvShare(msg, "bgr8")->image;

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
        _labelCameraView->setPixmap(qPix);
    }
}

void HMIWidget::stateChangedListener(const rw::kinematics::State &state)
{
    static bool runOnce = false;

    // Update new state
    _state = state;

    // Run once
    if(runOnce)
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
           ROS_ERROR("Failed to call the 'serviceKukaSetConfiguration'");


        // Move gripper
        // Get Q
        /*rw::math::Q qGripper = _devicePG70->getQ(_state);
        rc_hmi::Move moveObj;
        moveObj.request.pos = qGripper[0];
        if(!_servicePG70Move.call(moveObj))
            ROS_ERROR("Failed to call the 'servicePG70Move'");*/
    }

    runOnce = !runOnce;
}

void HMIWidget::eventCb(bool input)
{
    static bool runOnce = false;

    if(runOnce)
    {
        if(_cbAuto->isChecked())
        {
            writeToLog("Switched to auto mode!");
        }
        else if(_cbManual->isChecked())
        {
            writeToLog("Switched to manual mode!");
        }
    }

    runOnce = !runOnce;
}

void HMIWidget::eventBtn()
{
    QObject *obj = sender();

    if(obj == _btnCellReady)
    {
        writeToLog("Cell Ready");
        if(msgBoxHelper("The robot will move.","Do you want to continue?"))
        {
            rw::math::Q q(6,0,0,0,0,0,0);
            _deviceKuka->setQ(q, _state);
            _rws->setState(_state);
        }
    }
    if(obj == _btnCellBusy)
    {
        writeToLog("Cell Busy");
        if(msgBoxHelper("The robot will move.","Do you want to continue?"))
        {
            rw::math::Q q(6,0,0,-45*DEGREETORAD,0,-45*DEGREETORAD,0);
            _deviceKuka->setQ(q, _state);
            _rws->setState(_state);
        }
    }
    else if(obj == _btnCellError)
    {
        writeToLog("Cell Error");
        if(msgBoxHelper("The robot will move.","Do you want to continue?"))
        {
            /*rw::math::Q q(6,0,0,-90*DEGREETORAD,0,-90*DEGREETORAD,0);
            _deviceKuka->setQ(q, _state);
            _rws->setState(_state);*/

            // Find Brick Frame
            rw::kinematics::Frame *brickFrame = _rwWorkCell->findFrame("Brick");

            // Find toolcenter Frame
            rw::kinematics::Frame *toolFrame = _rwWorkCell->findFrame("PG70.TCP");

            // Make new invkin object here
            rw::invkin::JacobianIKSolver inverseKin(_deviceKuka, toolFrame, _state);
            inverseKin.setEnableInterpolation(true);
            inverseKin.setCheckJointLimits(true);

            // Load transformations
            rw::math::Transform3D<> _w2brick = rw::kinematics::Kinematics::worldTframe(brickFrame, _state);
            rw::math::Transform3D<> _w2base = _deviceKuka->worldTbase(_state);

            // Calculate idle Q
            rw::math::Transform3D<> posOffset(rw::math::Vector3D<>(0, 0, 0.3));
            rw::math::Transform3D<> w2brickoffset = _w2brick * posOffset;
            rw::math::Transform3D<> transform((inverse(_w2base)*w2brickoffset).P(), rw::math::RPY<>(M_PI, -18*DEGREETORAD, M_PI));

            std::vector<rw::math::Q> qVec = inverseKin.solve(transform, _state);
            if(qVec.empty())
            {
                ROS_ERROR("Error in inverse kinematics!");
                return;
            }

            _deviceKuka->setQ(qVec[0], _state);
            _rws->setState(_state);
        }
    }
    else if(obj == _btnEmStop)
    {
        // Stop robot
        rc_hmi::stopRobot stopObj;
        if(!_serviceKukaStop.call(stopObj))
            ROS_ERROR("Failed to call the 'serviceKukaStopRobot'");

        // Stop gripper
        /*rc_hmi::Stop stopObjPG70;
        if(!_servicePG70Stop.call(stopObjPG70))
            ROS_ERROR("Failed to call the 'servicePG70Stop'");*/
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
