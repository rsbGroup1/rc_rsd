#ifndef HMIWIDGET_HPP
#define HMIWIDGET_HPP

// Includes
// RobWork
#include <RobWorkStudio.hpp>
#include <rws/RobWorkStudio.hpp>

#include <rw/math/Constants.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

#include <rw/kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

#include <kuka_rsi/setConfiguration.h>
#include <kuka_rsi/getConfiguration.h>
#include <kuka_rsi/getSafety.h>
#include <kuka_rsi/stopRobot.h>
#include <pg70/Open.h>
#include <pg70/Close.h>
#include <pg70/Stop.h>
#include <rc_plc/ChangeDirection.h>
#include <rc_plc/StartConv.h>
#include <rc_plc/StopConv.h>
#include <rc_vision/getBricks.h>
#include <rc_mes_client/server.h>

#include <rw/models/WorkCell.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/SerialDevice.hpp>

#include <rw/graphics/WorkCellScene.hpp>
#include <rw/graphics/DrawableNode.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <image_transport/image_transport.h>

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv/cv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Boost
#include <boost/thread.hpp>

// QT
#include "ui_HMIWidget.h"
#include <QMessageBox>
#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <QTime>

// Other
#include <queue>

// Defines
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define IMAGE_SHOW_FREQUENCY    (1000/20) // 20 FPS
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Queue class
template <typename T>
class SynchronisedQueue
{
    private:
        std::queue<T> m_queue;              // Use STL queue to store data
        boost::mutex m_mutex;               // The mutex to synchronise on
        boost::condition_variable m_cond;   // The condition to wait for

    public:
        // Add data to the queue and notify others
        void enqueue(const T& data)
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // Add the data to the queue
            m_queue.push(data);

            // Notify others that data is ready
            m_cond.notify_one();
        }

        // Get data from the queue. Wait for data if not available
        T dequeue()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // When there is no data, wait till someone fills it.
            // Lock is automatically released in the wait and obtained
            // again after the wait
            while(m_queue.size()==0)
                m_cond.wait(lock);

            // Retrieve the data from the queue
            T result = m_queue.front();
            m_queue.pop();

            return result;
        }

        int size()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return m_queue.size();
        }
};

// Class
class HMIWidget : public QWidget, private Ui::HMIWidgetClass
{
    	Q_OBJECT

	public:
        // Constructors
        HMIWidget(QWidget *parent = 0);
        ~HMIWidget();

        // Member functions
        void initialize(rw::models::WorkCell::Ptr, rws::RobWorkStudio *);
        void stateChangedListener(const rw::kinematics::State &);

	private slots:
        // Member functions
        void callback();
        void eventBtn();
        void eventCb(bool);
        void imageQueueHandler();
        void eventSlider();

    private:
        // Member functions
        void liveImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void visionImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void anyBrickCallback(std_msgs::Bool msg);
        void mesRecCallback(rc_mes_client::server msg);
        void startROSThread();
        bool openWorkCell();
        bool msgBoxHelper(QString, QString);
        void writeToLog(QString text);
        void consoleCallback(std_msgs::String msg);
        void emergencyStop();

        // Variables
        rw::models::WorkCell::Ptr _rwWorkCell;
        rw::models::SerialDevice *_deviceKuka;
        rw::models::TreeDevice *_devicePG70;
	    rw::kinematics::State _state;
	    rws::RobWorkStudio *_rws;

	    // ROS	
        ros::NodeHandle *_nodeHandle;
        image_transport::ImageTransport *_itImg;
        image_transport::Subscriber _subLiveImg, _subVisionImg;
        ros::ServiceClient _serviceKukaSetConf, _serviceKukaStop, _serviceKukaGetConf, _serviceKukaGetSafety;
        ros::ServiceClient _servicePG70Open, _servicePG70Stop, _servicePG70Close;
        ros::ServiceClient _serviceConvChange, _serviceConvStop, _serviceConvStart;
        ros::ServiceClient _serviceGetBricks;
        ros::Subscriber _consoleSub, _mesMessageSub, _anyBrickSub;
        ros::Publisher _mesMessagePub, _hmiStatusPub, _visionParamPub;

        // Image
        QTimer *_imageShowTimer;
        SynchronisedQueue<sensor_msgs::ImageConstPtr> _imageQueue;
        SynchronisedQueue<QString> _consoleQueue;

        // UI
        boost::mutex _labelBricksMutex, _liveFeedMutex;
        bool _anyBricks, _safety;

        // Else
        rw::math::Q _qIdle;
        bool _liveFeed;
        bool _manualJog;
};

#endif // HMIWIDGET_HPP
