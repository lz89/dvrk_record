#include "include/logger.h"
#include <QDebug>

using namespace dvrk;

Logger::Logger(QString active_arm_) :
        isRec(false)
        , vid_writer(NULL)
        , m_us_writer(NULL)
        , stereo_rgb8_img_(cv::Mat())
        , frame_count (1)
        , m_record_source (UNKNOWN)
{

    m_stereo_updated = false;
    m_ultrasound_updated = false;
    init_pose();
    /***
     * ROS related init
     * */
    std::string stereo_tpc ("/davinci/stereo_image");
    std::string ultrasound_tpc ("/davinci/ultrasound_image");

    ros::NodeHandle nh;
    int q = 1;  // queue size
    stereo_sub_ = nh.subscribe<sensor_msgs::Image> (stereo_tpc, 1, &Logger::stereoCallback, this);
    ultrasound_sub_ = nh.subscribe<sensor_msgs::Image> (ultrasound_tpc, 1, &Logger::ultrasoundCallback, this);

    std::string psm1_curr_pose_tpc ("/dvrk/PSM1/position_cartesian_current");
    std::string psm2_curr_pose_tpc ("/dvrk/PSM2/position_cartesian_current");

    std::string psm1_curr_jnt_tpc ("/dvrk/PSM1/state_joint_current");
    std::string psm2_curr_jnt_tpc ("/dvrk/PSM2/state_joint_current");

    std::string psm3_curr_pose_tpc ("/dvrk/PSM3/position_cartesian_current");
    std::string mtml_curr_pose_tpc ("/dvrk/MTML/position_cartesian_current");
    std::string mtmr_curr_pose_tpc ("/dvrk/MTMR/position_cartesian_current");
    std::string ecm_curr_pose_tpc  ("/dvrk/ECM/position_cartesian_current");
    psm1_sub_ = nh.subscribe(psm1_curr_pose_tpc, 1, &Logger::currentPSM1PoseStampedCallback, this);
    psm2_sub_ = nh.subscribe(psm2_curr_pose_tpc, 1, &Logger::currentPSM2PoseStampedCallback, this);
    psm1_jnt_sub_ = nh.subscribe(psm1_curr_jnt_tpc, 1, &Logger::currentPSM1JointStampedCallback, this);
    psm2_jnt_sub_ = nh.subscribe(psm2_curr_jnt_tpc, 1, &Logger::currentPSM2JointStampedCallback, this);

    mtml_sub_ = nh.subscribe(mtml_curr_pose_tpc, 1, &Logger::currentMTMLPoseStampedCallback, this);
    mtmr_sub_ = nh.subscribe(mtmr_curr_pose_tpc, 1, &Logger::currentMTMRPoseStampedCallback, this);
}

Logger::~Logger()
{
    if (!vid_writer)
        delete vid_writer;  // release() is called in the destructor
    if (!m_us_writer)
        delete m_us_writer;  // release() is called in the destructor

}

/**********************************************************************************
 * Callback functions
 *********************************************************************************/

void Logger::stereoCallback (const sensor_msgs::ImageConstPtr& stereo_img_msg)
{
    QMutexLocker locker(&mx_stereo);
    try
    {
        stereo_rgb8_img_ = cv_bridge::toCvCopy(stereo_img_msg, sensor_msgs::image_encodings::BGR8)->image;
        m_stereo_updated = true;
        QImage qtemp = cvtCvMat2QImage(stereo_rgb8_img_);
        Q_EMIT newStereoImage(qtemp);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void Logger::ultrasoundCallback(const sensor_msgs::ImageConstPtr &img_msg) {
    QMutexLocker locker(&mx_ultrasound);

    try
    {
        m_ultrasound_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        m_ultrasound_updated = true;
        QImage qtemp = cvtCvMat2QImage(m_ultrasound_img);
        Q_EMIT newUltrasoundImage(qtemp);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


void Logger::start_record(int source)
{
    if (isRec)
        return;
    m_record_source = source;
    isRec = true;
    frame_count = 1;
    m_time_start = std::chrono::system_clock::now();
    Q_EMIT setStatusMsg("Recording...");
}

void Logger::stop_record()
{
    isRec = false;
    mx_stereo.lock();
    if (vid_writer)
    {
        delete vid_writer;  // release() is called in the destructor
        vid_writer = NULL;
    }
    mx_stereo.unlock();

    mx_ultrasound.lock();
    if (m_us_writer)
    {
        delete m_us_writer;
        m_us_writer = NULL;
    }
    mx_ultrasound.unlock();

    if (m_pose_file.is_open())
    {
        m_pose_file.close();
    }

    Q_EMIT setStatusMsg("Record stop");

    QString msg = QString::fromStdString("Saved stereo video to: " + stereo_filename);
    Q_EMIT setInfo1(msg);
    msg = QString::fromStdString("Saved ultrasound video to: " + m_ultrasound_filename);
    Q_EMIT setInfo2(msg);

    init_pose();
}


QImage  Logger::cvtCvMat2QImage( const cv::Mat &inMat )
{
    switch ( inMat.type() )
    {
        // 8-bit, 4 channel
        case CV_8UC4:
        {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

            return image;
        }

            // 8-bit, 3 channel
        case CV_8UC3:
        {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

            return image.rgbSwapped();
        }

            // 8-bit, 1 channel
        case CV_8UC1:
        {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if ( sColorTable.isEmpty() )
            {
                for ( int i = 0; i < 256; ++i )
                    sColorTable.push_back( qRgb( i, i, i ) );
            }

            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );

            return image;
        }

        default:
            qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
            break;
    }

    return QImage();
}


// If inImage exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inImage's
// data with the cv::Mat directly
//    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
cv::Mat Logger::cvtQImage2CvMat( const QImage &inImage, bool inCloneImageData )
{
    switch ( inImage.format() )
    {
        // 8-bit, 4 channel
        case QImage::Format_RGB32:
        {
            cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC4, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );

            return (inCloneImageData ? mat.clone() : mat);
        }

            // 8-bit, 3 channel
        case QImage::Format_RGB888:
        {
            if ( !inCloneImageData )
                qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";

            QImage   swapped = inImage.rgbSwapped();

            return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
        }

            // 8-bit, 1 channel
        case QImage::Format_Indexed8:
        {
            cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC1, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );

            return (inCloneImageData ? mat.clone() : mat);
        }

        default:
            qWarning() << "ASM::QImageToCvMat() - QImage format not handled in switch:" << inImage.format();
            break;
    }

    return cv::Mat();
}

void Logger::process_loop() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    // Stereo
    if (m_record_source & STEREO)
    {
        mx_stereo.lock();
        if (!vid_writer && isRec && !stereo_rgb8_img_.empty())
        {
            char buffer_stereo [100];
            strftime (buffer_stereo,80,"stereo-%G%b%dT%H%M%S",now);

            stereo_filename = "/home/hamlyn/temp/";
            stereo_filename += buffer_stereo;
            stereo_filename += ".avi";

            vid_writer = new cv::VideoWriter(stereo_filename, CV_FOURCC('D','I','V','X'), 25, stereo_rgb8_img_.size(), true);

            QString msg = QString::fromStdString("Writing stereo video to: " + stereo_filename);
            Q_EMIT setInfo1(msg);
        }

        if (isRec && m_stereo_updated)
        {
            vid_writer->write(stereo_rgb8_img_);
            m_stereo_updated = false;
            std::chrono::time_point<std::chrono::high_resolution_clock> curr = std::chrono::high_resolution_clock::now();
            int t = std::chrono::duration_cast<std::chrono::seconds>(curr - m_time_start).count();
            QString msg = QString::number(t) + QString("s");
            Q_EMIT setTimeCount(msg);
        }
        mx_stereo.unlock();
    }

    // Ultrasound
    if (m_record_source & ULTRASOUND)
    {
        mx_ultrasound.lock();
        if (!m_us_writer && isRec && !m_ultrasound_img.empty())
        {
            char buffer_us [100];
            strftime (buffer_us,80,"US-%G%b%dT%H%M%S",now);

            m_ultrasound_filename = "/home/hamlyn/temp/";
            m_ultrasound_filename += buffer_us;
            m_ultrasound_filename += ".avi";

            QString msg = QString::fromStdString("Writing ultrasound video to: " + m_ultrasound_filename);
            Q_EMIT setInfo2(msg);
            m_us_writer = new cv::VideoWriter(m_ultrasound_filename, CV_FOURCC('D','I','V','X'), 25, m_ultrasound_img.size(), true);
        }
        if (isRec && m_ultrasound_updated)
        {
            m_us_writer->write(m_ultrasound_img);
            m_ultrasound_updated = false;
        }
        mx_ultrasound.unlock();
    }

    // PSM
    if (m_record_source & ARM)
    {
        if (!m_pose_file.is_open() && isRec)
        {
            char buffer_pose [100];
            strftime (buffer_pose,80,"arm-%G%b%dT%H%M%S",now);

            m_pose_filename = "/home/hamlyn/temp/";
            m_pose_filename += buffer_pose;
            m_pose_filename += ".txt";

            std::cout << "Writing pose to: " << m_pose_filename << std::endl;

            m_pose_file.open(m_pose_filename);

            // Write header
            m_pose_file << "frame,timestamp,";
            m_pose_file << "psm1_00,psm1_01,psm1_02,psm1_03,psm1_10,psm1_11,psm1_12,psm1_13,psm1_20,psm1_21,psm1_22,psm1_23,";
            m_pose_file << "psm2_00,psm2_01,psm2_02,psm2_03,psm2_10,psm2_11,psm2_12,psm2_13,psm2_20,psm2_21,psm2_22,psm2_23,";
//            m_pose_file << "psm3_00,psm3_01,psm3_02,psm3_03,psm3_10,psm3_11,psm3_12,psm3_13,psm3_20,psm3_21,psm3_22,psm2_23,";
            m_pose_file << "psm1_jaw_angle,psm2_jaw_angle,psm3_jaw_angle,";
//            m_pose_file << "ecm_00,ecm_01,ecm_02,ecm_03,ecm_10,ecm_11,ecm_12,ecm_13,ecm_20,ecm_21,ecm_22,ecm_23,";
            m_pose_file << "mtm_l_00,mtm_l_01,mtm_l_02,mtm_l_03,mtm_l_10,mtm_l_11,mtm_l_12,mtm_l_13,mtm_l_20,mtm_l_21,mtm_l_22,mtm_l_23,";
            m_pose_file << "mtm_r_00,mtm_r_01,mtm_r_02,mtm_r_03,mtm_r_10,mtm_r_11,mtm_r_12,mtm_r_13,mtm_r_20,mtm_r_21,mtm_r_22,mtm_r_23,";
            m_pose_file << std::endl;
        }

        QMutexLocker locker1(&mx_psm1);
        QMutexLocker locker2(&mx_psm2);
        QMutexLocker locker3(&mx_mtml);
        QMutexLocker locker4(&mx_mtmr);

        if (isRec && m_arm_updated)
        {
            m_pose_file << frame_count++ << ",";
            m_pose_file <<  static_cast<u_int32_t>(ros::Time::now().toNSec() * 1e-6) << ",";

            for (int i = 0; i < 7; i++)
                m_pose_file << std::setprecision(6) << psm1_pose[i] << ",";
            for (int i = 0; i < 7; i++)
                m_pose_file << std::setprecision(6) << psm2_pose[i] << ",";
//        for (int i = 0; i < 7; i++)
//            m_pose_file << std::setprecision(6) << psm2_pose[i] << ",";
            // TODO: read jaw angle and save
            m_pose_file << std::setprecision(3) << psm_jaw[0] << "," << psm_jaw[1] << "," << psm_jaw[2] << ",";
            for (int i = 0; i < 7; i++)
                m_pose_file << std::setprecision(6) << mtml_pose[i] << ",";
            for (int i = 0; i < 6; i++)
                m_pose_file << std::setprecision(6) << mtmr_pose[i] << ",";
            m_pose_file << std::setprecision(6) << mtmr_pose[6] << std::endl;

            m_arm_updated = false;
        }
    }

    QString psm1_msg = QString("X: %1\nY:%2\nZ:%3\nJaw:%4 deg").arg(
            QString::number(psm1_pose[4]), QString::number(psm1_pose[5]), QString::number(psm1_pose[6]),
            QString::number(psm_jaw[0]*180/M_PI));
    QString psm2_msg = QString("X: %1\nY:%2\nZ:%3\nJaw:%4 deg").arg(
            QString::number(psm2_pose[4]), QString::number(psm2_pose[5]), QString::number(psm2_pose[6]),
            QString::number(psm_jaw[1]*180/M_PI));
    QString mtml_msg = QString("X: %1\nY:%2\nZ:%3").arg(QString::number(mtml_pose[4]), QString::number(mtml_pose[5]), QString::number(mtml_pose[6]));
    QString mtmr_msg = QString("X: %1\nY:%2\nZ:%3").arg(QString::number(mtmr_pose[4]), QString::number(mtmr_pose[5]), QString::number(mtmr_pose[6]));

    Q_EMIT setArmPos(psm1_msg, psm2_msg, mtml_msg, mtmr_msg);

    if (isRec)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> curr = std::chrono::high_resolution_clock::now();
        int t = std::chrono::duration_cast<std::chrono::seconds>(curr - m_time_start).count();
        QString msg = QString::number(t) + QString("s");
        Q_EMIT setTimeCount(msg);
    }
}

void Logger::currentPSM1PoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    QMutexLocker locker(&mx_psm1);
    psm1_pose[0] = msg->pose.orientation.w;
    psm1_pose[1] = msg->pose.orientation.x;
    psm1_pose[2] = msg->pose.orientation.y;
    psm1_pose[3] = msg->pose.orientation.z;
    psm1_pose[4] = msg->pose.position.x;
    psm1_pose[5] = msg->pose.position.y;
    psm1_pose[6] = msg->pose.position.z;
    m_arm_updated = true;
}

void Logger::currentPSM2PoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    QMutexLocker locker(&mx_psm2);
    psm2_pose[0] = msg->pose.orientation.w;
    psm2_pose[1] = msg->pose.orientation.x;
    psm2_pose[2] = msg->pose.orientation.y;
    psm2_pose[3] = msg->pose.orientation.z;
    psm2_pose[4] = msg->pose.position.x;
    psm2_pose[5] = msg->pose.position.y;
    psm2_pose[6] = msg->pose.position.z;
    m_arm_updated = true;
}


void Logger::currentPSM1JointStampedCallback(const sensor_msgs::JointState &msg) {
    psm_jaw[0] = msg.position[6];
}

void Logger::currentPSM2JointStampedCallback(const sensor_msgs::JointState &msg) {
    psm_jaw[1] = msg.position[6];
}


void Logger::currentMTMLPoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    QMutexLocker locker(&mx_mtml);
    mtml_pose[0] = msg->pose.orientation.w;
    mtml_pose[1] = msg->pose.orientation.x;
    mtml_pose[2] = msg->pose.orientation.y;
    mtml_pose[3] = msg->pose.orientation.z;
    mtml_pose[4] = msg->pose.position.x;
    mtml_pose[5] = msg->pose.position.y;
    mtml_pose[6] = msg->pose.position.z;
    m_arm_updated = true;
}

void Logger::currentMTMRPoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    QMutexLocker locker(&mx_mtmr);
    mtmr_pose[0] = msg->pose.orientation.w;
    mtmr_pose[1] = msg->pose.orientation.x;
    mtmr_pose[2] = msg->pose.orientation.y;
    mtmr_pose[3] = msg->pose.orientation.z;
    mtmr_pose[4] = msg->pose.position.x;
    mtmr_pose[5] = msg->pose.position.y;
    mtmr_pose[6] = msg->pose.position.z;
    m_arm_updated = true;
}

void Logger::init_pose() {
    for (int i = 0; i < 7; i++)
    {
        psm1_pose[i] = std::numeric_limits<double>::quiet_NaN();
        psm2_pose[i] = std::numeric_limits<double>::quiet_NaN();
        mtml_pose[i] = std::numeric_limits<double>::quiet_NaN();
        mtmr_pose[i] = std::numeric_limits<double>::quiet_NaN();
    }
    psm_jaw[0] = psm_jaw[1] = psm_jaw[2] = std::numeric_limits<double>::quiet_NaN();
}



