/*
    Logger for recording data

    2016-11-17 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef DVRK_LOGGER_H
#define DVRK_LOGGER_H

#include <chrono>
#include <fstream>
#include <QObject>
#include <QMutex>
#include <QImage>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>


namespace dvrk {

    class Logger : public QObject
    {
        ///QT Stuff, to communicate with the gui
    Q_OBJECT

    public:
        enum RecordSource{ UNKNOWN = 0,
            STEREO = 2,
            ULTRASOUND = 4,
            ARM = 8};

        Logger(QString active_arm_ = "");
        ~Logger();

    protected:
        ros::Subscriber stereo_sub_;

        ros::Subscriber ultrasound_sub_;

        ros::Subscriber psm1_sub_;
        ros::Subscriber psm2_sub_;
        ros::Subscriber mtml_sub_;
        ros::Subscriber mtmr_sub_;

        void stereoCallback (const sensor_msgs::ImageConstPtr& stereo_img_msg);
        void ultrasoundCallback (const sensor_msgs::ImageConstPtr& img_msg);
        void currentPSM1PoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void currentPSM2PoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void currentMTMLPoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void currentMTMRPoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    signals:
        void newStereoImage(QImage);
        void newUltrasoundImage(QImage);
        void setStatusMsg(QString);
        void setInfo1(QString);
        void setInfo2(QString);
        void setTimeCount(QString);
        void setArmPos(QString psm1, QString psm2, QString mtml, QString mtmr);

    private:
        std::string stereo_filename;

        std::string m_ultrasound_filename;

        std::string m_pose_filename;

        cv::VideoWriter *vid_writer;

        cv::VideoWriter *m_us_writer;

        std::ofstream m_pose_file;

        int m_record_source;

        cv::Mat stereo_rgb8_img_;

        cv::Mat m_ultrasound_img;

        QMutex mx_stereo;

        QMutex mx_ultrasound;

        QMutex mx_psm1, mx_psm2, mx_mtml, mx_mtmr;

        double psm1_pose[7], psm2_pose[7], mtml_pose[7], mtmr_pose[7];

        bool m_stereo_updated;
        bool m_ultrasound_updated;
        bool m_arm_updated;


        bool isRec;
        int frame_count;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_time_start;

        QImage cvtCvMat2QImage(const cv::Mat & image);
        cv::Mat cvtQImage2CvMat( const QImage &inImage, bool inCloneImageData = true );

        void init_pose();


    private Q_SLOTS:
        void start_record(int);
        void stop_record();
        void process_loop();


    };

}

#endif // DVRK_LOGGER_H
