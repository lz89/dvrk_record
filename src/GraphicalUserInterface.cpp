#include "GraphicalUserInterface.h"
#include <QPainter>

using namespace dvrk;

// Constructor
GraphicalUserInterface::GraphicalUserInterface()
{
    this->setupUi(this);


    m_green_icon = QPixmap (":/icons/green.png");
    m_red_icon = QPixmap (":/icons/red.png");
    m_hamlyn_logo = QPixmap (":/icons/hamlyn.jpg");
    this->light_label->setPixmap(m_green_icon.scaled(50,50,Qt::KeepAspectRatio));
    this->logo_label->setPixmap(m_hamlyn_logo);

    // Set up action signals and slots
    connect(this->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}


void GraphicalUserInterface::slotExit()
{
    qApp->exit();
}


void GraphicalUserInterface::newStereoImage(QImage qimage){
    int w = stereo_label->width();
    int h = stereo_label->height();
    if(stereo_label->isVisible()){
        stereo_label->setPixmap(QPixmap::fromImage(qimage).scaled(w,h,Qt::KeepAspectRatio));
//        stereo_label->setPixmap(QPixmap::fromImage(qimage));
        stereo_label->repaint();
    }
}

void GraphicalUserInterface::newUltrasoundImage(QImage qimage){
    int w = ultrasound_label->width();
    int h = ultrasound_label->height();
    if(ultrasound_label->isVisible()){
        ultrasound_label->setPixmap(QPixmap::fromImage(qimage).scaled(w,h,Qt::KeepAspectRatio));
        ultrasound_label->repaint();
    }
}

void GraphicalUserInterface::setStatusMsg(QString msg) {
    status_label->setText(msg);
}

void GraphicalUserInterface::setTimeCount(QString msg) {
    time_count_label->setText(msg);
}

void GraphicalUserInterface::setInfo1(QString message)
{
    info_label1->setText(message);
}

void GraphicalUserInterface::setInfo2(QString message)
{
    info_label2->setText(message);
}

void GraphicalUserInterface::on_recordButton_released()
{
    this->light_label->setPixmap(m_red_icon.scaled(50,50,Qt::KeepAspectRatio));
    Q_EMIT StartRecord();
}

void GraphicalUserInterface::on_stopButton_released()
{
    this->light_label->setPixmap(m_green_icon.scaled(50,50,Qt::KeepAspectRatio));
    Q_EMIT StopRecord();
}

