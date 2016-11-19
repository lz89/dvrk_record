#include "GraphicalUserInterface.h"
#include <QPainter>

// Constructor
GraphicalUserInterface::GraphicalUserInterface()
{
    this->setupUi(this);


    m_green_icon = QPixmap (":/icons/green.png");
    m_red_icon = QPixmap (":/icons/red.png");
    m_hamlyn_logo = QPixmap (":/icons/hamlyn.jpg");
    this->light_label->setPixmap(m_green_icon.scaled(50,50,Qt::KeepAspectRatio));
    this->logo_label->setPixmap(m_hamlyn_logo);

//    stereo_label->setScaledContents(true);

    // Set up action signals and slots
    connect(this->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}


void GraphicalUserInterface::slotExit()
{
    qApp->exit();
}


void GraphicalUserInterface::newStereoImage(QImage qimage){
//    int w = stereo_label->width();
//    int h = stereo_label->height();
    if(stereo_label->isVisible()){
//        stereo_label->setPixmap(QPixmap::fromImage(qimage).scaled(w,h,Qt::KeepAspectRatio));
        stereo_label->setPixmap(QPixmap::fromImage(qimage));
        stereo_label->repaint();
    }
}

void GraphicalUserInterface::newUltrasoundImage(QImage qimage){
//    int w = ultrasound_label->width();
//    int h = ultrasound_label->height();
    if(ultrasound_label->isVisible()){
//        ultrasound_label->setPixmap(QPixmap::fromImage(qimage).scaled(w,h,Qt::KeepAspectRatio));
        ultrasound_label->setPixmap(QPixmap::fromImage(qimage));
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
    int source = UNKNOWN;
    if (this->stereo_checkBox->checkState() == Qt::Checked)
        source = source | STEREO;
    if (this->ultrasound_checkBox->checkState() == Qt::Checked)
        source = source | ULTRASOUND;
    if (this->arm_checkBox->checkState() == Qt::Checked)
        source = source | ARM;


    Q_EMIT StartRecord(source);
}

void GraphicalUserInterface::on_stopButton_released()
{
    this->light_label->setPixmap(m_green_icon.scaled(50,50,Qt::KeepAspectRatio));
    Q_EMIT StopRecord();
}

void GraphicalUserInterface::setArmPos(QString psm1, QString psm2, QString mtml, QString mtmr) {
    this->psm1_pos_label->setText(psm1);
    this->psm2_pos_label->setText(psm2);
    this->mtml_pos_label->setText(mtml);
    this->mtmr_pos_label->setText(mtmr);
}

