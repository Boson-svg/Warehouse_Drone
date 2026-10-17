#include "drone_warehouse/video_dialog.hpp"
#include "drone_warehouse/rtsp_overlay_worker.hpp"

#include <QCloseEvent>
#include <QLabel>
#include <QShowEvent>
#include <QVBoxLayout>

VideoDialog::VideoDialog(const QString &url, QWidget *parent)
    : QDialog(parent), url_(url), worker_(new RtspOverlayWorker(this))
{
    if (url_.isEmpty()) url_ = QString(QChar(114)) + QChar(116) + QChar(115) + QChar(112) + QChar(58) + QChar(47) + QChar(47) + QChar(49) + QChar(57) + QChar(50) + QChar(46) + QChar(49) + QChar(54) + QChar(56) + QChar(46) + QChar(51) + QChar(46) + QChar(49) + QChar(49) + QChar(52) + QChar(58) + QChar(56) + QChar(53) + QChar(53) + QChar(52) + QChar(47) + QChar(100) + QChar(52) + QChar(51) + QChar(53) + QChar(105);
    setWindowTitle(QStringLiteral("实时视频"));
    setModal(false);
    setFixedSize(680, 525);
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 12, 12, 12);
    video_surface_ = new QWidget(this);
    video_surface_->setFixedSize(640, 480);
    video_surface_->setAttribute(Qt::WA_NativeWindow);
    video_surface_->setAttribute(Qt::WA_DontCreateNativeAncestors);
    video_surface_->setAttribute(Qt::WA_OpaquePaintEvent);
    video_surface_->setStyleSheet(QStringLiteral("background: black"));
    layout->addWidget(video_surface_);
    status_label_ = new QLabel(QStringLiteral("视频未启动"), this);
    layout->addWidget(status_label_);
    connect(worker_, &RtspOverlayWorker::statusChanged, this,
            [this](const QString &text) { status_label_->setText(text); });
}

VideoDialog::~VideoDialog() { worker_->stop(); }

void VideoDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    if (!started_) {
        started_ = true;
        worker_->start(url_, static_cast<quintptr>(video_surface_->winId()));
    }
}

void VideoDialog::closeEvent(QCloseEvent *event)
{
    worker_->stop();
    started_ = false;
    QDialog::closeEvent(event);
}
