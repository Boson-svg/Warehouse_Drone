#pragma once

#include <QDialog>

class QLabel;
class QWidget;
class RtspOverlayWorker;

class VideoDialog final : public QDialog
{
    Q_OBJECT
public:
    explicit VideoDialog(const QString &url, QWidget *parent = nullptr);
    ~VideoDialog() override;

protected:
    void showEvent(QShowEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private:
    QString url_;
    QWidget *video_surface_{nullptr};
    QLabel *status_label_{nullptr};
    RtspOverlayWorker *worker_{nullptr};
    bool started_{false};
};
