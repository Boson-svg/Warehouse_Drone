#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <atomic>
#include <thread>

class QWidget;
class QDialog;

class VideoReplayController final : public QObject
{
    Q_OBJECT

public:
    explicit VideoReplayController(const QString &rtsp_url,
                                   QWidget *dialog_parent,
                                   QObject *parent = nullptr);
    ~VideoReplayController() override;

    void setArmed(bool armed);
    void showReplayDialog();

signals:
    void recordingStateChanged(const QString &text);

private:
    void startRecording();
    void stopRecording();
    void recordingLoop(QString file_path);
    void refreshRecordings();
    void cleanupRecordings(int keep_count);

    QString rtsp_url_;
    QString recordings_dir_;
    QWidget *dialog_parent_{nullptr};
    std::thread recording_thread_;
    std::atomic_bool stop_requested_{false};
    std::atomic_bool recording_{false};
    bool armed_{false};
    int disarm_stable_count_{0};
    QString active_file_path_;
    QStringList recordings_;
    QDialog *replay_dialog_{nullptr};
};
