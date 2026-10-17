#pragma once

#include <QObject>
#include <QString>
#include <thread>
#include <atomic>

class RtspOverlayWorker final : public QObject
{
    Q_OBJECT
public:
    explicit RtspOverlayWorker(QObject *parent = nullptr);
    ~RtspOverlayWorker() override;

    void start(const QString &url, quintptr window_handle);
    void stop();

signals:
    void statusChanged(const QString &text);

private:
    void run(QString url, quintptr window_handle);
    std::thread thread_;
    std::atomic_bool stop_requested_{false};
    std::atomic_bool running_{false};
};
