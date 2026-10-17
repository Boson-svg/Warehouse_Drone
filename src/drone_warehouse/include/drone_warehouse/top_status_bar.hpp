#pragma once

#include <QFrame>
#include <QString>

class QLabel;
class QPushButton;
class RosManager;

class QTimer;

class TopStatusBar : public QFrame
{
    Q_OBJECT

public:
    explicit TopStatusBar(QWidget *parent = nullptr);

    void setConnectionText(const QString &text);//设置连接状态文本
    void setTaskText(const QString &text);//设置任务状态文本
    void setShelfText(const QString &text);//设置货架信息文本
    void setTimeText(const QString &text);//设置时间文本

    void setConnected(bool connected);
    // 空地协同模式只保留小车的暂停、恢复操作，隐藏常规巡检按钮。
    void setCollaborationMode(bool enabled);

    /*********************ros移植部分***********************/
    void setTriggerTime(const QString &text);//设置时间触发的目标时刻，格式先按 HH:mm:ss 使用
    void setTimeTriggerEnabled(bool enabled);//设置是否启用到点触发上传
    void updateDelta(double dx, double dy, double dyaw, bool valid);//更新无人机位置增量
    /******************************************************/

    QPoint connectionButtonBottomLeftGlobal() const;//返回连接按钮左下角的全局坐标
    QPoint titleButtonBottomLeftGlobal() const;//返回标题按钮左下角的全局坐标
    QPoint shelfButtonBottomLeftGlobal() const;//返回货架信息按钮左下角的全局坐标

signals:
    void connectionButtonClicked();//连接按钮被点击
    void titleClicked();//标题按钮被点击
    void connectionClicked();//连接状态按钮被点击
    void taskClicked();//任务状态按钮被点击
    void shelfButtonClicked();//货架按钮被点击
    void displayButtonClicked();//显示视频按钮被点击
    void replayButtonClicked();//回放按钮被点击
    void aiAnalyzeButtonClicked();//分析按钮被点击
    void executeButtonClicked();//执行按钮被点击
    void waypointButtonClicked();//航点飞行按钮被点击
    void scheduledcheckbuttonnClicked(QString mission_trigger_time);//巡检按钮被点击
    void carPauseRequested();//请求小车进入手动暂停模式
    void carResumeRequested();//请求小车恢复自动运行模式

    void exitRequested();//退出信号

    /*********************ros移植部分***********************/
    void triggerTimeReached(const QString &time_text);//顶部时钟到达目标时刻时发出信号，交给主窗口决定是否执行上传
    /******************************************************/

private:
    void updateOperationButtonVisibility();//统一刷新右侧操作按钮，避免连接和项目切换互相覆盖

    QPushButton *title_button_ = nullptr;//标题按钮
    QPushButton *connection_button_ = nullptr;//连接状态按钮
    QPushButton *shelf_button_ = nullptr;//货架状态按钮
    QPushButton *task_button_ = nullptr;//任务状态按钮
    QLabel *dx_indicator_label_{nullptr};//dx指示灯标签
    QLabel *dy_indicator_label_{nullptr};//dy指示灯标签
    QLabel *dyaw_indicator_label_{nullptr};//dyaw指示灯标签
    QLabel *dx_value_label_{nullptr};//dx数值标签
    QLabel *dy_value_label_{nullptr};//dy数值标签
    QLabel *dyaw_value_label_{nullptr};//dyaw数值标签
    QPushButton *analysis_button_ = nullptr;//分析按钮
    QPushButton *display_button_ = nullptr;//显示视频按钮
    QPushButton *replay_button_ = nullptr;//回放按钮
    QPushButton *execute_button_ = nullptr;//执行按钮
    QPushButton *waypoint_button_ = nullptr;//航点飞行按钮
    QPushButton *scheduled_check_button_ = nullptr;//执行按钮
    QPushButton *car_pause_button_ = nullptr;//空地协同中的小车暂停按钮
    QPushButton *car_resume_button_ = nullptr;//空地协同中的小车恢复按钮
    QLabel *time_label_ = nullptr;//时间标签

    RosManager *ros_manager_{nullptr};//ROS管理器

    QTimer *clock_timer_ = nullptr;//用于每秒刷新一次顶部时间

    /*********************ros移植部分***********************/
    QString trigger_time_text_;//目标触发时刻，当前先按 HH:mm:ss 保存
    QString last_triggered_time_text_;//记录最近一次已经触发过的时刻文本，避免同一秒重复触发
    bool time_trigger_enabled_ = false;//当前是否启用到点触发上传
    /******************************************************/

    bool connected_ = false;//保存连接状态，项目切换时据此决定按钮是否可见
    bool collaboration_mode_ = false;//当前是否为空地协同项目

    QTimer *exit_long_press_timer_{nullptr};
    bool stop_button_pressed_{false};
    bool long_press_triggered_{false};
    int stop_press_token_{0};
};