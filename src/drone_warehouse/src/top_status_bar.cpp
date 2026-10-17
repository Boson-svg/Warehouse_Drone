#include "drone_warehouse/top_status_bar.hpp"
#include "drone_warehouse/color_palette.hpp"
#include "drone_warehouse/ros_manager.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <QDateTime>
#include <QTimer>

TopStatusBar::TopStatusBar(QWidget *parent)
    : QFrame(parent)
{
    setObjectName("topStatusBar");

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 8, 12, 8);
    layout->setSpacing(12);

    title_button_ = new QPushButton("智航", this);
    connection_button_ = new QPushButton("未连接", this);
    shelf_button_ = new QPushButton("货架信息", this);
    task_button_ = new QPushButton("任务待命", this);

    dx_indicator_label_ = new QLabel(this);
    dy_indicator_label_ = new QLabel(this);
    dyaw_indicator_label_ = new QLabel(this);
    // dx_value_label_ = new QLabel("dx:", this);
    // dy_value_label_ = new QLabel("dy:", this);
    // dyaw_value_label_ = new QLabel("dyaw:", this);
    dx_indicator_label_->setFixedSize(16, 16);
    dx_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );
    dy_indicator_label_->setFixedSize(16, 16);
    dy_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );
    dyaw_indicator_label_->setFixedSize(16, 16);
    dyaw_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );

    analysis_button_ = new QPushButton("分析", this);
    display_button_ = new QPushButton(this); display_button_->setText(QString(QChar(26174)) + QChar(31034));
    replay_button_ = new QPushButton(this); replay_button_->setText(QString(QChar(22238)) + QChar(25918));
    execute_button_ = new QPushButton("执行", this);
    waypoint_button_ = new QPushButton("航点飞行", this);
    scheduled_check_button_ = new QPushButton("定时巡检", this);
    car_pause_button_ = new QPushButton("暂停", this);
    car_resume_button_ = new QPushButton("恢复", this);
    time_label_ = new QLabel("00:00:00", this);

    clock_timer_ = new QTimer(this);//新建定时器
    time_label_->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));//立即刷新一次

    layout->addWidget(connection_button_);
    layout->addWidget(title_button_);
    layout->addWidget(shelf_button_);
    
    layout->addWidget(task_button_);
    layout->addStretch();

    // layout->addWidget(dx_value_label_);
    layout->addWidget(dx_indicator_label_);
    // layout->addWidget(dy_value_label_);
    layout->addWidget(dy_indicator_label_);
    // layout->addWidget(dyaw_value_label_);
    layout->addWidget(dyaw_indicator_label_);

    layout->addStretch();
    layout->addWidget(display_button_);
    layout->addWidget(replay_button_);
    layout->addWidget(analysis_button_);
    layout->addWidget(execute_button_);
    layout->addWidget(waypoint_button_);
    layout->addWidget(scheduled_check_button_);
    layout->addWidget(car_pause_button_);
    layout->addWidget(car_resume_button_);
    layout->addWidget(time_label_);

    // title_button_->hide();
    // shelf_button_->hide();
    analysis_button_->hide();
    display_button_->hide();
    replay_button_->hide();
    task_button_->hide();
    execute_button_->hide();
    waypoint_button_->hide();
    scheduled_check_button_->hide();
    car_pause_button_->hide();
    car_resume_button_->hide();

    exit_long_press_timer_ = new QTimer(this);
    exit_long_press_timer_->setSingleShot(true);

    connect(connection_button_, &QPushButton::clicked, this, &TopStatusBar::connectionButtonClicked);
    connect(title_button_, &QPushButton::clicked, this, &TopStatusBar::titleClicked);
    connect(task_button_, &QPushButton::clicked, this, &TopStatusBar::taskClicked);
    connect(analysis_button_, &QPushButton::clicked, this, &TopStatusBar::aiAnalyzeButtonClicked);
    connect(display_button_, &QPushButton::clicked, this, &TopStatusBar::displayButtonClicked);
    connect(replay_button_, &QPushButton::clicked, this, &TopStatusBar::replayButtonClicked);
    connect(execute_button_, &QPushButton::clicked, this, &TopStatusBar::executeButtonClicked);
    connect(waypoint_button_, &QPushButton::clicked, this, &TopStatusBar::waypointButtonClicked);
    connect(shelf_button_, &QPushButton::clicked, this, &TopStatusBar::shelfButtonClicked);
    connect(car_pause_button_, &QPushButton::clicked, this, &TopStatusBar::carPauseRequested);
    connect(car_resume_button_, &QPushButton::clicked, this, &TopStatusBar::carResumeRequested);
    connect(scheduled_check_button_, &QPushButton::clicked, this, [this]() {
        const QString mission_trigger_time_text_ = QDateTime::currentDateTime().toString("HH:mm:ss");
        emit scheduledcheckbuttonnClicked(mission_trigger_time_text_);
    });

    connect(clock_timer_, &QTimer::timeout, this, [this]() {//每秒触发刷新一次时间文本
        const QString current_time_text = QDateTime::currentDateTime().toString("HH:mm:ss");
        time_label_->setText(current_time_text);

        /*********************ros移植部分***********************/
        // 当前先把“到点触发上传”的检测也挂在顶部状态栏已有的时钟定时器里。
        // 这样不需要额外再造一个新的 UI 定时器，主窗口只要接收信号并决定是否真正上传即可。
        if (time_trigger_enabled_ && !trigger_time_text_.isEmpty())
        {
            if (current_time_text == trigger_time_text_)
            {
                if (last_triggered_time_text_ != current_time_text)
                {
                    last_triggered_time_text_ = current_time_text;
                    emit triggerTimeReached(current_time_text);
                }
            }
            else
            {
                // 一旦离开目标时刻，就清掉“本秒已触发”的标记，方便后续再次到点时还能重新触发。
                last_triggered_time_text_.clear();
            }
        }
        /******************************************************/
    });

    connect(connection_button_, &QPushButton::pressed, this, [this]() {
        stop_button_pressed_ = true;
        long_press_triggered_ = false;

        const int current_token = ++stop_press_token_;

        QTimer::singleShot(1500, this, [this, current_token]() {
            if (!stop_button_pressed_) {
                return;
            }

            if (current_token != stop_press_token_) {
                return;
            }

            if (connection_button_ && connection_button_->isDown()) {
                long_press_triggered_ = true;

                emit exitRequested();
            }
        });
    });

    clock_timer_->start(1000);

    setStyleSheet(
        "#topStatusBar {"
        "background: rgba(20, 28, 40, 180);"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 10px;"
        "}"
        "#topStatusBar QPushButton {"
        "background: transparent;"//按钮背景透明
        "border: none;"//按钮无边框
        "color: #d7e3f4;"//按钮文字颜色
        "padding: 6px 10px;"//按钮内边距
        "}"
        "#topStatusBar QLabel {"
        "background: transparent;"//背景透明
        "border: none;"//无边框
        "color: #d7e3f4;"//文字颜色
        "padding: 6px 10px;"//内边距
        "}"
        "#topStatusBar QPushButton:hover {"//按钮悬停效果
        "background: rgba(70, 110, 160, 80);"//悬停时背景变亮
        "border-radius: 6px;"//悬停时圆角稍微变大
        "}"
        "#topStatusBar QLabel {"
        "color: #d7e3f4;"
        "}"
    );
}

void TopStatusBar::setConnected(bool connected)
{
    connected_ = connected;

    // title_button_->setVisible(connected);
    //shelf_button_->setVisible(connected);
    task_button_->setVisible(connected);

    // dx_value_label_->setVisible(connected);
    dx_indicator_label_->setVisible(connected);
    // dy_value_label_->setVisible(connected);
    dy_indicator_label_->setVisible(connected);
    // dyaw_value_label_->setVisible(connected);
    dyaw_indicator_label_->setVisible(connected);

    updateOperationButtonVisibility();
    connection_button_->setText(connected ? "已连接" : "未连接");
}

void TopStatusBar::setCollaborationMode(bool enabled)
{
    collaboration_mode_ = enabled;
    updateOperationButtonVisibility();
}

void TopStatusBar::updateOperationButtonVisibility()
{
    // 普通项目沿用原来的四个操作按钮；空地协同只显示暂停和恢复。
    const bool show_inspection_actions = connected_ && !collaboration_mode_;
    analysis_button_->setVisible(show_inspection_actions);
    display_button_->setVisible(show_inspection_actions);
    replay_button_->setVisible(show_inspection_actions);
    execute_button_->setVisible(show_inspection_actions);
    waypoint_button_->setVisible(show_inspection_actions);
    scheduled_check_button_->setVisible(show_inspection_actions);

    const bool show_car_actions = connected_ && collaboration_mode_;
    car_pause_button_->setVisible(show_car_actions);
    car_resume_button_->setVisible(show_car_actions);
}

void TopStatusBar::setConnectionText(const QString &text)
{
    connection_button_->setText(text);
}

void TopStatusBar::setTaskText(const QString &text)
{
    task_button_->setText(text);
}

void TopStatusBar::setShelfText(const QString &text)
{
    shelf_button_->setText(text);
}

void TopStatusBar::setTimeText(const QString &text)
{
    time_label_->setText(text);
}

/*********************ros移植部分***********************/
void TopStatusBar::setTriggerTime(const QString &text)
{
    trigger_time_text_ = text;
    last_triggered_time_text_ = text;
}

void TopStatusBar::setTimeTriggerEnabled(bool enabled)
{
    time_trigger_enabled_ = enabled;
    if (!time_trigger_enabled_)
    {
        last_triggered_time_text_.clear();
    }
}
/******************************************************/

QPoint TopStatusBar::connectionButtonBottomLeftGlobal() const
{
    return connection_button_->mapToGlobal(
        QPoint(0, connection_button_->height()));
}
QPoint TopStatusBar::titleButtonBottomLeftGlobal() const
{
    return title_button_->mapToGlobal(
        QPoint(0, title_button_->height()));
}
QPoint TopStatusBar::shelfButtonBottomLeftGlobal() const
{
    return shelf_button_->mapToGlobal(QPoint(0, shelf_button_->height()));
}

void TopStatusBar::updateDelta(double dx, double dy, double dyaw, bool valid)
{
    const double abs_dx = std::abs(dx);
    const double abs_dy = std::abs(dy);
    const double abs_dyaw = std::abs(dyaw);

    auto setIndicatorColor = [](QLabel *label, const QString &color) {
        if (!label) {
            return;
        }

        label->setStyleSheet(QString(
            "background-color: %1;"
            "border-radius: 6px;"
            "border: 1px solid #666;"
        ).arg(color));
    };

    if (!valid) {
        setIndicatorColor(dx_indicator_label_, "#9e9e9e");
        setIndicatorColor(dy_indicator_label_, "#9e9e9e");
        setIndicatorColor(dyaw_indicator_label_, "#9e9e9e");
        return;
    }

    auto updateIndicator = [&](QLabel *label, double value, double green_limit, double yellow_limit) {
        if (value <= green_limit) {
            setIndicatorColor(label, "#00c853");
        } else if (value <= yellow_limit) {
            setIndicatorColor(label, "#ffd600");
        } else {
            setIndicatorColor(label, "#d50000");
        }
    };

    updateIndicator(dx_indicator_label_, abs_dx, 0.3, 1.0);
    updateIndicator(dy_indicator_label_, abs_dy, 0.3, 1.0);
    updateIndicator(dyaw_indicator_label_, abs_dyaw, 15.0, 30.0);
}

