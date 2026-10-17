#include "drone_warehouse/mainwindow.hpp"

#include "drone_warehouse/models.hpp"
#include "drone_warehouse/cargo_inspection_page.hpp"
#include "drone_warehouse/animal_inspection_page.hpp"
#include "drone_warehouse/collaboration_grid_view.hpp"
#include "drone_warehouse/shelf_info_dialog.hpp"
#include "drone_warehouse/connection_info_dialog.hpp"
#include "drone_warehouse/title_info_dialog.hpp"
#include "drone_warehouse/top_status_bar.hpp"
#include "drone_warehouse/color_palette.hpp"
#include "drone_warehouse/ros_manager.hpp"
#include "drone_warehouse/gpio_output.hpp"
#include "drone_warehouse/ai_diff_analyzer.hpp"
#include <drone_warehouse/video_dialog.hpp>
#include <drone_warehouse/video_replay_controller.hpp>
#include "drone_warehouse/shelf_panel_storage.hpp"
#include "drone_warehouse/parameter_config_dialog.hpp"

#include <algorithm>
#include <cmath>
#include <QDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QListWidget>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QRegularExpression>
#include <stdexcept>
#include <QProcess>
#include <QStringList>
#include <QCoreApplication>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{
// 只比较会影响货架画板、槽位映射和航点生成的配置。
// 飞行、伺服或相机参数变化时不应触发货架业务数据迁移。
bool shelfConfigurationEquals(
    const WarehouseConfig &left,
    const WarehouseConfig &right)
{
    const SlotGridConfig &left_grid = left.slot_grid;
    const SlotGridConfig &right_grid = right.slot_grid;
    if (left_grid.front_yaw_rad != right_grid.front_yaw_rad ||
        left_grid.back_yaw_rad != right_grid.back_yaw_rad ||
        left_grid.pose_y_min != right_grid.pose_y_min ||
        left_grid.pose_y_max != right_grid.pose_y_max ||
        left_grid.pose_z_min != right_grid.pose_z_min ||
        left_grid.pose_z_max != right_grid.pose_z_max ||
        left.shelves.size() != right.shelves.size())
    {
        return false;
    }

    for (int shelf_index = 0;
         shelf_index < left.shelves.size();
         ++shelf_index)
    {
        const ShelfConfig &left_shelf =
            left.shelves.at(shelf_index);
        const ShelfConfig &right_shelf =
            right.shelves.at(shelf_index);
        if (left_shelf.code != right_shelf.code ||
            left_shelf.display_name != right_shelf.display_name ||
            left_shelf.front_slot_prefix != right_shelf.front_slot_prefix ||
            left_shelf.back_slot_prefix != right_shelf.back_slot_prefix ||
            left_shelf.base_rect != right_shelf.base_rect ||
            left_shelf.height != right_shelf.height ||
            left_shelf.scene_color != right_shelf.scene_color ||
            left_shelf.button_status_color != right_shelf.button_status_color ||
            left_shelf.front_waypoint_y_m != right_shelf.front_waypoint_y_m ||
            left_shelf.back_waypoint_y_m != right_shelf.back_waypoint_y_m ||
            left_shelf.rows != right_shelf.rows ||
            left_shelf.columns != right_shelf.columns ||
            left_shelf.waypoint_row_z_m != right_shelf.waypoint_row_z_m ||
            left_shelf.waypoint_front_x_m != right_shelf.waypoint_front_x_m ||
            left_shelf.waypoint_back_x_m != right_shelf.waypoint_back_x_m ||
            left_shelf.pose_regions.size() != right_shelf.pose_regions.size())
        {
            return false;
        }

        for (int region_index = 0;
             region_index < left_shelf.pose_regions.size();
             ++region_index)
        {
            const ShelfPoseRegionConfig &left_region =
                left_shelf.pose_regions.at(region_index);
            const ShelfPoseRegionConfig &right_region =
                right_shelf.pose_regions.at(region_index);
            if (left_region.side != right_region.side ||
                left_region.x_min != right_region.x_min ||
                left_region.x_max != right_region.x_max ||
                left_region.yaw_min != right_region.yaw_min ||
                left_region.yaw_max != right_region.yaw_max)
            {
                return false;
            }
        }
    }
    return true;
}

QString translatedDroneAction(const QString &action_name)
{
    const QString action = action_name.trimmed().toLower();
    if (action == "takeoff") {
        return "起飞";
    }
    if (action == "hover") {
        return "悬停";
    }
    if (action == "visual_servo") {
        return "伴飞";
    }
    if (action == "move") {
        return "移动";
    }
    if (action == "drop") {
        return "抛投";
    }
    if (action == "return") {
        return "返航";
    }
    if (action == "land") {
        return "降落";
    }
    if (action == "finished") {
        return "任务完成";
    }
    if (action == "stopped") {
        return "已停止";
    }
    if (action == "queued" || action == "waiting_start") {
        return "准备中";
    }
    if (action.isEmpty() || action == "idle") {
        return "待命";
    }
    return action_name;
}

QString translatedCarRouteState(const QString &route_state)
{
    const QString state = route_state.trimmed().toUpper();
    if (state == "IDLE") {
        return "等待状态";
    }
    if (state == "LEAVING_A" || state == "TO_B") {
        return "正在前往 B 点";
    }
    if (state == "TO_C") {
        return "正在前往 C 点";
    }
    if (state == "TO_D") {
        return "正在前往 D 点";
    }
    if (state == "TO_A") {
        return "正在前往 A 点";
    }
    if (state == "ALIGNING_AT_A") {
        return "正在 A 点对正";
    }
    if (state == "COMPLETE") {
        return "路线已完成";
    }
    return state.isEmpty() ? QString("等待状态") : state;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      config_(createDefaultWarehouseConfig())
{
    QString config_error;
    if (!loadWarehouseConfig(config_, &config_error))
    {
        throw std::invalid_argument(config_error.toStdString());
    }

    /*********************ros移植部分***********************/
    // 注册跨线程 Qt 信号里会用到的 WorldCoord 元类型。
    // 原 drone_qt 工程里 returnWorldGroupUpdated 会跨线程发 QVector<WorldCoord>，
    // 这里先把类型注册好，避免后面一旦启用这条信号链时 Qt 不认识这个自定义类型。
    qRegisterMetaType<WorldCoord>("WorldCoord");
    qRegisterMetaType<QVector<WorldCoord>>("QVector<WorldCoord>");
    /******************************************************/
    //setWindowFlags(Qt::Window | Qt::FramelessWindowHint);//去掉主窗口系统标题栏，不再显示上方 warehouse_gcs 那一层
    //resize(1024, 600);
    //setFixedSize(1024, 540);//设置初始窗口大小
    showFullScreen();
    activateWindow();
    setWindowFlags(Qt::FramelessWindowHint);
    raise();

    setupUi();
    setupFloatingWidgets();
    applyInspectionProject(config_.inspection_project);
    video_replay_controller_ = new VideoReplayController(QString(), this, this);
    setupConnections();
    connect(top_status_bar_, &TopStatusBar::replayButtonClicked, this, [this]() { video_replay_controller_->showReplayDialog(); });
    applyWindowStyle();
    setupDemoData();
    resetWorldBodyTransform();

    /*********************ros移植部分***********************/
    // 本轮先把时间触发接口和状态变量接进来，但按用户要求默认关闭。
    // 这样后面如果你要补一个“设置触发时刻”的入口，只需要把这两个值改掉即可。
    mission_trigger_time_text_ = "";
    mission_time_trigger_enabled_ = true;
    top_status_bar_->setTriggerTime(mission_trigger_time_text_);//传入想要定的时间
    top_status_bar_->setTimeTriggerEnabled(mission_time_trigger_enabled_);//传入是否开启时间定时
    /******************************************************/
    //top_status_bar_->setConnected(true);
    updateOverlayGeometry();

    /*********************ros移植部分***********************/
    // 界面和信号槽都准备好之后，再启动 RosManager 的 spin 线程。
    // 这样一来，后面 ROS 回调一到，就能立刻把状态发到已经存在的 Qt 控件上。
    if (ros_manager_)
    {
        ros_manager_->start();
        ros_manager_->publishIndustrialCameraParams(config_.industrial_camera);
    }
    /******************************************************/
}

void MainWindow::setupUi()
{
    central_container_ = new QWidget(this);
    //设置主容器
    setCentralWidget(central_container_);

    //创建主场景视图和顶部状态栏，并把它们放在主容器里，方便统一管理布局和坐标
    // 两套画板使用同一块主区域；项目切换时只显示其中一套。
    // 每个页面在构造时创建自己的画板、日志和局部控件，MainWindow 不再逐个持有。
    cargo_page_ = new CargoInspectionPage(central_container_);
    animal_page_ = new AnimalInspectionPage(central_container_);
    collaboration_grid_view_ = new CollaborationGridView(central_container_);
    cargo_page_->setGeometry(central_container_->rect());
    animal_page_->setGeometry(central_container_->rect());
    collaboration_grid_view_->setGeometry(central_container_->rect());
    cargo_page_->hide();
    animal_page_->hide();
    collaboration_grid_view_->hide();

    top_status_bar_ = new TopStatusBar(central_container_);

    //这里传入 config_.slot_grid，确保弹窗里能正确显示当前仓库的槽位结构和航点映射。
    shelf_info_dialog_ = new ShelfInfoDialog(config_.shelves, this);

    // config_.ros 已经保存了当前连接方式对应的话题和服务名称。
    // bridge_ros 只供 ground_link_bridge 使用，地面站不在两者之间临时选择。
    ros_manager_ = new RosManager(config_.ros, this);

}

void MainWindow::setupFloatingWidgets()
{
    /***********************日志控件*************************/

    // 日志面板已经随原有注释一起移动到 CargoInspectionPage 和 AnimalInspectionPage。
    // 定时器仍属于 MainWindow，因为它控制的是跨项目的临时运行提示。
    clock_timer_ = new QTimer(this);//新建定时器
    clock_timer_->start(5000);

    /*******************************************************/

    /*********************悬浮姿态控件***********************/

    attitude_panel_ = new QWidget(central_container_);
    auto *attitude_layout = new QGridLayout(attitude_panel_);
    attitude_panel_->setObjectName("attitudeSwitchPanel");
    attitude_layout->setContentsMargins(12, 12, 12, 12);//姿态面板内部边框留白

    attitude_layout->addWidget(new QLabel("高度", attitude_panel_), 0, 0);
    altitude_value_label_ = new QLabel("0.0 m", attitude_panel_);
    attitude_layout->addWidget(altitude_value_label_, 0, 1);

    attitude_layout->addWidget(new QLabel("航向", attitude_panel_), 1, 0);
    yaw_value_label_ = new QLabel("0.0°", attitude_panel_);
    attitude_layout->addWidget(yaw_value_label_, 1, 1);

    attitude_layout->addWidget(new QLabel("电量", attitude_panel_), 2, 0);
    battery_value_label_ = new QLabel("N/A", attitude_panel_);
    attitude_layout->addWidget(battery_value_label_, 2, 1);

    attitude_layout->addWidget(new QLabel("模式", attitude_panel_), 3, 0);
    mode_value_label_ = new QLabel("MODE_UNKNOWN", attitude_panel_);
    attitude_layout->addWidget(mode_value_label_, 3, 1);


    /*******************************************************/

    /*********************空地协同悬浮姿态控件***********************/

    drone_attitude_panel_ = new QWidget(central_container_);
    auto *drone_attitude_layout = new QGridLayout(drone_attitude_panel_);
    drone_attitude_panel_->setObjectName("droneattitudeSwitchPanel");
    drone_attitude_layout->setContentsMargins(12, 12, 12, 12);//姿态面板内部边框留白

    drone_attitude_layout->addWidget(new QLabel("高度", drone_attitude_panel_), 0, 0);
    drone_z_value_label_ = new QLabel("0.0 m", drone_attitude_panel_);
    drone_attitude_layout->addWidget(drone_z_value_label_, 0, 1);

    drone_attitude_layout->addWidget(new QLabel("坐标", drone_attitude_panel_), 1, 0);
    drone_xy_value_label_ = new QLabel("N/A", drone_attitude_panel_);
    drone_attitude_layout->addWidget(drone_xy_value_label_, 1, 1);

    drone_attitude_layout->addWidget(new QLabel("航向", drone_attitude_panel_), 2, 0);
    drone_yaw_value_label_ = new QLabel("0.0°", drone_attitude_panel_);
    drone_attitude_layout->addWidget(drone_yaw_value_label_, 2, 1);

    drone_attitude_layout->addWidget(new QLabel("电量", drone_attitude_panel_), 3, 0);
    drone_battery_value_label_ = new QLabel("N/A", drone_attitude_panel_);
    drone_attitude_layout->addWidget(drone_battery_value_label_, 3, 1);

    drone_attitude_layout->addWidget(new QLabel("模式", drone_attitude_panel_), 4, 0);
    drone_move_value_label_ = new QLabel("none", drone_attitude_panel_);
    drone_attitude_layout->addWidget(drone_move_value_label_, 4, 1);



    car_attitude_panel_ = new QWidget(central_container_);
    auto *car_attitude_layout = new QGridLayout(car_attitude_panel_);
    car_attitude_panel_->setObjectName("carattitudeSwitchPanel");
    car_attitude_layout->setContentsMargins(12, 12, 12, 12);//姿态面板内部边框留白

    car_attitude_layout->addWidget(new QLabel("高度", car_attitude_panel_), 0, 0);
    car_z_value_label_ = new QLabel("0.0 m", car_attitude_panel_);
    car_attitude_layout->addWidget(car_z_value_label_, 0, 1);

    car_attitude_layout->addWidget(new QLabel("坐标", car_attitude_panel_), 1, 0);
    car_xy_value_label_ = new QLabel("N/A", car_attitude_panel_);
    car_attitude_layout->addWidget(car_xy_value_label_, 1, 1);

    car_attitude_layout->addWidget(new QLabel("航向", car_attitude_panel_), 2, 0);
    car_yaw_value_label_ = new QLabel("0.0°", car_attitude_panel_);
    car_attitude_layout->addWidget(car_yaw_value_label_, 2, 1);

    // 空地协同右下角只显示两行任务语义状态，不重复坐标和姿态信息。
    collaboration_status_panel_ = new QWidget(central_container_);
    collaboration_status_panel_->setObjectName("collaborationStatusPanel");
    auto *collaboration_status_layout =
        new QGridLayout(collaboration_status_panel_);
    collaboration_status_layout->setContentsMargins(16, 12, 16, 12);
    collaboration_status_layout->setHorizontalSpacing(16);
    collaboration_status_layout->setVerticalSpacing(10);

    collaboration_status_layout->addWidget(
        new QLabel("无人机状态", collaboration_status_panel_), 0, 0);
    collaboration_drone_status_value_label_ =
        new QLabel("待命", collaboration_status_panel_);
    collaboration_status_layout->addWidget(
        collaboration_drone_status_value_label_, 0, 1);

    // collaboration_status_layout->addWidget(
    //     new QLabel("无人车状态", collaboration_status_panel_), 1, 0);
    // collaboration_car_status_value_label_ =
    //     new QLabel("未连接", collaboration_status_panel_);
    // collaboration_status_layout->addWidget(
    //     collaboration_car_status_value_label_, 1, 1);

    collaboration_status_panel_->hide();

    // Collaboration 不复用 Cargo 的日志控件。Cargo 页面在协同模式下是隐藏的，
    // 如果继续写入 Cargo 日志，操作员虽然收到消息却无法在当前界面看到。
    collaboration_log_panel_ = new QWidget(central_container_);
    collaboration_log_panel_->setObjectName("collaborationLogPanel");
    auto *collaboration_log_layout =
        new QVBoxLayout(collaboration_log_panel_);
    collaboration_log_layout->setContentsMargins(12, 10, 12, 12);
    collaboration_log_layout->setSpacing(8);

    auto *collaboration_log_title =
        new QLabel("运行日志", collaboration_log_panel_);
    collaboration_log_title->setObjectName("collaborationLogTitle");
    collaboration_log_layout->addWidget(collaboration_log_title);

    collaboration_run_log_view_ =
        new QPlainTextEdit(collaboration_log_panel_);
    collaboration_run_log_view_->setReadOnly(true);
    collaboration_run_log_view_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    collaboration_log_layout->addWidget(collaboration_run_log_view_, 1);

    // 控制端舵机抛投保持 3 秒；这段时间优先显示“抛投”，结束后恢复最新动作。
    drone_drop_status_timer_ = new QTimer(this);
    drone_drop_status_timer_->setSingleShot(true);
    drone_drop_status_timer_->setInterval(3000);
    connect(drone_drop_status_timer_, &QTimer::timeout, this, [this]() {
        collaboration_drone_status_value_label_->setText(
            translatedDroneAction(latest_drone_action_name_));
    });

    /*******************************************************/

    /*********************模式切换滑块***********************/

    // 模式和视角滑块已移动到 CargoInspectionPage，信号连接也由该页面管理。

    /*******************************************************/

    top_status_bar_->raise();//确保悬浮控件在主场景视图上面
    attitude_panel_->raise();//确保姿态面板在主场景视图上面
    collaboration_status_panel_->raise();
    collaboration_log_panel_->raise();
}

void MainWindow::setupConnections()
{
    connect(top_status_bar_, &TopStatusBar::exitRequested, this, [this]() {
        close();
    });

    connect(clock_timer_, &QTimer::timeout, this, [this]() {//每秒触发刷新一次日志文本
        clearRunLogs();
        clock_timer_->stop();
    });

    // 视图模式、3D 视角和 2D 视角的滑块连接已移动到 CargoInspectionPage。

    // 点击连接状态按钮后，打开连接方式和数传串口配置窗口。
    connect(top_status_bar_, &TopStatusBar::connectionButtonClicked, this, [this]() {
        const QPoint button_bottom_left =
            top_status_bar_->connectionButtonBottomLeftGlobal();
        const int margin = 20;

        ConnectionInfoDialog dialog(config_, this);
        dialog.adjustSize();
        dialog.move(
            button_bottom_left.x(),
            button_bottom_left.y() + margin);

        if (dialog.exec() == QDialog::Accepted)
        {
            // 弹窗已经完成 JSON 保存，这里同步主窗口内存中的配置。
            // RosManager 和数传串口均在启动时创建，因此新连接方式重启后生效。
            config_ = dialog.savedConfig();
            appendRunLog(
                "连接配置已保存，重启地面站后生效");
        }
    });

    // 点击标题按钮后，打开详细配置窗口。
    connect(top_status_bar_, &TopStatusBar::titleClicked, this, [this]() {
        const QPoint button_bottom_left =
            top_status_bar_->titleButtonBottomLeftGlobal();
        const int margin = 20;

        TitleInfoDialog dialog(this);
        dialog.adjustSize();
        dialog.move(
            button_bottom_left.x(),
            button_bottom_left.y() + margin);

        if (dialog.exec() == QDialog::Accepted)
        {
            // 弹窗已经完成 JSON 保存，这里同步主窗口内存中的配置。
            ParameterConfigDialog parameter_dialog(
                config_, this);

            // 参数页点击“启用”后保持打开，同时立即更新地面站内存配置和相机参数。
            connect(&parameter_dialog, &ParameterConfigDialog::configApplied,
                    this, [this](const WarehouseConfig &applied_config) {
                        const WarehouseConfig previous_config = config_;
                        config_ = applied_config;
                        applyInspectionProject(config_.inspection_project);
                        applyShelfConfigurationToUi(previous_config);
                        ros_manager_->publishIndustrialCameraParams(
                            config_.industrial_camera);
                    });

            // 让弹窗全屏显示，方便在小屏幕上操作。
            parameter_dialog.setWindowState(
                parameter_dialog.windowState() |
                Qt::WindowFullScreen);

            //在主窗口上方显示弹窗
            parameter_dialog.exec();
            // 配置已经由 configApplied 信号即时同步，不再等待 QDialog::Accepted。
            //
            // run_log_view_->appendPlainText(
            //     "仓储智航参数已启用");
        }
    });

    //连接顶部状态栏货架信息点击信号，打开货架信息窗口
    connect(top_status_bar_, &TopStatusBar::shelfButtonClicked, this, [this]() {
        //取出货物信息左下角的x，y坐标
        const QPoint button_bottom_left = top_status_bar_->shelfButtonBottomLeftGlobal();

        const int margin = 20;//让弹窗和顶部状态栏之间留一点空隙

        shelf_info_dialog_->adjustSize();
        shelf_info_dialog_->move(button_bottom_left.x(), button_bottom_left.y() + margin);//先把弹窗移动到货物信息下方
        shelf_info_dialog_->show();//显示弹窗
        shelf_info_dialog_->raise();//把弹窗提升到最前面
        shelf_info_dialog_->activateWindow();//让弹窗获取焦点，方便后续继续操作
    });
    connect(top_status_bar_, &TopStatusBar::displayButtonClicked, this, [this]() {
        if (!video_dialog_) {
            video_dialog_ = new VideoDialog(QString(), this);
        }
        if (video_dialog_->isVisible()) { video_dialog_->raise(); video_dialog_->activateWindow(); } else { video_dialog_->show(); }
    });

    connect(shelf_info_dialog_, &ShelfInfoDialog::slotDoubleClicked,
        this,
        [this](int shelf_index, const QString &side, int row, int col)
        {
            showShelfSlotImage(shelf_index, side, row, col);
        });

    // 地面站手动入库链路：
    // 1. 用户先在 ShelfInfoDialog 里选中一个格子。
    // 2. Dialog 通过串口拿到扫码文本后，发出 `manualStockInScanned(...)`。
    // 3. MainWindow 在这里接住信号，并把类别/包裹编号真正写回主数据 `shelf_panel_data_`。
    // 4. 写回完成后，再统一调用 `setShelfPanelData(...)` 把最新结果刷新回弹窗。
    connect(shelf_info_dialog_, &ShelfInfoDialog::manualStockInScanned,
        this,
        [this](int shelf_index, const QString &side, int row, int col,
               const QString &category_id, const QString &package_id)
        {
            applyManualStockIn(shelf_index, side, row, col, category_id, package_id);
        });

    // 地面站手动出库链路：
    // 1. 用户在弹窗里点击“出库”。
    // 2. Dialog 把“当前选中的货架/前后面/行列”发给 MainWindow。
    // 3. MainWindow 在这里统一执行清空，避免 Dialog 和主数据各改各的导致状态分叉。
    connect(shelf_info_dialog_, &ShelfInfoDialog::manualStockOutRequested,
        this,
        [this](int shelf_index, const QString &side, int row, int col, const QString &category_id, const QString &package_id)
        {
            applyManualStockOut(shelf_index, side, row, col, category_id, package_id);
        });

    /*********************ros通信相关***********************/
    
    if (ros_manager_)
    {
        connect(top_status_bar_, &TopStatusBar::scheduledcheckbuttonnClicked, 
            this, 
            [this](const QString &mission_trigger_time_text) {
            if(mission_trigger_time_text_flag_ == 1)
            {
                mission_trigger_time_text_ = mission_trigger_time_text;
                appendRunLog(QString("已设置定时巡检：%1").arg(mission_trigger_time_text_));
                mission_trigger_time_text_flag_ = 0;
                top_status_bar_->setTriggerTime(mission_trigger_time_text_);
                triggerMissionUpload("time");
                clock_timer_->start(5000);
            }
            else
            {
                mission_trigger_time_text_ = "";
                appendRunLog(QString("已关闭定时巡检"));
                mission_trigger_time_text_flag_ = 1;
                top_status_bar_->setTriggerTime(mission_trigger_time_text_);
                clock_timer_->start(5000);
            }
        });

        connect(top_status_bar_, &TopStatusBar::aiAnalyzeButtonClicked, this, [this]() {
            runClaudeApiDiffAnalysis();
        });

        connect(top_status_bar_, &TopStatusBar::executeButtonClicked,
            this,
            [this]()
            {
                QString trigger_source = "button";
                switch (config_.inspection_project)
                {
                case InspectionProject::Cargo:
                    trigger_source = "cargo";
                    break;
                case InspectionProject::Animal:
                    trigger_source = "animal";
                    break;
                case InspectionProject::Collaboration:
                    trigger_source = "collaboration";
                    break;
                }
                triggerMissionUpload(trigger_source);
            });

        // 空地协同顶部只暴露这两个小车控制命令。RosManager 会再次校验字符串，
        // 防止其他调用点发布文档以外的控制模式。
        connect(top_status_bar_, &TopStatusBar::carPauseRequested,
            this,
            [this]()
            {
                ros_manager_->publishCarControlMode("DISABLED");
                appendRunLog("已发送小车暂停命令");
            });

        connect(top_status_bar_, &TopStatusBar::carResumeRequested,
            this,
            [this]()
            {
                ros_manager_->publishCarControlMode("AUTO");
                appendRunLog("已发送小车恢复命令");
            });

        // 小车路线状态既用于界面显示，也作为空地协同任务的启动信号。
        // IDLE 只负责重新布防；同一轮路线中的 TO_B、TO_C 等多个非 IDLE
        // 状态共用一个锁存值，因此整轮小车任务只会触发一次无人机任务。
        connect(ros_manager_, &RosManager::carRouteStateReceived,
            this,
            [this](const QString &state)
            {
                const QString normalized_state = state.trimmed().toUpper();
                collaboration_car_status_value_label_->setText(
                    translatedCarRouteState(state));

                if (normalized_state == "IDLE")
                {
                    car_route_active_latched_ = false;
                    return;
                }
                if (normalized_state.isEmpty() ||
                    config_.inspection_project != InspectionProject::Collaboration ||
                    car_route_active_latched_)
                {
                    return;
                }

                car_route_active_latched_ = true;
                appendRunLog(
                    QString("小车路线进入 %1，开始初始化空地协同任务")
                        .arg(normalized_state));
                triggerMissionUpload("collaboration");
            },
            Qt::QueuedConnection);

        connect(top_status_bar_, &TopStatusBar::triggerTimeReached, this, [this](const QString &) {
            triggerMissionUpload("time");
        });

        connect(top_status_bar_, &TopStatusBar::waypointButtonClicked, this, [this]() {
            triggerMissionUpload("waypoint");
        });

        //清空航点
        connect(shelf_info_dialog_, &ShelfInfoDialog::clearWaypointRequested, this, [this]() {
            clearWaypointRequest();
        });

        //设置航点
        connect(shelf_info_dialog_, &ShelfInfoDialog::setWaypointRequested, 
            this, 
            [this](int shelf_index, const QString &side, int row, int col)
            {
                setWaypointRequest(shelf_index, side, row, col);
            });

        //查看任务yaml上传服务返回的内容
        connect(ros_manager_, &RosManager::missionUploadFinished,
            this,
            [this](bool success, const QString &message, const QString &saved_path)
            {
                handleMissionUploadFinished(success, message, saved_path);
            },
            Qt::QueuedConnection);

        //查看起飞启动服务返回的内容
        connect(ros_manager_, &RosManager::commandResult,
            this,
            [this](bool success, const QString &message)
            {
                //根据命令执行结果的成功与否，更新界面上的结果标签文本，显示相关消息
                updateCommandResult(success, message);
                appendRunLog(QString("%1").arg(message));
                if (!success && collaboration_mission_active_)
                {
                    // Start 失败后释放任务锁；路线锁仍保持到小车回到 IDLE，
                    // 防止同一轮路线状态持续发布时不断自动重试起飞。
                    collaboration_mission_active_ = false;
                    collaboration_task_running_seen_ = false;
                }

                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);

        //查看停止服务返回的内容
        connect(ros_manager_, &RosManager::stopcommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    // path_ready_ = false;
                    // waiting_push_result_ = false;
                    // start_button_->setEnabled(false);
                    // delta_result_ = true;
                    //push_button_->setEnabled(true);
                    appendRunLog(QString("%1").arg(message));
                    clock_timer_->start(5000);
                }
            },
            Qt::QueuedConnection);

        //查看offboard启动服务返回的内容
        connect(ros_manager_, &RosManager::offboardCommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if (!success && collaboration_mission_active_)
                {
                    collaboration_mission_active_ = false;
                    collaboration_task_running_seen_ = false;
                    appendRunLog(message.isEmpty()
                        ? "Offboard 启动失败，空地协同任务未启动"
                        : message);
                }

                if (success)
                {
                    // Animal 不在这里直接调用 startTask()。控制程序返回的有效路线
                    // 也到达后，tryStartAnimalTask() 才会按顺序打印路线并启动。
                    if (animal_route_start_pending_)
                    {
                        animal_offboard_ready_ = true;
                        appendRunLog(
                            message.isEmpty() ? "Offboard 启动成功" : message);
                        tryStartAnimalTask();
                    }
                    else if (collaboration_mission_active_)
                    {
                        // 固定 ground_station.yaml 不会回传需要等待的动态路线。
                        // Offboard 成功后直接调用 Start，才会真正执行预设任务。
                        appendRunLog(
                            message.isEmpty() ? "Offboard 启动成功，正在启动空地协同任务" : message);
                        ros_manager_->startTask();
                    }
                    else
                    {
                        appendRunLog(QString("%1").arg(message));
                    }
                }
                else if (animal_route_start_pending_)
                {
                    // Offboard 失败后结束本轮等待，后续残留路线消息不能误启动任务。
                    animal_route_start_pending_ = false;
                    animal_offboard_ready_ = false;
                    animal_returned_route_.clear();
                    appendRunLog(
                        message.isEmpty() ? "Offboard 启动失败，任务未启动" : message);
                }
            },
            Qt::QueuedConnection);












        //连接rosmanager发出的无人机状态信号
        connect(ros_manager_, &RosManager::statusUpdated,
            this,
            [this](bool connected, float battery_percent, int flight_mode, bool armed, const QString &task_name)
            {
                updateStatus(connected, battery_percent, flight_mode, armed, task_name);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的动作状态信号
        connect(ros_manager_, &RosManager::action_statusUpdated,
            this,
            [this](bool task_running, int action_step, int action_num, const QString &action_name)
            {
                action_updateStatus(task_running, action_step, action_num, action_name);
            },
            Qt::QueuedConnection);

        //连接ROS管理器的状态更新信号到一个lambda槽，用于解析状态消息并更新界面上的状态标签
        connect(ros_manager_, &RosManager::barcodeCaptured,
            this,
            [this](const QString &barcode,
                const QByteArray &image_data,
                const QString &image_format,
                const QString &time_text)
            {
                appendBarcodeRecord(barcode, image_data, image_format, time_text);
            },
            Qt::QueuedConnection);

        connect(ros_manager_, &RosManager::visionBarcodeCaptured,
            this,
            [this](const QString &barcode, const QString &time_text)
            {
                appendVisionBarcodeCount(barcode, time_text);
            },
            Qt::QueuedConnection);

        // 视觉伺服动作结束后，把成功跟踪的目标记录到 Animal 结果区。
        connect(ros_manager_, &RosManager::visionServoStatusUpdated,
            this,
            [this](bool active,
                   const QString &state,
                   const QString &requested_target_id,
                   const QString &tracked_target_id,
                   const QString &detail,
                   const QString &time_text)
            {
                handleVisionServoStatus(
                    active, state, requested_target_id,
                    tracked_target_id, detail, time_text);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的无人机位置信号
        connect(ros_manager_, &RosManager::positionUpdated,
            this,
            [this](double x, double y, double z, double qx, double qy, double qz, double qw)
            {
                // 上游只透传 MAVROS local_position。地面站先把原始 world_enu
                // 位姿统一转换成控制程序使用的 world_body，再交给三个画板。
                GroundTfPose world_enu_pose;
                world_enu_pose.x = x;
                world_enu_pose.y = y;
                world_enu_pose.z = z;
                world_enu_pose.qx = qx;
                world_enu_pose.qy = qy;
                world_enu_pose.qz = qz;
                world_enu_pose.qw = qw;

                GroundTfPose world_body_pose;
                if (!ground_world_body_tf_.update(
                        world_enu_pose, world_body_pose))
                {
                    // 与控制端一致，最初 10 帧只用于确定固定原点和初始朝向。
                    return;
                }

                const double body_siny_cosp = 2.0 * (
                    world_body_pose.qw * world_body_pose.qz +
                    world_body_pose.qx * world_body_pose.qy);
                const double body_cosy_cosp = 1.0 - 2.0 * (
                    world_body_pose.qy * world_body_pose.qy +
                    world_body_pose.qz * world_body_pose.qz);
                const double body_yaw_deg =
                    std::atan2(body_siny_cosp, body_cosy_cosp) *
                    180.0 / M_PI;

                // Cargo 保留当前场景坐标逻辑：
                //   world_body (0, 0) -> 场景像素 (150, 100)；
                //   1 m -> 100 px；x+ 向场景右，y+ 向场景上。
                // Current common mapping: x+ is up and y+ is left.
                scene_data_.drone_state.pose.x =
                    -world_body_pose.y * 100.0 + 150.0;
                scene_data_.drone_state.pose.y =
                    -world_body_pose.x * 100.0 + 100.0;
                scene_data_.drone_state.pose.z = world_body_pose.z;
                scene_data_.drone_state.pose.yaw = body_yaw_deg;
                cargo_page_->setSceneData(scene_data_);

                // Animal 保留现有 setPosition 内部的轴映射和右下角网格原点。
                animal_page_->setPosition(
                    world_body_pose.x,
                    world_body_pose.y,
                    world_body_pose.z);

                // Collaboration 直接使用 world_body：x+ 向上，y+ 向左。
                collaboration_grid_view_->setdronePosition(
                    world_body_pose.x,
                    world_body_pose.y,
                    world_body_pose.z);

                // 顶部通用姿态和 Collaboration 右侧信息统一显示 world_body。
                altitude_value_label_->setText(
                    QString::number(world_body_pose.z, 'f', 1) + " m");
                yaw_value_label_->setText(
                    QString::number(body_yaw_deg, 'f', 1) + "°");
                drone_z_value_label_->setText(
                    QString::number(world_body_pose.z, 'f', 1) + " m");
                drone_xy_value_label_->setText(
                    QString("(%1, %2) m")
                        .arg(world_body_pose.x, 0, 'f', 1)
                        .arg(world_body_pose.y, 0, 'f', 1));
                drone_yaw_value_label_->setText(
                    QString::number(body_yaw_deg, 'f', 1) + "°");
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的无人车位置信号
        connect(ros_manager_, &RosManager::carpositionUpdated,
            this,
            [this](double x, double y, double z, double qx, double qy, double qz, double qw)
            {
                const double siny_cosp = 2.0 * (qw * qz + qx * qy);
                const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                const double yaw = std::atan2(siny_cosp, cosy_cosp);          // 弧度
                const double yaw_deg = yaw * 180.0 / M_PI;                    // 角度

                car_z_value_label_->setText(QString::number(z, 'f', 1) + " m");
                car_xy_value_label_->setText(
                QString("(%1, %2) m")
                    .arg(x, 0, 'f', 1)
                    .arg(y, 0, 'f', 1));
                car_yaw_value_label_->setText(QString::number(yaw_deg, 'f', 1) + "°");
                collaboration_grid_view_->setcarPosition(x, y, z);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的x,y偏移信号
        connect(ros_manager_, &RosManager::deltaUpdated,
            this,
            [this](double dx, double dy, double dyaw, bool valid)
            {
                updateDelta(dx, dy, dyaw, valid);
            },
            Qt::QueuedConnection);

        //链接判断控制程序返回内容
        connect(ros_manager_, &RosManager::pathReadyChanged,
            this,
            [this](bool ready)
            {
                updatePathReadyState(ready);
            },
            Qt::QueuedConnection);

        // 接收控制程序回传路线。Animal 要等 Offboard 和路线都成功后再启动；
        // Cargo 保留原有的“收到路线后启动任务”流程。
        connect(ros_manager_, &RosManager::returnWorldGroupUpdated,
            this,
            [this](const QVector<WorldCoord> &points)
            {
                if (collaboration_mission_active_ ||
                    config_.inspection_project == InspectionProject::Collaboration)
                {
                    // 空地协同使用机载端固定 YAML，不依赖控制程序回传路线。
                    // 忽略这里的消息，防止 Offboard 回调已经 Start 后再次启动。
                    return;
                }

                if (config_.inspection_project == InspectionProject::Animal)
                {
                    // 只有点击“执行”建立了本轮等待状态，Animal 路线才有效。
                    // 成功启动后 pending 会被清除，重复发布的路线将在这里被忽略。
                    if (!animal_route_start_pending_)
                    {
                        return;
                    }

                    if (points.isEmpty())
                    {
                        appendRunLog(
                            "控制程序回传路线为空，继续等待有效路线");
                        return;
                    }

                    // 路线可能早于 Offboard 服务结果到达，所以先缓存，再统一检查。
                    animal_returned_route_ = points;
                    tryStartAnimalTask();
                    return;
                }

                ros_manager_->startTask();
                //run_log_view_->appendPlainText("初始化成功，准备巡检");
                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);
    }
}

/*********************ros移植部分***********************/

void MainWindow::triggerMissionUpload(const QString &trigger_source)
{
    if (!ros_manager_)
    {
        appendRunLog("初始化失败,rosmanager未就绪");
        //clock_timer_->start(5000);
        return;
    }

    // 空地协同任务从第一次 S4/执行触发开始锁定整条启动链路。任务运行期间
    // 不允许 S4、定时触发或其他上传入口改写机载端当前任务路径。
    if (collaboration_mission_active_)
    {
        appendRunLog("空地协同任务正在执行，已忽略重复启动请求");
        return;
    }

    if (mission_upload_in_progress_)
    {
        appendRunLog("无法初始化");
        //clock_timer_->start(5000);
        return;
    }

    // 上传服务返回后 mission_upload_in_progress_ 会先清除，但 Animal 还可能在等待
    // Offboard 或回传路线。此时再次点击执行会造成两轮状态交叉，所以直接拦截。
    if (trigger_source == "animal" && animal_route_start_pending_)
    {
        appendRunLog(
            "动物巡检正在等待 Offboard 和回传路线，请勿重复执行");
        return;
    }

    const MissionConfig &mission = config_.mission;
    drone_msgs::msg::MissionSummary summary;
    summary.takeoff_altitude = mission.takeoff_altitude;
    summary.move_altitude = mission.move_altitude;
    summary.start_altitude = mission.start_altitude;
    summary.yaw = mission.yaw;
    summary.tolerance = mission.tolerance;
    summary.yaw_tolerance_deg = mission.yaw_tolerance_deg;
    summary.max_xy_speed_mps = mission.max_xy_speed_mps;
    summary.max_z_speed_mps = mission.max_z_speed_mps;
    summary.max_yaw_rate_deg_s = mission.max_yaw_rate_deg_s;
    summary.takeoff_hover_duration = mission.takeoff_hover_duration;
    summary.landing_hover_duration = mission.landing_hover_duration;
    summary.move_hover_duration = mission.move_hover_duration;
    summary.add_hover_between_takeoff = mission.add_hover_between_takeoff;
    summary.add_hover_between_landing = mission.add_hover_between_landing;
    summary.add_hover_between_moves = mission.add_hover_between_moves;
    summary.auto_start_mission = mission.auto_start_mission;
    summary.frame = mission.frame.toStdString();

    // The ROS message is structured even though YAML writes these keys flat under system.
    const VisualServoConfig &visual = config_.visual_servo;
    summary.visual_servo.enabled = visual.enabled;
    summary.visual_servo.target_id = visual.target_id.toStdString();
    summary.visual_servo.require_confirmed = visual.require_confirmed;
    summary.visual_servo.image_x_axis = visual.image_x_axis.toStdString();
    summary.visual_servo.image_y_axis = visual.image_y_axis.toStdString();
    summary.visual_servo.image_x_sign = visual.image_x_sign;
    summary.visual_servo.image_y_sign = visual.image_y_sign;
    summary.visual_servo.kp_x = visual.kp_x;
    summary.visual_servo.ki_x = visual.ki_x;
    summary.visual_servo.kd_x = visual.kd_x;
    summary.visual_servo.kp_y = visual.kp_y;
    summary.visual_servo.ki_y = visual.ki_y;
    summary.visual_servo.kd_y = visual.kd_y;
    summary.visual_servo.integral_limit = visual.integral_limit;
    summary.visual_servo.filter_alpha = visual.filter_alpha;
    summary.visual_servo.enter_tolerance_x = visual.enter_tolerance_x;
    summary.visual_servo.enter_tolerance_y = visual.enter_tolerance_y;
    summary.visual_servo.exit_tolerance_x = visual.exit_tolerance_x;
    summary.visual_servo.exit_tolerance_y = visual.exit_tolerance_y;
    summary.visual_servo.settle_time_s = visual.settle_time_s;
    summary.visual_servo.acquire_timeout_s = visual.acquire_timeout_s;
    summary.visual_servo.lost_timeout_s = visual.lost_timeout_s;
    summary.visual_servo.overall_timeout_s = visual.overall_timeout_s;
    summary.visual_servo.max_body_speed_mps = visual.max_body_speed_mps;
    summary.visual_servo.continue_on_timeout = visual.continue_on_timeout;

    if (trigger_source == "animal")
    {
        // Animal 直接使用画板当前路线，不依赖货架航点或机载端静态路线。
        const QVector<WorldCoord> animal_points =
            animal_page_->plannedWorldPoints(
                mission.move_altitude,
                mission.yaw);
        if (animal_points.isEmpty())
        {
            appendRunLog(
                "动物巡检路线为空，请至少保留一个可通行格");
            return;
        }

        summary.compress_straight_segments =
            mission.compress_waypoint_segments;

        // 从点击执行开始建立一轮新的 Animal 等待状态。旧路线必须清空，
        // 防止上一轮回传数据被当成本轮结果使用。
        animal_route_start_pending_ = true;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(animal_points, summary);
    }
    else if (trigger_source == "collaboration")
    {
        // Collaboration 暂时没有独立航点列表，明确上传空路线并使用普通任务压缩策略。
        // 独立分支可避免以后增加空地协同任务时误改 Cargo 流程。
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        // false 配合空路线是机载端选择固定 ground_station.yaml 的明确协议。
        // 不能读取普通任务压缩配置，否则 true 会误入动态 YAML 生成分支。
        summary.compress_straight_segments = false;
        QVector<WorldCoord> empty_points;
        collaboration_mission_active_ = true;
        collaboration_task_running_seen_ = false;
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(empty_points, summary);
    }
    else if (trigger_source == "cargo")
    {
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();

        const auto a01_it = std::find_if(
            config_.shelves.cbegin(),
            config_.shelves.cend(),
            [](const ShelfConfig &shelf) {
                return shelf.code.compare(
                    "A01", Qt::CaseInsensitive) == 0;
            });
        if (a01_it == config_.shelves.cend())
        {
            appendRunLog("未配置 A01 货架，无法生成货物巡检路线");
            return;
        }

        const ShelfConfig &shelf = *a01_it;
        if (shelf.rows <= 0 || shelf.columns <= 0 ||
            shelf.waypoint_row_z_m.size() != shelf.rows ||
            shelf.waypoint_front_x_m.size() != shelf.columns)
        {
            appendRunLog("A01 正面航点配置无效，无法生成货物巡检路线");
            return;
        }

        // 按固定 mission.yaml 的点位顺序：从最底行开始，逐行向上蛇形遍历。
        QVector<WorldCoord> cargo_points;
        cargo_points.reserve(shelf.rows * shelf.columns);
        const auto append_front_point =
            [&](int row, int column) {
                cargo_points.push_back({
                    shelf.waypoint_front_x_m.at(column),
                    shelf.front_waypoint_y_m,
                    shelf.waypoint_row_z_m.at(row),
                    config_.slot_grid.front_yaw_rad});
            };

        for (int row = shelf.rows - 1; row >= 0; --row)
        {
            const int row_from_bottom = shelf.rows - 1 - row;
            if (row_from_bottom % 2 == 0)
            {
                for (int column = 0; column < shelf.columns; ++column)
                {
                    append_front_point(row, column);
                }
            }
            else
            {
                for (int column = shelf.columns - 1; column >= 0; --column)
                {
                    append_front_point(row, column);
                }
            }
        }

        // 货物巡检必须经过每个槽位，因此禁止压缩连续共线点。
        summary.compress_straight_segments = false;
        mission_upload_in_progress_ = true;
        appendRunLog(
            QString("正在上传 A01 正面巡检路线，共 %1 个点")
                .arg(cargo_points.size()));
        ros_manager_->uploadMissionSummary(cargo_points, summary);
    }
    else if (trigger_source == "waypoint"){
        // 切回 Cargo 流程时取消尚未完成的 Animal 等待状态。
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        summary.compress_straight_segments =
            mission.compress_waypoint_segments;

        if(path_points_.isEmpty()){
            appendRunLog("航点为空，不允许航点飞行");
            return;
        }
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(path_points_, summary);
    }
    else{
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        summary.compress_straight_segments =
            mission.compress_non_waypoint_segments;
        QVector<WorldCoord> empty_points;
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(empty_points, summary);
    }
}

void MainWindow::refreshWaypointLog()
{
    cargo_page_->setWaypointLogText("");

    if (waypoint_labels_.isEmpty()) {
        appendRunLog("已清空航点");
        clock_timer_->start(5000);
        return;
    }

    QStringList labels;
    for (const auto &label : waypoint_labels_) {
        labels.append(label);
    }

    cargo_page_->setWaypointLogText(labels.join("->"));
}

void MainWindow::clearWaypointRequest()
{
    path_points_.clear();
    waypoint_labels_.clear();
    refreshWaypointLog();
}

void MainWindow::setWaypointRequest(
    int shelf_index,
    const QString &side,
    int row,
    int col)
{
    if (shelf_index < 0 || shelf_index >= config_.shelves.size())
    {
        appendRunLog("添加航点失败：货架索引非法");
        return;
    }
    if (side != "front" && side != "back")
    {
        appendRunLog(QString("添加航点失败：side 非法：%1").arg(side));
        return;
    }

    const ShelfConfig &shelf = config_.shelves.at(shelf_index);
    if (row < 0 || row >= shelf.rows)
    {
        appendRunLog(QString("添加航点失败：%1 row 非法：%2")
                         .arg(shelf.code).arg(row));
        return;
    }
    if (col < 0 || col >= shelf.columns)
    {
        appendRunLog(QString("添加航点失败：%1 col 非法：%2")
                         .arg(shelf.code).arg(col));
        return;
    }

    const bool front = side == "front";
    const double x = front
        ? shelf.waypoint_front_x_m.at(col)
        : shelf.waypoint_back_x_m.at(col);
    const double y = front
        ? shelf.front_waypoint_y_m
        : shelf.back_waypoint_y_m;
    const double z = shelf.waypoint_row_z_m.at(row);
    const double yaw = front
        ? config_.slot_grid.front_yaw_rad
        : config_.slot_grid.back_yaw_rad;

    path_points_.push_back({x, y, z, yaw});
    waypoint_labels_.push_back(
        QString("%1%2：R%3C%4")
            .arg(shelf.code)
            .arg(front ? "F" : "B")
            .arg(row + 1)
            .arg(col + 1));
    refreshWaypointLog();
}
void MainWindow::handleMissionUploadFinished(bool success, const QString &message, const QString &saved_path)
{
    Q_UNUSED(saved_path);

    mission_upload_in_progress_ = false;

    if (success)
    {
        appendRunLog(QString("%1").arg(message));
        //clock_timer_->start(5000);
        resetWorldBodyTransform();
        ros_manager_->requestStartOffboard();
    }
    else if (!message.isEmpty())
    {
        collaboration_mission_active_ = false;
        collaboration_task_running_seen_ = false;
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        appendRunLog(QString("初始化失败：%1").arg(message));
        //clock_timer_->start(5000);
    }
    else
    {
        collaboration_mission_active_ = false;
        collaboration_task_running_seen_ = false;
        animal_route_start_pending_ = false;
        animal_offboard_ready_ = false;
        animal_returned_route_.clear();
        appendRunLog("初始化失败");
        //clock_timer_->start(5000);
    }
}

void MainWindow::updateCommandResult(bool success, const QString &message)
{
    Q_UNUSED(success);
    Q_UNUSED(message);
}


void MainWindow::resetWorldBodyTransform()
{
    // 控制端的 tf_bridge_node 每次启动都会用最初 10 帧建立 world_body。
    // 地面站在发出 start_offboard 前同步清空窗口，复现相同的初始化时机。
    ground_world_body_tf_.reset();

    // TF 等待 10 帧期间，三个画板都先回到各自原点，避免保留上一轮任务的位置。
    // Cargo 的原点是场景像素 (150, 100)，不是直接写成 (0, 0)。
    scene_data_.drone_state.pose.x = 150.0;
    scene_data_.drone_state.pose.y = 100.0;
    scene_data_.drone_state.pose.z = 0.0;
    scene_data_.drone_state.pose.yaw = 0.0;
    cargo_page_->setSceneData(scene_data_);

    // Animal 和 Collaboration 的接口都接收米坐标，传 (0, 0, 0) 即回到各自绘图原点。
    animal_page_->setPosition(0.0, 0.0, 0.0);
    collaboration_grid_view_->setdronePosition(0.0, 0.0, 0.0);

    altitude_value_label_->setText("N/A");
    drone_xy_value_label_->setText("N/A");
    yaw_value_label_->setText("N/A");
    drone_z_value_label_->setText("N/A");
    drone_yaw_value_label_->setText("N/A");
}










bool MainWindow::updateStatus_connected()  const
{
    return connect_status;
}

void MainWindow::updateStatus(
    bool connected,
    float battery_percent,
    int flight_mode,
    bool armed,
    const QString &task_name)
{
    if (video_replay_controller_) video_replay_controller_->setArmed(armed);
    switch (flight_mode)
    {
        case drone_msgs::msg::DroneStatus::MODE_MANUAL:
            scene_data_.drone_state.flight_mode = "MANUAL";
            break;
        case drone_msgs::msg::DroneStatus::MODE_OFFBOARD:
            scene_data_.drone_state.flight_mode = "OFFBOARD";
            break;
        case drone_msgs::msg::DroneStatus::MODE_STABILIZE:
            scene_data_.drone_state.flight_mode = "STABILIZE";
            break;
        case drone_msgs::msg::DroneStatus::MODE_AUTO:
            scene_data_.drone_state.flight_mode = "AUTO";
            break;
        case drone_msgs::msg::DroneStatus::MODE_LOITER:
            scene_data_.drone_state.flight_mode = "LOITER";
            break;
        case drone_msgs::msg::DroneStatus::MODE_RTL:
            scene_data_.drone_state.flight_mode = "RTL";
            break;
        default:
            scene_data_.drone_state.flight_mode = "UNKNOWN";
            break;
    }

    connect_status = connected;

    // 先把 ROS 返回的基础飞行状态同步到当前场景数据里。
    scene_data_.drone_state.connected = connected;
    scene_data_.drone_state.battery = battery_percent;
    // scene_data_.drone_state.flight_mode = task_name;

    // 当前项目的 TopStatusBar 只直接展示“连接状态”和“任务文本”，
    // 所以这里先把最关键的信息映射过去。
    top_status_bar_->setConnected(connected);
    //top_status_bar_->setTaskText(task_name);

    battery_value_label_->setText(QString::number(scene_data_.drone_state.battery*100, 'f', 1) + "%");
    drone_battery_value_label_->setText(QString::number(scene_data_.drone_state.battery*100, 'f', 1) + "%");

    mode_value_label_->setText(scene_data_.drone_state.flight_mode);
    drone_move_value_label_->setText(scene_data_.drone_state.flight_mode);

    // flight_mode 目前在当前仓储界面里没有单独的枚举显示控件，
    // 先把它拼进速度/航向旁的任务文本体系里，不额外造新控件。
    Q_UNUSED(flight_mode);

    cargo_page_->setSceneData(scene_data_);



    if (armed == true) {
        if (unlock_flag_ == false) {
            unlock_flag_ = true;       // 标记：本轮已经开锁过
            auto_stop_flag_ = false;    // 新一轮允许自动 stop
        }

        disarm_stable_count_ = 0;       // 只要又变回开锁，就清零去抖计数
    }

    if(unlock_flag_ == true && armed == false){
        disarm_stable_count_++;//只有在曾经开锁过的状态才加
    }
    else if(unlock_flag_ == false && armed == false){
        disarm_stable_count_ = 0;       // 一直没开锁过，不做自动 stop
    }

    //判断是否为从开锁到关索的状态并且判断是否是第一次运行
    if(unlock_flag_ == true && auto_stop_flag_ == false && disarm_stable_count_ >= 15){
        auto_stop_flag_ = true;
        unlock_flag_ = false;
        //waiting_push_result_ = false;//重置等待上传结果的标志，允许下一次上传
        if (ros_manager_) {
            ros_manager_->stopTask();
            //run_log_view_->appendPlainText("巡检任务结束");
            top_status_bar_->setTaskText("任务待命");
        }
    }
}

void MainWindow::action_updateStatus(
    bool task_running,
    int action_step,
    int action_num,
    const QString &action_name)
{
    if (collaboration_mission_active_)
    {
        if (task_running)
        {
            collaboration_task_running_seen_ = true;
        }
        else if (collaboration_task_running_seen_)
        {
            // 初始化阶段可能先收到 idle；只有进入过运行态后的 false 才表示任务结束。
            collaboration_mission_active_ = false;
            collaboration_task_running_seen_ = false;
        }
    }

    latest_drone_action_name_ =
        task_running ? action_name : QString("idle");
    const QString translated_action =
        translatedDroneAction(latest_drone_action_name_);

    // 抛投由视觉伺服成功事件单独显示 3 秒；此时仍缓存新的 action，
    // 但不让紧接着到来的 move 状态提前覆盖“抛投”。
    if (!drone_drop_status_timer_->isActive())
    {
        collaboration_drone_status_value_label_->setText(translated_action);
    }

    const QString task_text = task_running
        ? QString("%1 (%2/%3)").arg(translated_action).arg(action_step).arg(action_num)
        : QString("任务待命");
    top_status_bar_->setTaskText(task_text);
}

void MainWindow::appendBarcodeRecord(
    const QString &barcode,
    const QByteArray &image_data,
    const QString &image_format,
    const QString &time_text)
{
    // 机载巡检回填链路：
    // 1. ROS 管理器把无人机识别结果转成 `barcodeCaptured(...)`，最终调用到这里。
    // 2. `barcode` 正常格式是 `PKG-001|SKU-002|A-1-1`，前两段是识别出的类别/包裹编号，第三段是无人机返回的位置码。
    // 3. 这里先拆条码，再同时准备两套定位来源：
    //    - `code_location`：直接按第三段位置码解析格子。
    //    - `pose_location`：按无人机当前位姿估算格子。
    // 4. 规则上位置码优先，位姿只做兜底和冲突对照日志。
    // 5. 最终定位到格子后，把巡检字段和图片都写进主数据，再刷新弹窗显示。

    //先定义空字符串
    auto normalize_field = [](const QString &value) {
        const QString normalized = value.trimmed();
        if (normalized.compare("NA", Qt::CaseInsensitive) == 0 ||
            normalized.compare("N/A", Qt::CaseInsensitive) == 0) {
            return QString();
        }
        return normalized;
    };

    const QStringList parts = barcode.split('|', Qt::KeepEmptyParts);
    const QString package_id =
        (parts.size() >= 1) ? normalize_field(parts[0]) : QString();
    const QString category_id =
        (parts.size() >= 2) ? normalize_field(parts[1]) : QString();
    const QString slot_code =
        (parts.size() >= 3) ? normalize_field(parts[2]) : QString();

    // 全空保护
    if (package_id.isEmpty() && category_id.isEmpty() && slot_code.isEmpty())
    {
        appendRunLog("收到空条码消息，已忽略");
        return;
    }

    const SlotLocation pose_location = resolveSlotFromPose(scene_data_.drone_state.pose);
    const SlotLocation code_location = resolveSlotFromCode(slot_code, pose_location);

    SlotLocation target_location;
    if (code_location.valid)
    {
        target_location = code_location;
        if (pose_location.valid &&
            (pose_location.shelf_index != code_location.shelf_index ||
             pose_location.side != code_location.side ||
             pose_location.row != code_location.row ||
             pose_location.col != code_location.col))
        {
            appendRunLog(
                QString("位置码%1与位姿映射不一致，采用位置码：货架%2 %3 R%4C%5，位姿映射为货架%6 %7 R%8C%9")
                    .arg(slot_code)
                    .arg(code_location.shelf_index + 1)
                    .arg(code_location.side)
                    .arg(code_location.row + 1)
                    .arg(code_location.col + 1)
                    .arg(pose_location.shelf_index + 1)
                    .arg(pose_location.side)
                    .arg(pose_location.row + 1)
                    .arg(pose_location.col + 1));
        }
    }
    else
    {
        target_location = pose_location;
    }

    if (!target_location.valid)
    {
        appendRunLog(slot_code.isEmpty() ? "收到巡检结果，无法映射" : QString("收到巡检结果，位置码 %1 无法解析且位姿映射失败").arg(slot_code));
        return;
    }

    //根据槽位索引找到对应的货物信息
    ShelfSlotItem *slot = findShelfSlot(target_location.shelf_index, target_location.side, target_location.row, target_location.col);
    if (!slot)
    {
        appendRunLog("收到巡检结果，目标货架槽位无效");
        return;
    }

    // 先落文本识别结果：严格按视觉真实顺序 PKG|SKU|SHELF
    slot->observed_package_id = package_id;
    slot->observed_category_id = category_id;
    slot->position_package_id = slot_code;
    slot->observed_time_text = time_text;

    //解析完其他数据后独立处理图片
    if (!image_data.isEmpty())
    {
        slot->has_image = true;//标记这个槽位已经有图
        slot->latest_image.image_data = image_data;
        slot->latest_image.image_format = image_format;
        slot->latest_image.barcode = barcode;
        slot->latest_image.time_text = time_text;
    }
    else
    {
        slot->has_image = false;
        slot->latest_image = SlotImageData{};
    }

    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        appendRunLog(QString("货架数据保存失败：%1").arg(storage_error));
    }
}

void MainWindow::appendVisionBarcodeCount(
    const QString &barcode,
    const QString &time_text)
{
    Q_UNUSED(barcode);
    Q_UNUSED(time_text);

    // 当前轮次先不使用这条统计信号，只保留接口避免后续接线时再次改动。
}

void MainWindow::applyManualStockIn(int shelf_index, const QString &side, int row, int col,
                                    const QString &category_id, const QString &package_id)
{
    // 地面站手动入库真正落库的地方：
    // 1. Dialog 只负责把“当前选中格 + 扫码结果”发过来。
    // 2. 这里根据格子坐标拿到主数据里的 `ShelfSlotItem`。
    // 3. 只更新手动台账字段 `category_id/package_id`。
    // 4. 更新完成后再统一刷新弹窗，保证主数据和界面显示始终一致。
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);
    if (!slot)
    {
        appendRunLog("手动入库失败：目标槽位无效");
        return;
    }

    slot->category_id = category_id;
    slot->package_id = package_id;
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        appendRunLog(QString("货架数据保存失败：%1").arg(storage_error));
    }
}

void MainWindow::applyManualStockOut(int shelf_index, const QString &side, int row, int col, const QString &category_id, const QString &package_id)
{
    // 地面站手动出库真正清空数据的地方：
    // 1. 先按当前选中格定位到主数据里的槽位。
    // 2. 再把手动台账、巡检结果、位置码、时间、图片统一清空。
    // 3. 这样做的目的，是让“出库”后的格子完整回到空位状态，避免残留旧巡检痕迹误导界面。
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);
    if (!slot)
    {
        appendRunLog("手动出库失败：目标槽位无效");
        return;
    }

    const QString scanned_category_id = category_id.trimmed();
    const QString scanned_package_id = package_id.trimmed();

    if (slot->category_id.isEmpty() || slot->package_id.isEmpty())
    {
        appendRunLog("台帐数据缺失");
        return;
    }
    if (slot->category_id != scanned_category_id ||
        slot->package_id != scanned_package_id)
    {
        appendRunLog("台帐数据不对应");
        return;
    }

    slot->category_id.clear();
    slot->package_id.clear();
    slot->observed_category_id.clear();
    slot->observed_package_id.clear();
    slot->position_package_id.clear();
    slot->observed_time_text.clear();
    slot->has_image = false;
    slot->latest_image = SlotImageData{};
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        appendRunLog(QString("货架数据保存失败：%1").arg(storage_error));
    }

    appendRunLog(
    QString("手动出库成功：货架%1 %2 R%3C%4")
        .arg(shelf_index + 1)
        .arg(side)
        .arg(row + 1)
        .arg(col + 1));
}

SlotLocation MainWindow::resolveSlotFromCode(
    const QString &slot_code,
    const SlotLocation &pose_location) const
{
    SlotLocation location;
    // 同时兼容视觉端 A-0-0 / B-0-0 和地面站配置的
    // A01F-0-0 / A01B-0-0 位置码。视觉端 A/B 只表示正反面，
    // 不包含货架编号：单货架时直接使用唯一货架，多货架时再用
    // 当前位姿区域确定货架，行列始终以位置码为准。
    static const QRegularExpression pattern("^([A-Z0-9]+)-(\\d+)-(\\d+)$");
    const QRegularExpressionMatch match =
        pattern.match(slot_code.trimmed().toUpper());
    if (!match.hasMatch())
    {
        return location;
    }

    const QString prefix = match.captured(1);
    const int row = match.captured(2).toInt();
    const int col = match.captured(3).toInt();
    for (int shelf_index = 0; shelf_index < config_.shelves.size(); ++shelf_index)
    {
        const ShelfConfig &shelf = config_.shelves.at(shelf_index);
        if (prefix == shelf.front_slot_prefix.trimmed().toUpper())
        {
            location.side = "front";
        }
        else if (prefix == shelf.back_slot_prefix.trimmed().toUpper())
        {
            location.side = "back";
        }
        else
        {
            continue;
        }

        if (row < 0 || row >= shelf.rows ||
            col < 0 || col >= shelf.columns)
        {
            return {};
        }
        location.shelf_index = shelf_index;
        location.row = row;
        location.col = col;
        location.valid = shelf_index < shelf_panel_data_.size();
        return location;
    }

    if (prefix == "A" || prefix == "B")
    {
        int shelf_index = -1;
        if (config_.shelves.size() == 1 && !shelf_panel_data_.isEmpty())
        {
            shelf_index = 0;
        }
        else if (pose_location.valid &&
                 pose_location.shelf_index >= 0 &&
                 pose_location.shelf_index < config_.shelves.size() &&
                 pose_location.shelf_index < shelf_panel_data_.size())
        {
            shelf_index = pose_location.shelf_index;
        }

        if (shelf_index < 0)
        {
            return {};
        }

        const ShelfConfig &shelf =
            config_.shelves.at(shelf_index);
        if (row < 0 || row >= shelf.rows ||
            col < 0 || col >= shelf.columns)
        {
            return {};
        }

        location.shelf_index = shelf_index;
        location.side = (prefix == "A") ? "front" : "back";
        location.row = row;
        location.col = col;
        location.valid = true;
        return location;
    }

    return location;
}
ShelfSlotItem *MainWindow::findShelfSlot(int shelf_index, const QString &side, int row, int col)
{
    //根据货架索引、前后面、行列号，找到真正的 `ShelfSlotItem*`
    if (shelf_index < 0 || shelf_index >= shelf_panel_data_.size())
    {
        return nullptr;
    }

    if (shelf_index >= config_.shelves.size())
    {
        return nullptr;
    }
    const ShelfConfig &shelf_config = config_.shelves.at(shelf_index);
    if (row < 0 || row >= shelf_config.rows ||
        col < 0 || col >= shelf_config.columns)
    {
        return nullptr;
    }

    // 决定当前到底是在访问 front 还是 back
    QVector<ShelfSlotItem> *slot_list = nullptr;
    if (side == "front")
    {
        slot_list = &shelf_panel_data_[shelf_index].front_slots;
    }
    else if (side == "back")
    {
        slot_list = &shelf_panel_data_[shelf_index].back_slots;
    }
    else
    {
        return nullptr;
    }

    const int index = row * shelf_config.columns + col;//按当前货架列数转一维下标
    if (!slot_list || index < 0 || index >= slot_list->size())
    {
        return nullptr;
    }

    return &(*slot_list)[index];
}

SlotLocation MainWindow::resolveSlotFromPose(const Pose3D &pose) const
{
    SlotLocation location;

    for (int shelf_index = 0; shelf_index < config_.shelves.size(); ++shelf_index)
    {
        const ShelfConfig &shelf = config_.shelves.at(shelf_index);
        for (const ShelfPoseRegionConfig &region : shelf.pose_regions)
        {
            if (pose.x >= region.x_min && pose.x <= region.x_max &&
                pose.yaw >= region.yaw_min && pose.yaw <= region.yaw_max)
            {
                location.shelf_index = shelf_index;
                location.side = region.side;
                break;
            }
        }
        if (location.shelf_index >= 0)
        {
            break;
        }
    }

    if (location.shelf_index < 0 ||
        location.shelf_index >= shelf_panel_data_.size())
    {
        return {};
    }

    const double clamped_y = qBound(
        config_.slot_grid.pose_y_min, pose.y, config_.slot_grid.pose_y_max);
    const double clamped_z = qBound(
        config_.slot_grid.pose_z_min, pose.z, config_.slot_grid.pose_z_max);
    const double normalized_col =
        (clamped_y - config_.slot_grid.pose_y_min) /
        (config_.slot_grid.pose_y_max - config_.slot_grid.pose_y_min);
    const double normalized_row =
        (clamped_z - config_.slot_grid.pose_z_min) /
        (config_.slot_grid.pose_z_max - config_.slot_grid.pose_z_min);

    const ShelfConfig &matched_shelf =
        config_.shelves.at(location.shelf_index);
    location.col = qBound(
        0,
        static_cast<int>(normalized_col * matched_shelf.columns),
        matched_shelf.columns - 1);
    location.row = qBound(
        0,
        static_cast<int>(normalized_row * matched_shelf.rows),
        matched_shelf.rows - 1);
    location.valid = true;
    return location;
}

void MainWindow::showShelfSlotImage(int shelf_index, const QString &side, int row, int col)
{
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);//找到对应的槽位
    //检查这个槽位是否真的有图
    if (!slot || !slot->has_image || slot->latest_image.image_data.isEmpty())
    {
        // run_log_view_->appendPlainText("当前槽位暂无图片");
        // clock_timer_->start(5000);
        return;
    }

    //处理照片
    QImage image;
    const QByteArray format_bytes = slot->latest_image.image_format.toUtf8();
    const char *format_ptr = format_bytes.isEmpty() ? nullptr : format_bytes.constData();
    if (!image.loadFromData(slot->latest_image.image_data, format_ptr))
    {
        // run_log_view_->appendPlainText("当前槽位图片解析失败");
        // clock_timer_->start(5000);
        return;
    }

    if (!image_preview_dialog_)
    {
        image_preview_dialog_ = new QDialog(this);//图片预览窗口本体
        image_preview_dialog_->resize(600, 400);

        auto *layout = new QVBoxLayout(image_preview_dialog_);//垂直布局，把里面的滚动区域铺进去
        auto *scroll_area = new QScrollArea(image_preview_dialog_);//滚动区域，避免大图把窗口撑爆
        image_preview_label_ = new QLabel(scroll_area);//真正承载图片的控件
        image_preview_label_->setAlignment(Qt::AlignCenter);
        scroll_area->setWidget(image_preview_label_);
        scroll_area->setWidgetResizable(true);
        layout->addWidget(scroll_area);
    }

    //写弹窗标题
    image_preview_dialog_->setWindowTitle(
        QString("货架%1 %2 R%3C%4 | %5 | %6")
            .arg(shelf_index + 1)
            .arg(side)
            .arg(row + 1)
            .arg(col + 1)
            .arg(slot->latest_image.barcode)
            .arg(slot->latest_image.time_text));

    //显示图片
    image_preview_label_->setPixmap(QPixmap::fromImage(image));
    image_preview_label_->adjustSize();
    image_preview_dialog_->show();
    image_preview_dialog_->raise();
    image_preview_dialog_->activateWindow();
}

void MainWindow::updateDelta(double dx, double dy, double dyaw, bool valid)
{
    if (!top_status_bar_)
    {
        return;
    }

    top_status_bar_->updateDelta(dx, dy, dyaw, valid);
}

void MainWindow::updatePathReadyState(bool ready)
{
    
}

bool MainWindow::updateWorldGroupState(const QVector<WorldCoord> &points)
{
    if (points.isEmpty())
    {
        return false;
    }

    // 参考 drone_qt 的显示方式，但这里只输出用户关心的 x、y。
    QString text = QString("收到控制程序回传路线，共 %1 个点").arg(points.size());
    for (const WorldCoord &point : points)
    {
        text += QString(" -> (%1, %2)")
                    .arg(point.x, 0, 'f', 1)
                    .arg(point.y, 0, 'f', 1);
    }

    appendRunLog(text);
    return true;
}

void MainWindow::tryStartAnimalTask()
{
    // ROS 的路线话题和 Offboard 服务结果来自不同回调，不能假定谁先到。
    if (!animal_route_start_pending_ ||
        !animal_offboard_ready_ ||
        animal_returned_route_.isEmpty())
    {
        return;
    }

    if (!updateWorldGroupState(animal_returned_route_))
    {
        return;
    }

    // 必须在调用 startTask() 前清除 pending。即使控制程序重复发布路线，
    // 后续回调也不会让同一轮 Animal 执行再次调用 start 服务。
    animal_route_start_pending_ = false;
    animal_offboard_ready_ = false;
    animal_returned_route_.clear();
    appendRunLog("回传路线确认成功，开始动物巡检任务");
    ros_manager_->startTask();
}

/******************************************************/

void MainWindow::handleVisionServoStatus(
    bool active,
    const QString &state,
    const QString &requested_target_id,
    const QString &tracked_target_id,
    const QString &detail,
    const QString &time_text)
{
    Q_UNUSED(requested_target_id);
    // A locked target is retained for every terminal state, including timeout.
    const QString normalized_state = state.trimmed().toLower();

    if (!tracked_target_id.trimmed().isEmpty()) {
        current_tracked_target_id_ = tracked_target_id.trimmed();
    }

    if (active) {
        vision_servo_active_seen_ = true;
        last_vision_servo_active_ = true;
        return;
    }

    // A retained active=false sample received after startup is not an edge. Only
    // finish a record after this process has observed the matching active=true.
    const bool action_finished =
        vision_servo_active_seen_ && last_vision_servo_active_;
    last_vision_servo_active_ = false;
    if (!action_finished) {
        return;
    }


    // 控制端在视觉伺服成功后立即触发舵机，抛投不是独立 TaskStatus 动作。
    // 因此借助这条完成状态，在右下角准确显示舵机实际保持的 3 秒。
    if (normalized_state == "succeeded")
    {
        collaboration_drone_status_value_label_->setText("抛投");
        drone_drop_status_timer_->start();
    }
    vision_servo_active_seen_ = false;
    const QString completed_target_id = current_tracked_target_id_;
    current_tracked_target_id_.clear();

    // The active phase caches the last non-empty tracked_target_id for this action.
    if (completed_target_id.isEmpty() ||
        config_.inspection_project != InspectionProject::Animal) {
        return;
    }

    // The old drone_qt record contained image bytes as well. This view keeps
    // only three text fields; empty optional values are simply not appended.
    // 具体的列表项创建和数量上限由 AnimalInspectionPage 管理。
    animal_page_->appendRecognitionRecord(
        completed_target_id, detail, time_text);
}

void MainWindow::appendRunLog(const QString &text)
{
    // 运行日志属于项目页面；MainWindow 只根据当前项目转发文本，
    // 不再直接操作页面内部的 QPlainTextEdit。

    switch (config_.inspection_project)
    {
    case InspectionProject::Cargo:
        cargo_page_->appendRunLog(text);
        break;
    case InspectionProject::Animal:
        animal_page_->appendRunLog(text);
        break;
    case InspectionProject::Collaboration:
        collaboration_run_log_view_->appendPlainText(text);
        break;
    }
}

void MainWindow::clearRunLogs()
{
    // 定时清理时同时清除三个项目的日志，避免切换后看到上一轮短提示。
    cargo_page_->clearRunLog();
    animal_page_->clearRunLog();
    collaboration_run_log_view_->clear();
}

void MainWindow::applyInspectionProject(InspectionProject project)
{
    const bool collaboration =
        project == InspectionProject::Collaboration;
    top_status_bar_->setCollaborationMode(collaboration);

    // 路线状态锁只由 IDLE 清除，切换页面不能让同一轮小车路线重复触发。

    // const bool animal =
    //     project == InspectionProject::Animal;

    // // 两套画板始终保留各自状态，只切换可见性，不在切换时重新创建。
    // cargo_page_->setVisible(!animal);
    // animal_page_->setVisible(animal);

    cargo_page_->hide();
    animal_page_->hide();
    collaboration_grid_view_->hide();
    collaboration_status_panel_->hide();
    collaboration_log_panel_->hide();

    switch (project)
    {
    case InspectionProject::Cargo:
        cargo_page_->show();
        attitude_panel_->show();
        drone_attitude_panel_->hide();
        car_attitude_panel_->hide();
        break;
    case InspectionProject::Animal:
        animal_page_->show();
        attitude_panel_->show();
        drone_attitude_panel_->hide();
        car_attitude_panel_->hide();
        break;
    case InspectionProject::Collaboration:
        collaboration_grid_view_->show();
        collaboration_status_panel_->show();
        collaboration_log_panel_->show();
        attitude_panel_->hide();
        drone_attitude_panel_->show();
        car_attitude_panel_->show();
        break;
    }

    // Animal 只保留运行日志和姿态状态；Cargo 继续显示原来的三个日志区域。
    // Animal 是固定二维视角，不显示 Cargo 的 2D/3D 和观察角度滑块。
    // Animal 是固定二维视角；Cargo 的 2D/3D、航点日志和 AI 日志
    // 都已经封装在 CargoInspectionPage 内，切换页面时会一起显示或隐藏。
    // appendRunLog(
    //     animal
    //         ? "已切换到动物巡检二维画板"
    //         : "已切换到货物巡检仓库画板");
    // clock_timer_->start(5000);

    // 项目切换后立即应用对应布局，不必等待下一次窗口缩放。
    updateOverlayGeometry();
}

void MainWindow::applyWindowStyle()
{
    setStyleSheet(
        "QMainWindow, QWidget {"
        "background: #0c1018;"
        "color: #d7e3f4;"
        "}"
        "QSlider::groove:horizontal {"//滑块槽道样式
        "height: 30px;"//槽道高度
        "background: rgba(70, 90, 120, 120);"//槽道背景颜色
        "border-radius: 8px;"//槽道圆角
        "}"
        "QSlider::handle:horizontal {"//滑块样式
        "width: 40px;"//滑块宽度
        "height: 30px;"//滑块高度
        "background: #00c8ff;" //滑块颜色
        "margin: -6px 0;"//滑块垂直居中
        "border-radius: 8px;"//滑块圆角
        "}"
    );

    // Cargo 日志和视角控件、Animal 日志和识别记录的原样式，
    // 已分别随控件移动到两个页面类中。
    attitude_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 100);"//半透明深色背景
        "border: 1px solid rgba(90, 130, 180, 100);"//边框颜色和透明度
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );
    drone_attitude_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 100);"//半透明深色背景
        "border: 1px solid rgba(90, 130, 180, 100);"//边框颜色和透明度
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );
    car_attitude_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 100);"//半透明深色背景
        "border: 1px solid rgba(90, 130, 180, 100);"//边框颜色和透明度
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );
    collaboration_status_panel_->setStyleSheet(
        "#collaborationStatusPanel {"
        "background: rgba(18, 24, 34, 180);"
        "border: 1px solid rgba(90, 130, 180, 140);"
        "border-radius: 8px;"
        "}"
        "#collaborationStatusPanel QLabel {"
        "background: transparent;"
        "border: none;"
        "color: #d7e3f4;"
        "font-size: 18px;"
        "}"
    );
    collaboration_log_panel_->setStyleSheet(
        "#collaborationLogPanel {"
        "background: rgba(18, 24, 34, 180);"
        "border: 1px solid rgba(90, 130, 180, 140);"
        "border-radius: 8px;"
        "}"
        "#collaborationLogTitle {"
        "background: transparent;"
        "border: none;"
        "color: #d7e3f4;"
        "font-size: 18px;"
        "font-weight: 600;"
        "}"
        "QPlainTextEdit {"
        "background: rgba(8, 12, 18, 150);"
        "border: 1px solid rgba(90, 130, 180, 90);"
        "border-radius: 4px;"
        "color: #d7e3f4;"
        "font-size: 16px;"
        "padding: 6px;"
        "}"
    );
}

void MainWindow::applyShelfConfigurationToUi(
    const WarehouseConfig &previous_config)
{
    // 飞行、伺服或相机参数变化不应重建货架业务数据。
    if (shelfConfigurationEquals(previous_config, config_))
    {
        return;
    }

    scene_data_.shelves.clear();
    scene_data_.shelves.reserve(config_.shelves.size());
    for (const ShelfConfig &shelf_config : config_.shelves)
    {
        ShelfBlock shelf;
        shelf.base_rect = shelf_config.base_rect;
        shelf.height = shelf_config.height;
        shelf.name = shelf_config.code;
        shelf.color = shelf_config.scene_color;
        scene_data_.shelves.push_back(shelf);
    }
    cargo_page_->setSceneData(scene_data_);

    // 每个货架独立迁移仍存在的行列。缩小会舍弃越界格子，扩大时新增格子为空。
    const QVector<ShelfPanelData> previous_panels = shelf_panel_data_;
    QVector<ShelfPanelData> migrated_panels;
    migrated_panels.reserve(config_.shelves.size());
    for (int shelf_index = 0;
         shelf_index < config_.shelves.size();
         ++shelf_index)
    {
        const ShelfConfig &new_shelf = config_.shelves.at(shelf_index);
        ShelfPanelData panel;
        panel.display_name = new_shelf.code;
        panel.button_status_color = new_shelf.button_status_color;
        panel.front_slots.resize(new_shelf.slotCountPerSide());
        panel.back_slots.resize(new_shelf.slotCountPerSide());

        if (shelf_index < previous_panels.size() &&
            shelf_index < previous_config.shelves.size())
        {
            const ShelfPanelData &old_panel = previous_panels.at(shelf_index);
            const ShelfConfig &old_shelf =
                previous_config.shelves.at(shelf_index);
            const int copied_rows = std::min(old_shelf.rows, new_shelf.rows);
            const int copied_columns =
                std::min(old_shelf.columns, new_shelf.columns);
            auto copy_face = [&](const QVector<ShelfSlotItem> &source,
                                 QVector<ShelfSlotItem> &target) {
                for (int row = 0; row < copied_rows; ++row)
                {
                    for (int col = 0; col < copied_columns; ++col)
                    {
                        const int old_index = row * old_shelf.columns + col;
                        const int new_index = row * new_shelf.columns + col;
                        if (old_index >= 0 && old_index < source.size() &&
                            new_index >= 0 && new_index < target.size())
                        {
                            target[new_index] = source.at(old_index);
                        }
                    }
                }
            };
            copy_face(old_panel.front_slots, panel.front_slots);
            copy_face(old_panel.back_slots, panel.back_slots);
        }
        migrated_panels.push_back(panel);
    }

    shelf_panel_data_ = migrated_panels;
    shelf_info_dialog_->setShelfConfigs(config_.shelves);
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);

    // 旧航点已经包含修改前的坐标，货架配置生效后必须重新选择。
    clearWaypointRequest();

    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        appendRunLog(
            QString("货架配置已生效，但槽位数据保存失败：%1")
                .arg(storage_error));
    }
    else
    {
        appendRunLog("货架和槽位配置已立即生效");
    }
}
void MainWindow::setupDemoData()
{
    WarehouseSceneData data;
    data.drone_state.flight_mode = "OFFBOARD";
    data.drone_state.pose.z = 0.0;
    data.drone_state.speed = 4.2;
    data.drone_state.battery = 87.0;

    shelf_panel_data_.clear();
    for (const ShelfConfig &shelf_config : config_.shelves)
    {
        ShelfBlock shelf;
        shelf.base_rect = shelf_config.base_rect;
        shelf.height = shelf_config.height;
        shelf.name = shelf_config.code;
        shelf.color = shelf_config.scene_color;
        data.shelves.push_back(shelf);

        ShelfPanelData panel;
        panel.display_name = shelf_config.display_name;
        panel.button_status_color = shelf_config.button_status_color;
        panel.front_slots.resize(shelf_config.slotCountPerSide());
        panel.back_slots.resize(shelf_config.slotCountPerSide());
        shelf_panel_data_.push_back(panel);
    }

    scene_data_ = data;
    cargo_page_->setSceneData(scene_data_);
    top_status_bar_->setConnected(data.drone_state.connected);
    top_status_bar_->setTaskText("任务待命");

    QString storage_error;
    if (ShelfPanelStorage::load(
            shelf_panel_data_,
            &storage_error))
    {
        appendRunLog(
            QString("已加载货架持久化数据：%1，文件：%2")
                .arg(shelf_panel_data_.size())
                .arg(ShelfPanelStorage::defaultFilePath()));
    }
    else
    {
        appendRunLog(
            QString("未加载历史货架数据：%1，文件：%2")
                .arg(storage_error)
                .arg(ShelfPanelStorage::defaultFilePath()));
    }

    shelf_info_dialog_->setShelfConfigs(config_.shelves);
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    updateOverlayGeometry();//窗口大小改变时调整悬浮控件的位置和大小
}

void MainWindow::updateOverlayGeometry()
{
    if (!central_container_)
    {
        return;
    }

    const QRect area = central_container_->rect();

    // 顶部状态栏属于两个项目共用区域，先确定它的位置。
    top_status_bar_->setGeometry(20, 16, area.width() - 40, 52);
    drone_attitude_panel_->setGeometry(width() - 540, 84, 250, 250);
    car_attitude_panel_->setGeometry(width() - 260, 84, 250, 180);
    const int status_panel_width = 380;
    const int status_panel_height = 112;
    const int status_panel_y =
        area.height() - status_panel_height - 20;
    collaboration_status_panel_->setGeometry(
        area.width() - status_panel_width - 20,
        status_panel_y,
        status_panel_width, status_panel_height);

    // 日志使用右侧两块姿态面板的总宽度，并在下方状态面板前结束。
    // 1024x600 下仍保留约 100 px 高度，更高分辨率会自动获得更多空间。
    const int collaboration_log_y = 350;
    const int collaboration_log_height =
        std::max(90, status_panel_y - collaboration_log_y - 12);
    collaboration_log_panel_->setGeometry(
        area.width() - 540, collaboration_log_y,
        520, collaboration_log_height);

    // 两个页面始终使用同一块完整区域；页面内部根据自己的项目规则
    // 安排画板、日志、识别记录和 Cargo 视角控件。
    cargo_page_->setGeometry(area);
    animal_page_->setGeometry(area);
    collaboration_grid_view_->setGeometry(area);

    // 姿态面板是共享控件，但不同项目需要不同位置。
    attitude_panel_->setGeometry(
        config_.inspection_project == InspectionProject::Animal
            ? animal_page_->attitudePanelGeometry()
            : cargo_page_->attitudePanelGeometry());

    // 页面铺满主容器后，共享状态控件必须保持在最上层。
    top_status_bar_->raise();
    attitude_panel_->raise();
    drone_attitude_panel_->raise();
    car_attitude_panel_->raise();
    collaboration_status_panel_->raise();
    collaboration_log_panel_->raise();
}

QVector<SlotAnalysisInput> MainWindow::collectSlotAnalysisInputs() const
{
    QVector<SlotAnalysisInput> inputs;

    for (int shelf_index = 0; shelf_index < shelf_panel_data_.size(); ++shelf_index)
    {
        if (shelf_index >= config_.shelves.size())
        {
            break;
        }
        const ShelfPanelData &shelf = shelf_panel_data_[shelf_index];
        const ShelfConfig &shelf_config = config_.shelves.at(shelf_index);

        auto append_slot_items = [&](const QVector<ShelfSlotItem> &slot_items, const QString &side) {
            for (int row = 0; row < shelf_config.rows; ++row)
            {
                for (int col = 0; col < shelf_config.columns; ++col)
                {
                    const int index = row * shelf_config.columns + col;
                    if (index < 0 || index >= slot_items.size())
                    {
                        continue;
                    }

                    const ShelfSlotItem &slot = slot_items[index];

                    SlotAnalysisInput input;
                    input.shelf_name = shelf.display_name;
                    input.shelf_index = shelf_index;
                    input.side = side;
                    input.row = row;
                    input.col = col;

                    input.manual_category_id = slot.category_id;
                    input.manual_package_id = slot.package_id;

                    input.observed_category_id = slot.observed_category_id;
                    input.observed_package_id = slot.observed_package_id;
                    input.observed_slot_code = slot.position_package_id;
                    input.observed_time_text = slot.observed_time_text;
                    input.has_image = slot.has_image;

                    inputs.push_back(input);
                }
            }
        };

        append_slot_items(shelf.front_slots, "front");
        append_slot_items(shelf.back_slots, "back");
    }

    return inputs;
}

QVector<SlotRuleAnalysis> MainWindow::buildRuleAnalysisResults() const
{
    const QVector<SlotAnalysisInput> inputs = collectSlotAnalysisInputs();
    return AiDiffAnalyzer::analyzeAll(inputs);
}

QString MainWindow::buildRuleAnalysisReport(const QVector<SlotRuleAnalysis> &results) const
{
    QStringList lines;
    lines << "=== AI差异巡检助手（规则预分析） ===";

    int high_count = 0;
    int medium_count = 0;
    int low_count = 0;

    for (const SlotRuleAnalysis &result : results)
    {
        if (result.priority >= 80)
        {
            ++high_count;
        }
        else if (result.priority >= 50)
        {
            ++medium_count;
        }
        else
        {
            ++low_count;
        }
    }

    lines << QString("高优先级：%1 项").arg(high_count);
    lines << QString("中优先级：%1 项").arg(medium_count);
    lines << QString("低优先级：%1 项").arg(low_count);
    lines << "";

    for (const SlotRuleAnalysis &result : results)
    {
        if (result.status == SlotDiffStatus::Matched || result.status == SlotDiffStatus::Empty)
        {
            continue;
        }

        lines << result.summary;
        lines << QString("原因：%1").arg(result.reason);
        lines << QString("优先级：%1").arg(result.priority);
        lines << QString("建议复查：%1").arg(result.should_revisit ? "是" : "否");
        lines << "";
    }

    return lines.join('\n');
}

void MainWindow::runAiDiffAnalysis()
{
    const QVector<SlotRuleAnalysis> results = buildRuleAnalysisResults();
    const QString report = buildRuleAnalysisReport(results);
    cargo_page_->setAiLogText(report);
}

QString MainWindow::buildAiPrompt(const SlotRuleAnalysis &result) const
{
    const SlotAnalysisInput &input = result.input;

    QStringList lines;
    lines << "你是仓储槽位图片巡检助手。";
    lines << "请直接查看图片，并结合槽位上下文判断：";
    lines << "1. 是否能确认图中有包裹；";
    lines << "2. 包裹主体是否清晰可见；";
    lines << "3. 标签/条码/身份信息是否可辨认；";
    lines << "4. 当前图片证据是否足以支持判断；";
    lines << "5. 如无法确认，请明确写无法确认。";
    lines << "";
    lines << "请只输出 JSON 数组。";
    lines << "每项字段固定为：slot,severity,summary,reason,action,should_revisit。";
    lines << "severity 只能是 low / medium / high。";
    lines << "should_revisit 只能是 true / false。";
    lines << "";
    lines << "约束：";
    lines << "- 优先依据图片本身判断，不要只复述上下文字段。";
    lines << "- 不要臆造未看见的内容。";
    lines << "- 不要输出 markdown。";
    lines << "- 不要长篇解释。";
    lines << "- summary：一句话结论，尽量简短。";
    lines << "- reason：只写最关键事实。";
    lines << "- action：只给一个动作。";
    lines << "";
    lines << "槽位上下文：";
    lines << QString("- slot=%1 %2 R%3C%4")
                .arg(input.shelf_name)
                .arg(input.side)
                .arg(input.row + 1)
                .arg(input.col + 1);
    lines << QString("- manual_package=%1").arg(input.manual_package_id);
    lines << QString("- manual_category=%1").arg(input.manual_category_id);
    lines << QString("- observed_package=%1").arg(input.observed_package_id);
    lines << QString("- observed_category=%1").arg(input.observed_category_id);
    lines << QString("- observed_slot=%1").arg(input.observed_slot_code);
    lines << QString("- observed_time=%1").arg(input.observed_time_text);
    lines << QString("- local_summary=%1").arg(result.summary);
    lines << QString("- local_reason=%1").arg(result.reason);

    return lines.join('\n');
}

void MainWindow::runClaudeApiDiffAnalysis()
{
    const QVector<SlotRuleAnalysis> results = buildRuleAnalysisResults();

    cargo_page_->clearAiLog();
    // ai_log_view_->appendPlainText("AI分析开始...");

    auto slot_label = [](const SlotRuleAnalysis &result) {
        const QString shelf_short = result.input.shelf_name.endsWith("A") ? "A" :
                                    (result.input.shelf_name.endsWith("B") ? "B" : result.input.shelf_name);
        const QString side_short = (result.input.side == "front") ? "F" :
                                   ((result.input.side == "back") ? "B" : result.input.side);
        return QString("%1 %2 R%3C%4")
            .arg(shelf_short)
            .arg(side_short)
            .arg(result.input.row + 1)
            .arg(result.input.col + 1);
    };

    auto status_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("台账与巡检不一致");
        case SlotDiffStatus::PartialObserved:
            return QString("巡检结果不完整");
        case SlotDiffStatus::ObservedOnly:
            return QString("巡检识别到货物，但台账为空");
        case SlotDiffStatus::ManualOnly:
            return QString("存在台账，但巡检为空");
        case SlotDiffStatus::PositionOnly:
            return QString("仅识别到位置，没有识别到货物身份");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("巡检有结果，但缺少图片证据");
        case SlotDiffStatus::Matched:
            return QString("台账与巡检一致");
        case SlotDiffStatus::Empty:
            return QString("空槽位");
        }
        return QString("未知状态");
    };

    auto field_text = [](const QString &value) {
        return value.isEmpty() ? QString("空") : value;
    };

    auto risk_level_text = [](int priority) {
        if (priority >= 80)
        {
            return QString("高");
        }
        if (priority >= 60)
        {
            return QString("中");
        }
        return QString("低");
    };

    auto short_reason_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("台账与巡检货物信息冲突");
        case SlotDiffStatus::PartialObserved:
            return QString("包裹或类别信息识别不完整");
        case SlotDiffStatus::ObservedOnly:
            return QString("识别到货物但无台账记录");
        case SlotDiffStatus::ManualOnly:
            return QString("台账货物未被巡检识别");
        case SlotDiffStatus::PositionOnly:
            return QString("仅识别到位置，货物信息缺失");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("识别结果缺少图片证据");
        case SlotDiffStatus::Matched:
        case SlotDiffStatus::Empty:
            return QString("无异常");
        }
        return QString("待复查");
    };

    auto action_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("核对实物并更新台账");
        case SlotDiffStatus::PartialObserved:
        case SlotDiffStatus::PositionOnly:
            return QString("补拍并重新识别");
        case SlotDiffStatus::ObservedOnly:
            return QString("核对实物并补登记");
        case SlotDiffStatus::ManualOnly:
            return QString("复查货物是否在位");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("补拍并留存证据");
        case SlotDiffStatus::Matched:
        case SlotDiffStatus::Empty:
            return QString("无需处理");
        }
        return QString("人工复查");
    };

    QVector<const SlotRuleAnalysis *> abnormal_results;

    int abnormal_count = 0;
    int high_count = 0;
    for (const SlotRuleAnalysis &result : results)
    {
        if (result.status == SlotDiffStatus::Matched || result.status == SlotDiffStatus::Empty)
        {
            continue;
        }

        ++abnormal_count;
        if (result.priority >= 80)
        {
            ++high_count;
        }
        abnormal_results.push_back(&result);
    }

    if (abnormal_count == 0)
    {
        // ai_log_view_->appendPlainText(QString("仓库状态：正常，异常0/%1。").arg(results.size()));
        return;
    }

    std::sort(abnormal_results.begin(), abnormal_results.end(),
        [](const SlotRuleAnalysis *left, const SlotRuleAnalysis *right) {
            return left->priority > right->priority;
        });

    QStringList ai_input_lines;
    QStringList problem_lines;
    QStringList fallback_problem_lines;
    QStringList fallback_action_lines;
    QStringList expected_slots;
    for (int index = 0; index < abnormal_results.size(); ++index)
    {
        const SlotRuleAnalysis &result = *abnormal_results.at(index);
        const QString slot = slot_label(result);

        if (index >= 3)
        {
            continue;
        }

        ai_input_lines << QString("槽位：%1").arg(slot);
        ai_input_lines << QString("- 异常类型：%1").arg(status_text(result.status));
        ai_input_lines << QString("- 规则结论：%1").arg(result.summary);
        ai_input_lines << QString("- 规则依据：%1").arg(result.reason);
        ai_input_lines << QString("- 风险等级：%1").arg(risk_level_text(result.priority));
        ai_input_lines << QString("- 台账类别：%1").arg(field_text(result.input.manual_category_id));
        ai_input_lines << QString("- 台账包裹：%1").arg(field_text(result.input.manual_package_id));
        ai_input_lines << QString("- 巡检类别：%1").arg(field_text(result.input.observed_category_id));
        ai_input_lines << QString("- 巡检包裹：%1").arg(field_text(result.input.observed_package_id));
        ai_input_lines << QString("- 巡检位置码：%1").arg(field_text(result.input.observed_slot_code));
        ai_input_lines << QString("- 巡检时间：%1").arg(field_text(result.input.observed_time_text));
        ai_input_lines << QString("- 图片证据：%1").arg(result.input.has_image ? "有" : "无");
        ai_input_lines << QString("- 规则建议复查：%1").arg(result.should_revisit ? "是" : "否");
        ai_input_lines << "";

        if (index < 3)
        {
            problem_lines << QString("%1 %2").arg(slot).arg(status_text(result.status));
            fallback_problem_lines << QString("%1 %2").arg(slot).arg(short_reason_text(result.status));
            fallback_action_lines << QString("%1 %2").arg(slot).arg(action_text(result.status));
            expected_slots << slot;
        }
    }

    QStringList lines;
    // lines << QString("仓库状态：异常%1/%2，高优先级%3。")
    //              .arg(abnormal_count)
    //              .arg(results.size())
    //              .arg(high_count);
    // lines << QString("问题槽位：%1").arg(problem_lines.join("；"));
    lines << "AI正在生成简短建议...";
    cargo_page_->appendAiLog(lines.join('\n'));

    QStringList prompt_lines;
    prompt_lines << "你是仓储巡检值班助手。";
    prompt_lines << "只能依据以下事实回答，不能增加槽位、数量或现场信息。";
    prompt_lines << "必须只输出3行，不要markdown，不要解释规则。";
    prompt_lines << QString("第一行必须以“仓库状态：”开头，并使用：异常%1处，高优先级%2处。")
                        .arg(abnormal_count)
                        .arg(high_count);
    prompt_lines << "第二行必须以“问题槽位：”开头，只能使用下列允许槽位，说明台账和巡检的实际差异。";
    prompt_lines << "第三行必须以“下一步：”开头，针对最高风险槽位给出具体复查动作。";
    prompt_lines << "禁止输出槽位1、槽位2、原因、动作1等占位词；不得重复三行内容。";
    prompt_lines << "允许输出的异常槽位：";
    for (const QString &slot : expected_slots)
    {
        prompt_lines << QString("- %1").arg(slot);
    }
    prompt_lines << "";
    prompt_lines << "槽位事实：";
    prompt_lines << ai_input_lines;

    const QString fallback_text = QStringList{
        QString("仓库状态：异常%1处，高优先级%2处。").arg(abnormal_count).arg(high_count),
        QString("问题槽位：%1").arg(fallback_problem_lines.join("；")),
        QString("下一步：%1").arg(fallback_action_lines.join("；"))
    }.join('\n');

    const QString temp_dir = QDir::tempPath();
    const QString prompt_path = temp_dir + "/warehouse_ai_prompt.txt";
    const QString image_meta_path = temp_dir + "/warehouse_ai_image_meta.json";
    const QString output_path = temp_dir + "/warehouse_ai_output.txt";
    const QString script_path = QCoreApplication::applicationDirPath() + "/warehouse_ai_runner.py";

    QFile prompt_file(prompt_path);
    if (!prompt_file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        cargo_page_->appendAiLog("AI总结失败：无法写入 prompt 文件");
        return;
    }
    prompt_file.write(prompt_lines.join('\n').toUtf8());
    prompt_file.close();

    QJsonObject image_meta;
    image_meta["mode"] = "text_summary";

    QFile image_meta_file(image_meta_path);
    if (!image_meta_file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        cargo_page_->appendAiLog("AI总结失败：无法写入输入元数据文件");
        return;
    }
    image_meta_file.write(QJsonDocument(image_meta).toJson(QJsonDocument::Compact));
    image_meta_file.close();

    auto *process = new QProcess(this);
    connect(process,
        QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this,
        [this, process, output_path, fallback_text, expected_slots, abnormal_count, high_count](int exit_code, QProcess::ExitStatus exit_status) {
        Q_UNUSED(exit_status);

        if (exit_code != 0)
        {
            cargo_page_->appendAiLog("AI总结失败：调用脚本退出异常，已显示规则保底结果。");
            cargo_page_->appendAiLog(QString::fromUtf8(process->readAllStandardError()));
            cargo_page_->appendAiLog(fallback_text);
            process->deleteLater();
            return;
        }

        QFile output_file(output_path);
        if (!output_file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            cargo_page_->appendAiLog("AI总结失败：无法读取输出结果，已显示规则保底结果。");
            cargo_page_->appendAiLog(fallback_text);
            process->deleteLater();
            return;
        }

        const QString output_text = QString::fromUtf8(output_file.readAll()).trimmed();
        output_file.close();

        const QStringList output_lines = output_text.split('\n', Qt::SkipEmptyParts);
        bool output_is_valid = output_lines.size() == 3
            && output_lines.at(0).trimmed().startsWith("仓库状态：")
            && output_lines.at(1).trimmed().startsWith("问题槽位：")
            && output_lines.at(2).trimmed().startsWith("下一步：");

        const QRegularExpression placeholder_pattern("槽位\\s*[0-9]+|动作\\s*[0-9]+");
        if (output_is_valid && placeholder_pattern.match(output_text).hasMatch())
        {
            output_is_valid = false;
        }

        auto normalize_text = [](QString text) {
            text.remove(QRegularExpression("\\s+"));
            return text;
        };

        const QString normalized_output = normalize_text(output_text);

        bool has_real_slot = false;
        for (const QString &slot : expected_slots)
        {
            if (normalized_output.contains(normalize_text(slot)))
            {
                has_real_slot = true;
                break;
            }
        }

        const QString normalized_status = normalize_text(output_lines.at(0));
        const bool has_correct_counts =
            normalized_status.contains(QString("异常%1").arg(abnormal_count))
            && normalized_status.contains(QString("高优先级%1").arg(high_count));

        bool has_unknown_slot = false;
        const QRegularExpression slot_pattern("R\\d+C\\d+");
        QRegularExpressionMatchIterator matches =
            slot_pattern.globalMatch(normalized_output);

        while (matches.hasNext())
        {
            const QString detected_code = matches.next().captured(0);
            bool is_allowed = false;

            for (const QString &slot : expected_slots)
            {
                if (normalize_text(slot).contains(detected_code))
                {
                    is_allowed = true;
                    break;
                }
            }

            if (!is_allowed)
            {
                has_unknown_slot = true;
                break;
            }
        }

        output_is_valid = output_is_valid
            && has_real_slot
            && has_correct_counts
            && !has_unknown_slot;

        cargo_page_->appendAiLog("");
        if (output_is_valid)
        {
            cargo_page_->appendAiLog(output_text);
        }
        else
        {
            cargo_page_->appendAiLog("AI输出无效，已显示规则保底结果。");
            cargo_page_->appendAiLog("AI原始输出：");
            cargo_page_->appendAiLog(
                output_text.isEmpty() ? "（空输出）" : output_text);
            cargo_page_->appendAiLog("规则保底结果：");
            cargo_page_->appendAiLog(fallback_text);
        }

        process->deleteLater();
    });

    QStringList args;
    args << script_path << prompt_path << image_meta_path << output_path;
    process->start("python3", args);
}

