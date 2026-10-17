#include "drone_warehouse/cargo_inspection_page.hpp"

#include "drone_warehouse/scene_view.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QSlider>
#include <QVBoxLayout>

CargoInspectionPage::CargoInspectionPage(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
    setupConnections();
}

void CargoInspectionPage::setupUi()
{
    // 创建货物巡检主场景视图，负责绘制仓库、无人机和轨迹。
    scene_view_ = new SceneView(this);

    // 三个日志面板原来由 MainWindow 创建，现在完整归 Cargo 页面管理。
    auto make_log_panel = [this](const char *object_name,
                                  QWidget *&panel,
                                  QPlainTextEdit *&view) {
        panel = new QWidget(this);
        panel->setObjectName(object_name);
        auto *layout = new QVBoxLayout(panel);
        panel->setContentsMargins(5, 5, 5, 5); // 姿态面板内部边框留白
        view = new QPlainTextEdit(panel);
        view->setReadOnly(true);
        view->setMaximumBlockCount(1000);
        layout->addWidget(view);
    };

    make_log_panel("logSwitchPanel", run_log_panel_, run_log_view_);
    make_log_panel("logwaypointSwitchPanel", waypoint_log_panel_, waypoint_log_view_);
    make_log_panel("aiLogPanel", ai_log_panel_, ai_log_view_);

    // 以下被注释的备用界面代码来自原 MainWindow，拆分后仍保留在所属页面。
    //waypoint_log_view_->appendPlainText("航点日志初始化成功");
    // auto *ai_log_title = new QLabel("AI分析", ai_log_panel_);
    // ai_log_title->setObjectName("aiLogTitle");
    // ai_log_layout->addWidget(ai_log_title);
    // ai_log_view_->appendPlainText("AI分析日志初始化成功");

    /*********************模式切换滑块***********************/

    view_mode_widget_ = new QWidget(this);
    auto *mode_layout = new QHBoxLayout(view_mode_widget_);
    mode_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    mode_layout->setSpacing(1);//滑块中各元素间距

    view_mode_left_label_ = new QLabel("2D", view_mode_widget_);
    view_mode_right_label_ = new QLabel("3D", view_mode_widget_);
    view_mode_slider_ = new QSlider(Qt::Horizontal, view_mode_widget_);//水平滑动条
    view_mode_slider_->setRange(0, 1);//滑块取值范围0-1，0表示2D模式，1表示3D模式
    view_mode_slider_->setValue(1);//默认3D模式
    view_mode_slider_->setFixedHeight(35);

    mode_layout->addWidget(view_mode_left_label_);
    mode_layout->addWidget(view_mode_slider_);
    mode_layout->addWidget(view_mode_right_label_);

    /*********************视角切换滑块***********************/

    view_perspective_widget_ = new QWidget(this);
    auto *perspective_layout = new QHBoxLayout(view_perspective_widget_);
    perspective_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    perspective_layout->setSpacing(1);//滑块中各元素间距

    view_perspective_slider_ =
        new QSlider(Qt::Horizontal, view_perspective_widget_);//水平滑动条
    view_perspective_slider_->setRange(0, 3);//滑块取值范围0-3
    view_perspective_slider_->setValue(0);
    view_perspective_slider_->setFixedHeight(35);
    //slider_Perspective_layout->addWidget(view_Perspective_left_label_);
    perspective_layout->addWidget(view_perspective_slider_);
    //slider_Perspective_layout->addWidget(view_Perspective_right_label_);

    view_2d_widget_ = new QWidget(this);
    auto *view_2d_layout = new QHBoxLayout(view_2d_widget_);
    view_2d_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    view_2d_layout->setSpacing(1);//滑块中各元素间距

    view_2d_slider_ = new QSlider(Qt::Horizontal, view_2d_widget_);//水平滑动条
    view_2d_slider_->setRange(0, 2);//滑块取值范围0-2，0表示上视图，1表示左视图，2表示右视图
    view_2d_slider_->setValue(0);
    view_2d_slider_->setFixedHeight(35);
    view_2d_layout->addWidget(view_2d_slider_);

    // 页面内部控件统一在这里设置样式，MainWindow 不再了解这些控件。
    // 原来预留的 AI 面板样式方案继续保留，后面需要半透明面板时可以恢复。
    // "#aiLogPanel {"
    // "background: rgba(18, 24, 34, 150);"
    // "border: 1px solid rgba(90, 130, 180, 100);"
    // "border-radius: 10px;"
    // "}"
    // "#aiLogTitle {"
    // "background: transparent;"
    // "border: none;"
    // "font-size: 16px;"
    // "font-weight: 600;"
    // "color: #8fe7ff;"
    // "}"
    // "QPlainTextEdit {"
    // "background: rgba(10, 14, 22, 170);"
    // "border: none;"
    // "color: #d7e3f4;"
    // "font-size: 14px;"
    // "padding: 6px;"
    // "}"

    const QString transparent_panel_style =
        "background: rgba(18, 24, 34, 0);"
        "font-size: 16px; border: none; border-radius: 10px;"
        "padding: 6px 10px;";
    run_log_panel_->setStyleSheet(transparent_panel_style);
    waypoint_log_panel_->setStyleSheet(transparent_panel_style);
    ai_log_panel_->setStyleSheet(transparent_panel_style);

    //左视图标签
    //中视图标签
    //右视图标签
    //"border: 1px solid rgba(90, 130, 180, 120);"
    const QString view_control_style =
        "background: rgba(18, 24, 34, 170);"
        "border: none; border-radius: 10px;";
    view_mode_widget_->setStyleSheet(view_control_style);
    view_perspective_widget_->setStyleSheet(view_control_style);
    view_2d_widget_->setStyleSheet(view_control_style);
    //标签无边框
    view_mode_left_label_->setStyleSheet("border: none; font-size: 18px;");
    view_mode_right_label_->setStyleSheet("border: none; font-size: 18px;");

    run_log_view_->appendPlainText("日志初始化成功");
    updateViewControlVisibility();
    updatePageGeometry();
}

void CargoInspectionPage::setupConnections()
{
    //连接视图模式滑动条的值改变信号，根据值切换视图模式
    // 连接视图模式滑动条的值改变信号，根据值切换视图模式。
    // 三组滑块和 SceneView 都属于 Cargo 页面，因此 connect 也留在页面内部。
    connect(view_mode_slider_, &QSlider::valueChanged, this, [this](int value) {
        scene_view_->setViewMode(value == 0 ? ViewMode::Top2D : ViewMode::Pseudo3D);
        updateViewControlVisibility();
    });

    //连接3D切换视角滑动条的值改变信号，根据值切换视图模式
    // 连接 3D 切换视角滑动条的值改变信号，根据值切换视图模式。
    connect(view_perspective_slider_, &QSlider::valueChanged,
            this, [this](int value) {
        switch (value)
        {
        case 0:
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective225);
            break;
        case 1:
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective315);
            break;
        case 2:
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective45);
            break;
        default:
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective135);
            break;
        }
    });

    //连接2D切换视角滑动条的值改变信号，根据值切换视图模式
    // 连接 2D 切换视角滑动条的值改变信号，根据值切换视图模式。
    connect(view_2d_slider_, &QSlider::valueChanged, this, [this](int value) {
        switch (value)
        {
        case 0:
            scene_view_->setView2DMode(View2DMode::Perspectivetop);
            break;
        case 1:
            scene_view_->setView2DMode(View2DMode::Perspective90);
            break;
        default:
            scene_view_->setView2DMode(View2DMode::Perspective0);
            break;
        }
    });
}

void CargoInspectionPage::setSceneData(const WarehouseSceneData &data)
{
    scene_view_->setSceneData(data);
}

void CargoInspectionPage::appendRunLog(const QString &text)
{
    run_log_view_->appendPlainText(text);
}

void CargoInspectionPage::clearRunLog()
{
    run_log_view_->clear();
}

void CargoInspectionPage::setWaypointLogText(const QString &text)
{
    waypoint_log_view_->clear();
    if (!text.isEmpty())
    {
        waypoint_log_view_->appendPlainText(text);
    }
}

void CargoInspectionPage::clearAiLog()
{
    ai_log_view_->clear();
}

void CargoInspectionPage::appendAiLog(const QString &text)
{
    ai_log_view_->appendPlainText(text);
}

void CargoInspectionPage::setAiLogText(const QString &text)
{
    ai_log_view_->clear();
    ai_log_view_->appendPlainText(text);
}

QRect CargoInspectionPage::attitudePanelGeometry() const
{
    return QRect(width() - 220, 84, 220, 160);
}

void CargoInspectionPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updatePageGeometry();
}

void CargoInspectionPage::updateViewControlVisibility()
{
    const bool is_2d = view_mode_slider_->value() == 0;
    view_perspective_widget_->setVisible(!is_2d);
    view_2d_widget_->setVisible(is_2d);
}

void CargoInspectionPage::updatePageGeometry()
{
    const QRect area = rect();
    //Cargo 主场景占满整个主容器
    scene_view_->setGeometry(area); // Cargo 主场景占满整个页面

    // Cargo 完全保留原来的仓库画板和三个日志区域布局。
    // 这些数值保持原货物巡检界面的布局，拆分类不改变现有视觉效果。
    run_log_panel_->setGeometry(5, 78, 310, 200);
    waypoint_log_panel_->setGeometry(250, area.height() - 90, 600, 200);
    ai_log_panel_->setGeometry(area.width() - 320, 260, 330, 280);
    view_mode_widget_->setGeometry(100, area.height() - 70, 160, 40);
    view_perspective_widget_->setGeometry(
        area.width() - 220, area.height() - 70, 160, 40);
    view_2d_widget_->setGeometry(
        area.width() - 220, area.height() - 70, 160, 40);

    scene_view_->lower();
    run_log_panel_->raise();//确保日志区主场景视图上面
    waypoint_log_panel_->raise();//确保日志区主场景视图上面
    ai_log_panel_->raise();//确保AI分析日志区在主场景视图上面
    view_mode_widget_->raise();//确保视图模式控件在主场景视图上面
    view_perspective_widget_->raise();//确保视角切换控件在主场景视图上面
    view_2d_widget_->raise();//确保2D视角切换控件在主场景视图上面
}
