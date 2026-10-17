#pragma once

#include "drone_warehouse/models.hpp"

#include <QRect>
#include <QWidget>

class QLabel;
class QPlainTextEdit;
class QResizeEvent;
class QSlider;
class SceneView;

// 货物巡检页面只管理货物项目自己的画板、日志和视角控件。
// ROS 通信、任务上传、货架数据等业务状态仍由 MainWindow 统一管理。
class CargoInspectionPage : public QWidget
{
public:
    explicit CargoInspectionPage(QWidget *parent = nullptr);

    void setSceneData(const WarehouseSceneData &data);

    void appendRunLog(const QString &text);
    void clearRunLog();
    void setWaypointLogText(const QString &text);
    void clearAiLog();
    void appendAiLog(const QString &text);
    void setAiLogText(const QString &text);

    // 姿态面板由 MainWindow 共享，但位置由当前项目页面决定。
    QRect attitudePanelGeometry() const;

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void setupUi();
    void setupConnections();
    void updatePageGeometry();
    void updateViewControlVisibility();

    SceneView *scene_view_ = nullptr;//Cargo 主场景，负责绘制仓库、无人机和轨迹

    QWidget *run_log_panel_ = nullptr;//日志面板
    QPlainTextEdit *run_log_view_ = nullptr;
    QWidget *waypoint_log_panel_ = nullptr;//航点日志面板
    QPlainTextEdit *waypoint_log_view_ = nullptr;
    QWidget *ai_log_panel_ = nullptr;//AI分析日志面板
    QPlainTextEdit *ai_log_view_ = nullptr;

    QWidget *view_mode_widget_ = nullptr;//视图模式控件
    QLabel *view_mode_left_label_ = nullptr;//2D标签
    QLabel *view_mode_right_label_ = nullptr;//3D标签
    QSlider *view_mode_slider_ = nullptr;//视图模式滑动条
    QWidget *view_perspective_widget_ = nullptr;//视角切换控件
    QSlider *view_perspective_slider_ = nullptr;//视角切换滑动条
    QWidget *view_2d_widget_ = nullptr;//2D视角切换控件
    QSlider *view_2d_slider_ = nullptr;//2D视角切换滑动条
};
