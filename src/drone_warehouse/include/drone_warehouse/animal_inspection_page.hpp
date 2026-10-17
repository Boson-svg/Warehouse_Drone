#pragma once

#include "drone_warehouse/models.hpp"

#include <QRect>
#include <QVector>
#include <QWidget>

class AnimalGridView;
class QListWidget;
class QPlainTextEdit;
class QResizeEvent;

// 动物巡检页面管理固定二维栅格、运行日志和识别结果。
// 页面不订阅 ROS，也不启动任务；这些动作仍由 MainWindow 统一编排。
class AnimalInspectionPage : public QWidget
{
public:
    explicit AnimalInspectionPage(QWidget *parent = nullptr);

    void setPosition(double x, double y, double z);
    QVector<WorldCoord> plannedWorldPoints(double altitude, double yaw) const;

    void appendRunLog(const QString &text);
    void clearRunLog();
    void appendRecognitionRecord(
        const QString &target_id,
        const QString &detail,
        const QString &time_text);

    // 姿态面板由 MainWindow 共享，但位置由当前项目页面决定。
    QRect attitudePanelGeometry() const;

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void setupUi();
    void updatePageGeometry();
    QRect sidebarGeometry() const;

    AnimalGridView *grid_view_ = nullptr;//Animal 固定 7x9 二维栅格画板
    QWidget *run_log_panel_ = nullptr;
    QPlainTextEdit *run_log_view_ = nullptr;
    QWidget *result_panel_ = nullptr;//Animal 右上角的视觉伺服成功目标列表
    QListWidget *result_list_ = nullptr;
};
