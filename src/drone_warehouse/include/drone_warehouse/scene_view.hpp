#pragma once

#include <QPoint>
#include <QWidget>

#include "drone_warehouse/models.hpp"

//视图档位切换
enum class ViewMode
{
    Top2D,
    Pseudo3D
};

enum class ViewPerspectiveMode
{
    Perspective225,
    Perspective315,
    Perspective45,
    Perspective135
};

enum class View2DMode
{
    Perspectivetop,
    Perspective90,
    Perspective0
};

class SceneView : public QWidget
{
    Q_OBJECT

public:
    explicit SceneView(QWidget *parent = nullptr);

    //设置视图模式
    void setViewMode(ViewMode mode);
    void setViewPerspectiveMode(ViewPerspectiveMode mode);
    void setView2DMode(View2DMode mode);
    ViewMode viewMode() const;

    //外部传入最新场景数据
    void setSceneData(const WarehouseSceneData &data);
    //恢复默认缩放和平移
    void resetView();

protected:
    void paintEvent(QPaintEvent *event) override;//统一绘制背景、网格、货架、轨迹和无人机
    void wheelEvent(QWheelEvent *event) override;//处理滚轮缩放
    //处理拖动平移
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void drawBackground(QPainter &painter);//绘制背景
    void drawGrid(QPainter &painter);//绘制网格
    void drawShelves(QPainter &painter);//绘制货架
    void drawTrajectory(QPainter &painter);//绘制轨迹
    void drawDrone(QPainter &painter);//绘制无人机

    //这是伪3D方案的关键函数，用于把场景坐标投影到屏幕坐标
    QPointF projectPoint(double x, double y, double z) const;

private:
    ViewMode view_mode_ = ViewMode::Pseudo3D;
    ViewPerspectiveMode view_perspective_mode_ = ViewPerspectiveMode::Perspective225;
    View2DMode view_2d_mode_ = View2DMode::Perspectivetop;

    WarehouseSceneData scene_data_;//当前场景数据

    double zoom_ = 1.0;//缩放因子
    QPointF pan_offset_;//当前拖动偏移量

    bool dragging_ = false;//是否正在拖动
    QPoint last_mouse_pos_;//上次鼠标位置
};