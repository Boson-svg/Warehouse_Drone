#include "drone_warehouse/scene_view.hpp"
#include "drone_warehouse/color_palette.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

SceneView::SceneView(QWidget *parent)
    : QWidget(parent)
{
    setMouseTracking(true);
}

void SceneView::setViewMode(ViewMode mode)
{
    view_mode_ = mode;
    update();
}

void SceneView::setViewPerspectiveMode(ViewPerspectiveMode mode)
{
    view_perspective_mode_ = mode;
    update();
}

void SceneView::setView2DMode(View2DMode mode)
{
    view_2d_mode_ = mode;
    update();
}

ViewMode SceneView::viewMode() const
{
    return view_mode_;
}

void SceneView::setSceneData(const WarehouseSceneData &data)
{
    scene_data_ = data;
    update();
}

void SceneView::resetView()
{
    zoom_ = 1.0;
    pan_offset_ = QPointF(0.0, 0.0);
    update();
}

void SceneView::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    //统一绘制背景、网格、货架、轨迹和无人机
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    drawBackground(painter);
    drawGrid(painter);
    drawShelves(painter);
    drawTrajectory(painter);
    drawDrone(painter);
}

void SceneView::wheelEvent(QWheelEvent *event)
{
    //处理滚轮缩放
    const QPoint angle_delta = event->angleDelta();
    if (angle_delta.y() > 0)
    {
        zoom_ *= 1.1;
    }
    else
    {
        zoom_ *= 0.9;
    }

    if (zoom_ < 0.2)
    {
        zoom_ = 0.2;
    }
    if (zoom_ > 5.0)
    {
        zoom_ = 5.0;
    }

    update();
}

void SceneView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        dragging_ = true;
        last_mouse_pos_ = event->pos();
    }
}

void SceneView::mouseMoveEvent(QMouseEvent *event)
{
    if (!dragging_)
    {
        return;
    }

    const QPoint delta = event->pos() - last_mouse_pos_;
    pan_offset_ += QPointF(delta.x(), delta.y());
    last_mouse_pos_ = event->pos();
    update();
}

void SceneView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        dragging_ = false;
    }
}

void SceneView::drawBackground(QPainter &painter)
{
    painter.fillRect(rect(), ColorPalette::BlueBlack);
}

void SceneView::drawGrid(QPainter &painter)
{
    painter.save();
    painter.setPen(QPen(ColorPalette::BlueGrayDark, 1));

    //网格总区域：500x500，网格间距：50，所以从-500到500每隔50画一条线，形成20x20的网格
    for (int x = -500; x <= 500; x += 50)
    {
        //在op2D模式下做平面映射，在Pseudo3D模式下做简化斜投影，所以网格线也要根据当前视图模式投影到屏幕坐标
        painter.drawLine(projectPoint(x, -500, 0.0), projectPoint(x, 500, 0.0));
    }

    for (int y = -500; y <= 500; y += 50)
    {
        painter.drawLine(projectPoint(-500, y, 0.0), projectPoint(500, y, 0.0));
    }

    painter.restore();
}

void SceneView::drawShelves(QPainter &painter)
{
    painter.save();
    //painter.setPen(Qt::NoPen);
    painter.setPen(QPen(ColorPalette::BlueLight, 1));

    for (const ShelfBlock &shelf : scene_data_.shelves)
    {
        const QRectF &r = shelf.base_rect;

        //底面四个点
        QPointF p1 = projectPoint(r.left(), r.top(), 0.0);
        QPointF p2 = projectPoint(r.right(), r.top(), 0.0);
        QPointF p3 = projectPoint(r.right(), r.bottom(), 0.0);
        QPointF p4 = projectPoint(r.left(), r.bottom(), 0.0);

        //顶面四个点（高度由货架的height属性决定）
        QPointF p1_top = projectPoint(r.left(), r.top(), shelf.height);
        QPointF p2_top = projectPoint(r.right(), r.top(), shelf.height);
        QPointF p3_top = projectPoint(r.right(), r.bottom(), shelf.height);
        QPointF p4_top = projectPoint(r.left(), r.bottom(), shelf.height);

        //货架用一个半透明的颜色填充，顶面和四个侧面分别画成一个多边形
        painter.setBrush(shelf.color);
        painter.drawPolygon(QPolygonF() << p1_top << p2_top << p3_top << p4_top);//顶面
        painter.drawPolygon(QPolygonF() << p1 << p2 << p2_top << p1_top);//前面
        painter.drawPolygon(QPolygonF() << p4 << p3 << p3_top << p4_top);//后面
        painter.drawPolygon(QPolygonF() << p1 << p4 << p4_top << p1_top);//左面
        painter.drawPolygon(QPolygonF() << p2 << p3 << p3_top << p2_top);//右面

        // painter.drawLine(p1, p2);
        // painter.drawLine(p2, p3);
        // painter.drawLine(p3, p4);
        // painter.drawLine(p4, p1);
    }

    painter.restore();
}

void SceneView::drawTrajectory(QPainter &painter)
{
    if (scene_data_.trajectory.size() < 2)
    {
        return;
    }

    painter.save();

    QPainterPath air_path;
    QPainterPath ground_path;

    const TrajectoryPoint &first = scene_data_.trajectory.first();
    air_path.moveTo(projectPoint(first.x, first.y, first.z));
    ground_path.moveTo(projectPoint(first.x, first.y, 0.0));

    for (int i = 1; i < scene_data_.trajectory.size(); ++i)
    {
        const TrajectoryPoint &pt = scene_data_.trajectory[i];
        air_path.lineTo(projectPoint(pt.x, pt.y, pt.z));
        ground_path.lineTo(projectPoint(pt.x, pt.y, 0.0));
    }

    //飞行轨迹用亮色实线表示，地面投影用暗色虚线表示（线宽分别为2，1）
    painter.setPen(QPen(ColorPalette::Cyan, 2));
    painter.drawPath(air_path);

    painter.setPen(QPen(ColorPalette::withAlpha(ColorPalette::CyanBlue, 80), 1, Qt::DashLine));
    painter.drawPath(ground_path);

    painter.restore();
}

void SceneView::drawDrone(QPainter &painter)
{
    const DroneState &drone = scene_data_.drone_state;
    const QPointF center = projectPoint(drone.pose.x, drone.pose.y, drone.pose.z);

    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::AquaGreen);
    //无人机用一个简单的圆点表示，大小固定不随缩放变化
    painter.drawEllipse(center, 6, 6);
    painter.restore();
}

QPointF SceneView::projectPoint(double x, double y, double z) const
{
    //在op2D模式下做平面映射，在Pseudo3D模式下做简化斜投影
    const QPointF screen_center(width() * 0.5, height() * 0.5);

    double screen_x = 0.0;
    double screen_y = 0.0;

    if (view_mode_ == ViewMode::Top2D)
    {
        if (view_2d_mode_ == View2DMode::Perspectivetop)
        {
            screen_x = x;
            screen_y = y;
        }
        else if (view_2d_mode_ == View2DMode::Perspective90)
        {
            screen_x = y;
            screen_y = -z;
        }
        else
        {
            screen_x = -y;
            screen_y = -z;
        }
    }
    else
    {
        if (view_perspective_mode_ == ViewPerspectiveMode::Perspective225)
        {
            screen_x = x - y;
            screen_y = (x + y) * 0.4 - z;
        }
        else if (view_perspective_mode_ == ViewPerspectiveMode::Perspective315)
        {
            screen_x = -(x + y);
            screen_y = (x - y) * 0.4 - z;
        }
        else if (view_perspective_mode_ == ViewPerspectiveMode::Perspective45)
        {
            screen_x = -x + y;
            screen_y = -(x + y) * 0.4 - z;
        }
        else
        {
            screen_x = x + y;
            screen_y = (-x + y) * 0.4 - z;
        }
    }

    return QPointF(screen_center.x() + screen_x * zoom_ + pan_offset_.x(),
                   screen_center.y() + screen_y * zoom_ + pan_offset_.y());
}