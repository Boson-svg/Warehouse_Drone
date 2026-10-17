#include "drone_warehouse/collaboration_grid_view.hpp"

#include "drone_warehouse/color_palette.hpp"

#include <QFont>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>

namespace
{
// ======================== 场地图形参数 ========================
// 下面所有带 Cm 后缀的数值单位都是厘米。
// 如果现场尺寸或示意图发生变化，优先修改这里，不需要改后面的绘图公式。

constexpr qreal kCmPerMeter = 100.0;

// 直圆跑道外接矩形：左边距 150 cm、底边距 125 cm、宽 150 cm、高 300 cm。
// 圆角半径为 75 cm，所以跑道上下两端正好是半圆。
constexpr qreal kTrackLeftCm = 150.0;
constexpr qreal kTrackBottomCm = 125.0;
constexpr qreal kTrackWidthCm = 150.0;
constexpr qreal kTrackHeightCm = 300.0;
constexpr qreal kTrackRadiusCm = 75.0;

// 图片左下角圆形区域：
//   圆的左外沿距离场地左边 75 cm；
//   圆的下外沿距离场地底边 75 cm；
//   圆的半径是 37.5 cm。
//
// 所以圆心的场地尺寸坐标是：
//   horizontal = 75 + 37.5 = 112.5 cm；
//   vertical   = 75 + 37.5 = 112.5 cm。
//
// 这个圆心只作为无人机红点的初始中心，同时也是 world_body 坐标系的 (0, 0)。
// 小车黄色圆的初始中心是跑道 A 点，不是这个圆心。
constexpr qreal kCircleLeftClearanceCm = 75.0;
constexpr qreal kCircleBottomClearanceCm = 75.0;
constexpr qreal kOuterCircleRadiusCm = 37.5;
constexpr qreal kOriginHorizontalCm =
    kCircleLeftClearanceCm + kOuterCircleRadiusCm;
constexpr qreal kOriginVerticalCm =
    kCircleBottomClearanceCm + kOuterCircleRadiusCm;
constexpr qreal kInnerCircleRadiusCm = 25.0;

// 实际显示范围只取圆形和直圆跑道的外沿，不包含 400x500 cm 场地的空白。
// 左、下边界来自圆形外沿，右、上边界来自直圆跑道外沿。
// 删除场地外框后按这个范围缩放，图形就能占满状态栏下方的可用区域。
constexpr qreal kDrawingLeftCm = kCircleLeftClearanceCm;
constexpr qreal kDrawingRightCm = kTrackLeftCm + kTrackWidthCm;
constexpr qreal kDrawingBottomCm = kCircleBottomClearanceCm;
constexpr qreal kDrawingTopCm = kTrackBottomCm + kTrackHeightCm;


// 跑道上的四个命名点。
// Horizontal 表示从场地左边向右量，Vertical 表示从场地底边向上量。
// 注意：这只是绘制示意图使用的场地尺寸坐标，不是 ROS 的 x/y 坐标。
constexpr qreal kPointAHorizontalCm = 150.0;
constexpr qreal kPointAVerticalCm = 200.0;
constexpr qreal kPointBHorizontalCm = 150.0;
constexpr qreal kPointBVerticalCm = 350.0;
constexpr qreal kPointCHorizontalCm = 300.0;
constexpr qreal kPointCVerticalCm = 350.0;
constexpr qreal kPointDHorizontalCm = 300.0;
constexpr qreal kPointDVerticalCm = 200.0;

// 实时位置圆点使用固定像素半径，窗口缩放时仍然清晰可见。
// 如果希望圆点跟随场地一起缩放，可以改成“实际厘米尺寸 * mapScale()”。
constexpr qreal kPositionMarkerRadiusPx = 10.0;

// 圆心十字和命名点同样使用像素尺寸，只承担视觉提示作用。
constexpr qreal kOriginCrossHalfSizePx = 9.0;
constexpr qreal kNamedPointRadiusPx = 4.0;

// 绘制 A/B/C/D 的公共函数。
// point 已经是窗口像素坐标，因此这里不再做厘米或米的换算。
void drawNamedPoint(
    QPainter &painter,
    const QPointF &point,
    const QString &name)
{
    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::GrayLight);
    painter.drawEllipse(
        point,
        kNamedPointRadiusPx,
        kNamedPointRadiusPx);

    // 标签向右下方稍微偏移，避免文字压住中心圆点。
    painter.setPen(ColorPalette::BlueLight);
    painter.drawText(
        point + QPointF(7.0, 5.0),
        name);
}
}

CollaborationGridView::CollaborationGridView(QWidget *parent)
    : QWidget(parent)
{
    // 只限制最小尺寸；实际场地会在 paintEvent 中按窗口大小等比例缩放。
    setMinimumSize(420, 360);
}

void CollaborationGridView::setdronePosition(
    double x,
    double y,
    double z)
{
    // 输入是地面站已经转换好的 world_body 坐标，单位为米，方向约定：
    //   x 增大 -> 画面向上移动；
    //   y 增大 -> 画面向左移动；
    //   z 只用于界面文字显示，不参与二维位置计算。
    display_x_ = x;
    display_y_ = y;
    altitude_ = z;

    // 数据变化后请求 Qt 重新执行 paintEvent。
    update();
}

void CollaborationGridView::setcarPosition(
    double x,
    double y,
    double z)
{
    // 小车使用 A 点作为自己的 (0, 0)，方向和单位与无人机相同。
    car_display_x_ = x;
    car_display_y_ = y;
    car_altitude_ = z;
    update();
}

QRectF CollaborationGridView::mapRect() const
{
    // MainWindow 顶部状态栏位于 y=16~68，无人机和小车信息位于 y=70~109。
    // 图形从 y=112 开始；上方直圆跑道直接顶到该位置，不再竖直居中。
    // 左右和底部只留出防止粗线及文字被裁切的安全距离。
    // 这些数值单位是窗口像素，不是厘米。
    const qreal left_margin = 24.0;
    const qreal right_margin = 24.0;
    const qreal top_margin = 112.0;
    const qreal bottom_margin = 16.0;

    // 防止窗口过小时得到 0 或负数，后续比例计算始终有合法结果。
    const qreal available_width =
        std::max<qreal>(
            1.0,
            width() - left_margin - right_margin);

    const qreal available_height =
        std::max<qreal>(
            1.0,
            height() - top_margin - bottom_margin);

    // 只根据实际图形包围范围计算缩放，不再把没有图形的场地空白算进去。
    // 宽高方向取较小比例，保证圆不会变成椭圆，现场尺寸关系也不会变形。
    const qreal drawing_width_cm =
        kDrawingRightCm - kDrawingLeftCm;
    const qreal drawing_height_cm =
        kDrawingTopCm - kDrawingBottomCm;
    const qreal scale = std::min(
        available_width / drawing_width_cm,
        available_height / drawing_height_cm);

    // 图形不再水平居中，直接贴近左侧安全边距。
    // 原来的“剩余宽度 / 2”会在宽屏窗口中产生很大一块左侧空白。
    const qreal drawing_left = left_margin;
    const qreal drawing_top = top_margin;

    // mapPoint 仍使用完整场地尺寸坐标，因此这里反推出 400x500 cm
    // 虚拟场地矩形的位置。该矩形可以超出窗口，但不会被实际绘制出来。
    const qreal map_left =
        drawing_left - kDrawingLeftCm * scale;
    const qreal map_top =
        drawing_top - (kMapHeightCm - kDrawingTopCm) * scale;

    return QRectF(
        map_left,
        map_top,
        kMapWidthCm * scale,
        kMapHeightCm * scale);
}

QPointF CollaborationGridView::mapPoint(
    qreal horizontal_cm,
    qreal vertical_cm) const
{
    const QRectF map = mapRect();
    const qreal scale = mapScale();

    // 场地尺寸图使用“左下角量尺寸”的方式：
    //   horizontal_cm 从场地左边向右增大；
    //   vertical_cm   从场地底边向上增大。
    //
    // Qt 窗口像素坐标的 Y 轴向下增大，所以竖直坐标必须使用
    // map.bottom() - vertical_cm * scale 才能实现“向上为正”。
    return QPointF(
        map.left() + horizontal_cm * scale,
        map.bottom() - vertical_cm * scale);
}

QRectF CollaborationGridView::mapObjectRect(
    qreal left_cm,
    qreal bottom_cm,
    qreal width_cm,
    qreal height_cm) const
{
    // QRectF 需要左上角，而现场尺寸通常从左下角给出。
    // 因此先把“底边 + 高度”换算成矩形左上角的场地坐标。
    const QPointF top_left =
        mapPoint(left_cm, bottom_cm + height_cm);

    const qreal scale = mapScale();

    return QRectF(
        top_left.x(),
        top_left.y(),
        width_cm * scale,
        height_cm * scale);
}

qreal CollaborationGridView::mapScale() const
{
    // mapRect 仍表示 400x500 cm 虚拟场地，因此用宽度除以 400 cm 得到比例。
    // 返回值含义：现场 1 cm 在当前窗口中对应多少像素。
    return mapRect().width() / kMapWidthCm;
}

void CollaborationGridView::drawInspectionTrack(
    QPainter &painter) const
{
    const qreal scale = mapScale();

    // 先得到跑道的外接矩形，再通过 75 cm 圆角生成直圆跑道。
    const QRectF track_rect =
        mapObjectRect(
            kTrackLeftCm,
            kTrackBottomCm,
            kTrackWidthCm,
            kTrackHeightCm);

    QPainterPath track;
    track.addRoundedRect(
        track_rect,
        kTrackRadiusCm * scale,
        kTrackRadiusCm * scale,
        Qt::AbsoluteSize);

    painter.setBrush(Qt::NoBrush);
    // 圆头和圆角连接可避免粗线在转弯处出现尖角。
    // 与 Animal 网格外边界一致，主要固定轮廓使用 BlueLight。
    QPen track_pen(ColorPalette::BlueLight, 3.0);
    track_pen.setCapStyle(Qt::RoundCap);
    track_pen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(track_pen);

    painter.drawPath(track);
}

void CollaborationGridView::drawCircularArea(
    QPainter &painter) const
{
    const qreal scale = mapScale();

    // 左下圆心既是图形中心，也是实时位置坐标系的原点。
    const QPointF center =
        mapPoint(
            kOriginHorizontalCm,
            kOriginVerticalCm);

    painter.setBrush(Qt::NoBrush);

    // 外圆半径是 37.5 cm，圆心为 (112.5, 112.5) cm。
    // 外圆与跑道使用相同颜色、相同线宽，保持固定场地图形风格一致。
    // 外圆同样属于主要边界，与跑道统一使用 BlueLight。
    QPen outer_circle_pen(ColorPalette::BlueLight, 3.0);
    outer_circle_pen.setCapStyle(Qt::RoundCap);
    outer_circle_pen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(outer_circle_pen);
    painter.drawEllipse(
        center,
        kOuterCircleRadiusCm * scale,
        kOuterCircleRadiusCm * scale);

    // 内圆仅用于还原图片样式，半径可独立调整。
    // 内圆属于辅助线，使用 Animal 网格内部线相同的 BlueGrayDark。
    QPen inner_circle_pen(ColorPalette::BlueGrayDark, 2.0);
    inner_circle_pen.setCapStyle(Qt::RoundCap);
    painter.setPen(inner_circle_pen);
    painter.drawEllipse(
        center,
        kInnerCircleRadiusCm * scale,
        kInnerCircleRadiusCm * scale);

    // 原点十字采用固定像素长度，红色无人机圆点会绘制在它上面。
    painter.drawLine(
        center + QPointF(-kOriginCrossHalfSizePx, 0.0),
        center + QPointF(kOriginCrossHalfSizePx, 0.0));

    painter.drawLine(
        center + QPointF(0.0, -kOriginCrossHalfSizePx),
        center + QPointF(0.0, kOriginCrossHalfSizePx));
}

void CollaborationGridView::drawCarMarker(
    QPainter &painter) const
{
    const qreal scale = mapScale();
    // 小车自己的坐标原点是 A 点，不使用无人机所在的左下圆心。
    const QPointF car_origin =
        mapPoint(
            kPointAHorizontalCm,
            kPointAVerticalCm);

    // 小车米坐标转换为窗口像素坐标：
    //
    //   car_display_x_ * 100     把米转换为厘米；
    //   再乘 scale              把厘米转换为像素。
    //
    // 坐标方向（注意这里的零点是 A）：
    //   x+ 向上，而 Qt 屏幕 Y 向下，所以屏幕 Y 要减去 x；
    //   y+ 向左，而 Qt 屏幕 X 向右，所以屏幕 X 要减去 y。
    const QPointF car_point(
        car_origin.x() -
            car_display_y_ * kCmPerMeter * scale,
        car_origin.y() -
            car_display_x_ * kCmPerMeter * scale);

    // 与 Animal 的位置标记一致，圆点不描边，直接在深色背景上使用高亮色。
    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::Yellow);
    painter.drawEllipse(
        car_point,
        kPositionMarkerRadiusPx,
        kPositionMarkerRadiusPx);
}

void CollaborationGridView::paintEvent(QPaintEvent *event)
{
    // 先让 QWidget 完成默认绘制流程，再覆盖绘制自定义场地。
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(
        QPainter::SmoothPixmapTransform,
        true);

    // 整个 Collaboration 页面背景。
    painter.fillRect(
        rect(),
        ColorPalette::BlueBlack);


    // 第 1 层：绘制不随 ROS 数据变化的固定场地图形。
    drawInspectionTrack(painter);
    drawCircularArea(painter);

    // 第 2 层：先画黄色小车圆。
    // 小车 (0,0) 的中心就是 A 点，此时还没有绘制 A 点的浅色中心圆和字母。
    drawCarMarker(painter);

    // 第 3 层：最后绘制 A/B/C/D 的浅色中心圆和蓝色字母。
    // A 点会画在黄色小车圆上层，因此小车 (0,0) 与 A 点中心完全重合，
    // 同时仍然可以看到 A 点的小圆以及旁边的字母 A。
    QFont point_font = painter.font();
    point_font.setPixelSize(14);
    point_font.setBold(true);
    painter.setFont(point_font);

    drawNamedPoint(
        painter,
        mapPoint(
            kPointAHorizontalCm,
            kPointAVerticalCm),
        "A");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointBHorizontalCm,
            kPointBVerticalCm),
        "B");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointCHorizontalCm,
            kPointCVerticalCm),
        "C");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointDHorizontalCm,
            kPointDVerticalCm),
        "D");

    const qreal scale = mapScale();
    const QPointF origin =
        mapPoint(
            kOriginHorizontalCm,
            kOriginVerticalCm);

    // 无人机仍从左下圆心换算；它与小车只共用方向和单位，不共用原点。
    // 默认 (0, 0) m 不产生偏移，所以红点位于左下圆心。
    const QPointF drone_point(
        origin.x() -
            display_y_ * kCmPerMeter * scale,
        origin.y() -
            display_x_ * kCmPerMeter * scale);

    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::Red);
    painter.drawEllipse(
        drone_point,
        kPositionMarkerRadiusPx,
        kPositionMarkerRadiusPx);

    // 顶部显示无人机 world_body 坐标和小车本地坐标。分成两行可保证较窄窗口也能完整显示。
    // 颜色、字号和字重与 Animal 画板的信息行保持一致。
    QFont info_font = painter.font();
    info_font.setPixelSize(17);
    info_font.setBold(false);
    painter.setFont(info_font);
    painter.setPen(ColorPalette::GrayLight);
    painter.drawText(
        QRectF(
            24.0,
            70.0,
            width() - 48.0,
            20.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QString(
            "drone  x=%1 m  y=%2 m  z=%3 m")
            .arg(display_x_, 0, 'f', 2)
            .arg(display_y_, 0, 'f', 2)
            .arg(altitude_, 0, 'f', 2));

    painter.drawText(
        QRectF(
            24.0,
            90.0,
            width() - 48.0,
            20.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QString(
            "car       x=%1 m  y=%2 m  z=%3 m")
            .arg(car_display_x_, 0, 'f', 2)
            .arg(car_display_y_, 0, 'f', 2)
            .arg(car_altitude_, 0, 'f', 2));
}