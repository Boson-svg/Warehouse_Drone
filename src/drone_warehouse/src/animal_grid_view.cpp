#include "drone_warehouse/animal_grid_view.hpp"

#include "drone_warehouse/color_palette.hpp"

#include <QFont>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QQueue>

#include <algorithm>
#include <cmath>

AnimalGridView::AnimalGridView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(420, 360);
    setCursor(Qt::PointingHandCursor);

    // 初次进入 Animal 项目时就显示完整路线，不必先点击一个格子。
    rebuildPlannedPath();
}

void AnimalGridView::setPosition(double x, double y, double z)
{
    // 输入已经由 MainWindow 转换为 world_body；这里只保留 Animal 原有的二维映射：
    // world_body x 正方向映射到画面右侧，world_body y 正方向映射到画面上方。
    // 这里不再减起始位置，TF 原点偏移只允许在 MainWindow 中做一次。
    // 画板路线自身使用“向上为 x、向左为 y”的右下角原点坐标。
    // Current mapping matches the route and Collaboration view:
    // x+ is up and y+ is left; do not swap or negate the input axes.
    display_x_ = x;
    display_y_ = y;
    altitude_ = z;
    update();
}

QVector<WorldCoord>
AnimalGridView::plannedWorldPoints(double altitude, double yaw) const
{
    QVector<WorldCoord> points;
    points.reserve(planned_path_.size());

    for (const GridCell &cell : planned_path_)
    {
        const DisplayCoord display =
            toDisplayCoord(cell.row, cell.column);

        WorldCoord point;
        point.x = display.row * kCellSizeM;
        point.y = display.column * kCellSizeM;
        point.z = altitude;
        point.yaw = yaw;
        points.push_back(point);
    }
    return points;
}

QRectF AnimalGridView::gridRect() const
{
    // 保留顶部位置文字、左侧行标和底部列标后，以等边格铺满剩余区域。
    const qreal left_margin = 64.0;
    const qreal right_margin = 28.0;
    const qreal top_margin = 112.0;
    const qreal bottom_margin = 58.0;
    const qreal available_width =
        std::max<qreal>(0.0, width() - left_margin - right_margin);
    const qreal available_height =
        std::max<qreal>(0.0, height() - top_margin - bottom_margin);
    const qreal cell_side = std::max<qreal>(
        1.0,
        std::min(available_width / kColumns,
                 available_height / kRows));
    const qreal grid_width = cell_side * kColumns;
    const qreal grid_height = cell_side * kRows;
    // Animal 主画板占左侧区域，网格本身也靠左排列，右侧留给信息栏。
    const qreal left = left_margin;
    const qreal top =
        top_margin + (available_height - grid_height) / 2.0;
    return QRectF(left, top, grid_width, grid_height);
}

QRectF AnimalGridView::cellRect(int row, int column) const
{
    const QRectF grid = gridRect();
    const qreal cell_width = grid.width() / kColumns;
    const qreal cell_height = grid.height() / kRows;
    return QRectF(
        grid.left() + column * cell_width,
        grid.top() + row * cell_height,
        cell_width,
        cell_height);
}

QPointF AnimalGridView::cellCenter(int row, int column) const
{
    return cellRect(row, column).center();
}

int AnimalGridView::cellIndex(int row, int column) const
{
    return row * kColumns + column;
}

bool AnimalGridView::isInside(int row, int column) const
{
    return row >= 0 && row < kRows &&
           column >= 0 && column < kColumns;
}

bool AnimalGridView::isBlocked(int row, int column) const
{
    return blocked_cells_.contains(cellIndex(row, column));
}

bool AnimalGridView::pointToCell(
    const QPointF &point,
    int &row,
    int &column) const
{
    const QRectF grid = gridRect();
    if (!grid.contains(point))
    {
        return false;
    }

    const qreal cell_width = grid.width() / kColumns;
    const qreal cell_height = grid.height() / kRows;
    column = static_cast<int>(
        (point.x() - grid.left()) / cell_width);
    row = static_cast<int>(
        (point.y() - grid.top()) / cell_height);
    column = std::min(column, kColumns - 1);
    row = std::min(row, kRows - 1);
    return isInside(row, column);
}

AnimalGridView::DisplayCoord
AnimalGridView::toDisplayCoord(int row, int column) const
{
    // 右下角显示为 (0,0)，左上角显示为 (6,8)。
    return {kRows - 1 - row, kColumns - 1 - column};
}

QSet<int>
AnimalGridView::computeReachableCells(const GridCell &start) const
{
    QSet<int> reachable;
    if (!isInside(start.row, start.column) ||
        isBlocked(start.row, start.column))
    {
        return reachable;
    }

    QQueue<GridCell> queue;
    queue.enqueue(start);
    reachable.insert(cellIndex(start.row, start.column));

    const int row_offset[4] = {-1, 1, 0, 0};
    const int column_offset[4] = {0, 0, -1, 1};
    while (!queue.isEmpty())
    {
        const GridCell current = queue.dequeue();
        for (int direction = 0; direction < 4; ++direction)
        {
            const int next_row =
                current.row + row_offset[direction];
            const int next_column =
                current.column + column_offset[direction];
            if (!isInside(next_row, next_column) ||
                isBlocked(next_row, next_column))
            {
                continue;
            }

            const int index = cellIndex(next_row, next_column);
            if (reachable.contains(index))
            {
                continue;
            }
            reachable.insert(index);
            queue.enqueue({next_row, next_column});
        }
    }
    return reachable;
}

QVector<AnimalGridView::GridCell>
AnimalGridView::buildSnakeTargets(const QSet<int> &reachable) const
{
    QVector<GridCell> targets;
    // 从右下角开始逐行蛇形扫描，和 drone_qt 当前路线顺序一致。
    for (int row = kRows - 1; row >= 0; --row)
    {
        const int offset_from_bottom = kRows - 1 - row;
        if (offset_from_bottom % 2 == 0)
        {
            for (int column = kColumns - 1; column >= 0; --column)
            {
                if (reachable.contains(cellIndex(row, column)))
                {
                    targets.push_back({row, column});
                }
            }
        }
        else
        {
            for (int column = 0; column < kColumns; ++column)
            {
                if (reachable.contains(cellIndex(row, column)))
                {
                    targets.push_back({row, column});
                }
            }
        }
    }
    return targets;
}

QVector<AnimalGridView::GridCell>
AnimalGridView::findPathBfs(
    const GridCell &start,
    const GridCell &goal) const
{
    QVector<GridCell> empty;
    if (!isInside(start.row, start.column) ||
        !isInside(goal.row, goal.column) ||
        isBlocked(start.row, start.column) ||
        isBlocked(goal.row, goal.column))
    {
        return empty;
    }

    const int total = kRows * kColumns;
    QVector<bool> visited(total, false);
    QVector<int> parent(total, -1);
    QQueue<GridCell> queue;
    queue.enqueue(start);
    visited[cellIndex(start.row, start.column)] = true;

    const int row_offset[4] = {-1, 1, 0, 0};
    const int column_offset[4] = {0, 0, -1, 1};
    while (!queue.isEmpty())
    {
        const GridCell current = queue.dequeue();
        if (current == goal)
        {
            break;
        }

        for (int direction = 0; direction < 4; ++direction)
        {
            const int next_row =
                current.row + row_offset[direction];
            const int next_column =
                current.column + column_offset[direction];
            if (!isInside(next_row, next_column) ||
                isBlocked(next_row, next_column))
            {
                continue;
            }

            const int next_index =
                cellIndex(next_row, next_column);
            if (visited[next_index])
            {
                continue;
            }
            visited[next_index] = true;
            parent[next_index] =
                cellIndex(current.row, current.column);
            queue.enqueue({next_row, next_column});
        }
    }

    const int goal_index = cellIndex(goal.row, goal.column);
    if (!visited[goal_index])
    {
        return empty;
    }

    QVector<GridCell> reversed;
    for (int current = goal_index;
         current != -1;
         current = parent[current])
    {
        reversed.push_back(
            {current / kColumns, current % kColumns});
    }

    QVector<GridCell> path;
    path.reserve(reversed.size());
    for (int index = reversed.size() - 1; index >= 0; --index)
    {
        path.push_back(reversed[index]);
    }
    return path;
}

int AnimalGridView::findTargetIndex(
    const QVector<GridCell> &cells,
    const GridCell &target) const
{
    for (int index = 0; index < cells.size(); ++index)
    {
        if (cells[index] == target)
        {
            return index;
        }
    }
    return -1;
}

void AnimalGridView::rebuildPlannedPath()
{
    planned_path_.clear();

    GridCell start{route_start_row_, route_start_column_};
    if (!isInside(start.row, start.column) ||
        isBlocked(start.row, start.column))
    {
        // 用户把当前起点设为禁行格后，回退到固定的右下角原点。
        start = {kRows - 1, kColumns - 1};
    }
    if (isBlocked(start.row, start.column))
    {
        return;
    }

    const QSet<int> reachable = computeReachableCells(start);
    QVector<GridCell> targets = buildSnakeTargets(reachable);
    if (targets.isEmpty())
    {
        return;
    }

    int start_index = findTargetIndex(targets, start);
    if (start_index < 0)
    {
        targets.prepend(start);
        start_index = 0;
    }

    QVector<GridCell> ordered_targets;
    ordered_targets.reserve(targets.size());
    for (int index = start_index; index < targets.size(); ++index)
    {
        ordered_targets.push_back(targets[index]);
    }
    for (int index = 0; index < start_index; ++index)
    {
        ordered_targets.push_back(targets[index]);
    }

    GridCell current = start;
    planned_path_.push_back(current);
    for (const GridCell &target : ordered_targets)
    {
        if (target == current)
        {
            continue;
        }

        const QVector<GridCell> segment =
            findPathBfs(current, target);
        for (int index = 1; index < segment.size(); ++index)
        {
            planned_path_.push_back(segment[index]);
        }
        if (!segment.isEmpty())
        {
            current = target;
        }
    }

    // 最后一段回到起点，生成现有逻辑使用的闭环路线。
    if (!(current == start))
    {
        const QVector<GridCell> return_segment =
            findPathBfs(current, start);
        for (int index = 1;
             index < return_segment.size();
             ++index)
        {
            planned_path_.push_back(return_segment[index]);
        }
    }
}

void AnimalGridView::drawDirectionArrow(
    QPainter &painter,
    const QPointF &start,
    const QPointF &end) const
{
    const qreal dx = end.x() - start.x();
    const qreal dy = end.y() - start.y();
    const qreal length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0)
    {
        return;
    }

    const QPointF middle(
        (start.x() + end.x()) / 2.0,
        (start.y() + end.y()) / 2.0);
    const qreal unit_x = dx / length;
    const qreal unit_y = dy / length;
    const qreal arrow_length = 12.0;
    const qreal arrow_width = 6.0;
    const QPointF tip(
        middle.x() + unit_x * arrow_length / 2.0,
        middle.y() + unit_y * arrow_length / 2.0);
    const QPointF tail(
        middle.x() - unit_x * arrow_length / 2.0,
        middle.y() - unit_y * arrow_length / 2.0);
    const QPointF left(
        tail.x() - unit_y * arrow_width,
        tail.y() + unit_x * arrow_width);
    const QPointF right(
        tail.x() + unit_y * arrow_width,
        tail.y() - unit_x * arrow_width);

    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::Cyan);
    painter.drawPolygon(QPolygonF() << tip << left << right);
    painter.restore();
}

void AnimalGridView::drawPlannedPath(QPainter &painter) const
{
    if (planned_path_.size() < 2)
    {
        return;
    }

    QPen route_pen(ColorPalette::Cyan, 2.0);
    route_pen.setJoinStyle(Qt::RoundJoin);
    route_pen.setCapStyle(Qt::RoundCap);
    painter.setPen(route_pen);

    QPointF segment_start =
        cellCenter(planned_path_.front().row,
                   planned_path_.front().column);
    QPointF previous_point = segment_start;
    int previous_row_delta =
        planned_path_[1].row - planned_path_[0].row;
    int previous_column_delta =
        planned_path_[1].column - planned_path_[0].column;

    for (int index = 1; index < planned_path_.size(); ++index)
    {
        const QPointF from =
            cellCenter(planned_path_[index - 1].row,
                       planned_path_[index - 1].column);
        const QPointF to =
            cellCenter(planned_path_[index].row,
                       planned_path_[index].column);
        painter.drawLine(from, to);

        const int row_delta =
            planned_path_[index].row -
            planned_path_[index - 1].row;
        const int column_delta =
            planned_path_[index].column -
            planned_path_[index - 1].column;
        if (row_delta != previous_row_delta ||
            column_delta != previous_column_delta)
        {
            drawDirectionArrow(painter, segment_start, from);
            segment_start = from;
            previous_row_delta = row_delta;
            previous_column_delta = column_delta;
        }
        previous_point = to;
    }
    drawDirectionArrow(painter, segment_start, previous_point);
}

void AnimalGridView::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), ColorPalette::BlueBlack);

    const QRectF grid = gridRect();

    // 先画禁行格，再画路线和网格，保证边界始终清楚。
    for (int index : blocked_cells_)
    {
        const int row = index / kColumns;
        const int column = index % kColumns;
        painter.fillRect(
            cellRect(row, column),
            ColorPalette::withAlpha(ColorPalette::Gray, 190));
    }
    drawPlannedPath(painter);

    painter.setPen(QPen(ColorPalette::BlueGrayDark, 1.0));
    for (int row = 0; row <= kRows; ++row)
    {
        const qreal y =
            grid.top() + row * grid.height() / kRows;
        painter.drawLine(
            QPointF(grid.left(), y),
            QPointF(grid.right(), y));
    }
    for (int column = 0; column <= kColumns; ++column)
    {
        const qreal x =
            grid.left() + column * grid.width() / kColumns;
        painter.drawLine(
            QPointF(x, grid.top()),
            QPointF(x, grid.bottom()));
    }
    painter.setPen(QPen(ColorPalette::BlueLight, 1.0));
    painter.drawRect(grid);

    QFont label_font = painter.font();
    label_font.setPixelSize(16);
    label_font.setBold(true);
    painter.setFont(label_font);
    painter.setPen(ColorPalette::BlueLight);

    // 行列编号沿用 drone_qt：左侧 B7..B1，底部 A1..A9。
    for (int row = 0; row < kRows; ++row)
    {
        const QRectF label_rect(
            grid.left() - 52.0,
            grid.top() + row * grid.height() / kRows,
            44.0,
            grid.height() / kRows);
        painter.drawText(
            label_rect,
            Qt::AlignCenter,
            QString("B%1").arg(kRows - row));
    }
    for (int column = 0; column < kColumns; ++column)
    {
        const QRectF label_rect(
            grid.left() + column * grid.width() / kColumns,
            grid.bottom() + 8.0,
            grid.width() / kColumns,
            30.0);
        painter.drawText(
            label_rect,
            Qt::AlignCenter,
            QString("A%1").arg(column + 1));
    }

    const QPointF origin =
        cellCenter(kRows - 1, kColumns - 1);
    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::Red);
    painter.drawEllipse(origin, 10.0, 10.0);

    // 实时位置使用与网格格距一致的比例，窗口缩放后标记仍与格子对齐。
    const qreal pixels_per_x_meter =
        (grid.height() / kRows) / kCellSizeM;
    const qreal pixels_per_y_meter =
        (grid.width() / kColumns) / kCellSizeM;
    const QPointF drone_point(
        origin.x() - display_y_ * pixels_per_y_meter,
        origin.y() - display_x_ * pixels_per_x_meter);
    painter.setBrush(ColorPalette::AquaGreen);
    painter.drawEllipse(drone_point, 7.0, 7.0);

    QFont info_font = painter.font();
    info_font.setPixelSize(17);
    info_font.setBold(false);
    painter.setFont(info_font);
    painter.setPen(ColorPalette::GrayLight);
    painter.drawText(
        QRectF(24.0, 74.0, width() - 48.0, 30.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QString("Animal 2D    x=%1 m    y=%2 m    z=%3 m    route=%4 points")
            .arg(display_x_, 0, 'f', 2)
            .arg(display_y_, 0, 'f', 2)
            .arg(altitude_, 0, 'f', 2)
            .arg(planned_path_.size()));
}

void AnimalGridView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton)
    {
        QWidget::mousePressEvent(event);
        return;
    }

    int row = -1;
    int column = -1;
    if (!pointToCell(event->localPos(), row, column))
    {
        QWidget::mousePressEvent(event);
        return;
    }

    const int index = cellIndex(row, column);
    if (blocked_cells_.contains(index))
    {
        blocked_cells_.remove(index);
    }
    else
    {
        blocked_cells_.insert(index);
    }

    // 与 drone_qt 当前交互一致：被点击的格子同时成为候选路线起点；
    // 若该格刚被设成禁行格，规划器会自动回退到右下角原点。
    route_start_row_ = row;
    route_start_column_ = column;
    rebuildPlannedPath();
    update();
}
