#include "drone_qt/position_view_widget.hpp"
#include "drone_qt/ros_manager.hpp"

#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QColor>
#include <QQueue>   
#include <cmath>

PositionViewWidget::PositionViewWidget(QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(500, 400);
    setAutoFillBackground(true);
    // 初始化一次显示坐标：内部右下角 (6,8) 对应显示坐标 (0,0)
    current_display_coord_ = toDisplayCoord(current_row_, current_col_);
    //rebuildPlannedPath();
    show_selection_rect_ = false;
}

void PositionViewWidget::setPosition(double x, double y, double z)
{
    // 原始 /drone/local_position 语义：
    // x+ = 右, y+ = 上
    // 地面站显示语义：
    // x+ = 上, y+ = 左
    x_ = y;
    y_ = -x;
    z_ = z;
    update();
}

void PositionViewWidget::drawPlannedPath(QPainter &painter) const
{
    // planned_path_ 已经是完整路径序列；这里只负责显示，不再做规划计算。
    // 箭头按“连续直行段”绘制一次，而不是每个小格之间都画一次，避免画面过密。
    if (planned_path_.size() < 2) {
        return;
    }

    QPen pen(QColor(255, 140, 0), 1);
    pen.setJoinStyle(Qt::RoundJoin);
    pen.setCapStyle(Qt::RoundCap);
    painter.setPen(pen);

    QPointF segment_start = cellCenter(planned_path_[0].row,
                                       planned_path_[0].col);
    QPointF previous_point = segment_start;
    int previous_dr = planned_path_[1].row - planned_path_[0].row;
    int previous_dc = planned_path_[1].col - planned_path_[0].col;

    for (int i = 1; i < planned_path_.size(); ++i) {
        const QPointF p1 = cellCenter(planned_path_[i - 1].row,
                                      planned_path_[i - 1].col);
        const QPointF p2 = cellCenter(planned_path_[i].row,
                                      planned_path_[i].col);
        painter.drawLine(p1, p2);

        const int current_dr = planned_path_[i].row - planned_path_[i - 1].row;
        const int current_dc = planned_path_[i].col - planned_path_[i - 1].col;

        if (current_dr != previous_dr || current_dc != previous_dc) {
            drawDirectionArrow(painter, segment_start, p1);
            segment_start = p1;
            previous_dr = current_dr;
            previous_dc = current_dc;
        }

        previous_point = p2;
    }

    drawDirectionArrow(painter, segment_start, previous_point);
}

void PositionViewWidget::drawDirectionArrow(QPainter &painter, const QPointF &start, const QPointF &end) const
{
    const qreal dx = end.x() - start.x();
    const qreal dy = end.y() - start.y();
    const qreal length = std::sqrt(dx * dx + dy * dy);

    if (length < 1.0) {
        return;
    }

    const QPointF mid((start.x() + end.x()) / 2.0, (start.y() + end.y()) / 2.0);
    const qreal ux = dx / length;
    const qreal uy = dy / length;
    const qreal arrow_length = 10.0;
    const qreal arrow_width = 5.0;

    const QPointF tip(mid.x() + ux * arrow_length / 2.0,
                      mid.y() + uy * arrow_length / 2.0);
    const QPointF tail(mid.x() - ux * arrow_length / 2.0,
                       mid.y() - uy * arrow_length / 2.0);
    const QPointF left(tail.x() - uy * arrow_width,
                       tail.y() + ux * arrow_width);
    const QPointF right(tail.x() + uy * arrow_width,
                        tail.y() - ux * arrow_width);

    painter.save();
    painter.setBrush(QColor(255, 140, 0));
    painter.setPen(Qt::NoPen);
    painter.drawPolygon(QPolygonF() << tip << left << right);
    painter.restore();
}

void PositionViewWidget::moveSelectionUp()
{
    if (current_row_ > 0) {
        --current_row_;
        current_display_coord_ = toDisplayCoord(current_row_, current_col_);
        show_selection_rect_ = true;
        update();
    }
}

void PositionViewWidget::moveSelectionDown()
{
    if (current_row_ < rows_ - 1) {
        ++current_row_;
        current_display_coord_ = toDisplayCoord(current_row_, current_col_);
        show_selection_rect_ = true;
        update();
    }
}

void PositionViewWidget::moveSelectionLeft()
{
    if (current_col_ > 0) {
        --current_col_;
        current_display_coord_ = toDisplayCoord(current_row_, current_col_);
        show_selection_rect_ = true;
        update();
    }
}

void PositionViewWidget::moveSelectionRight()
{
    if (current_col_ < cols_ - 1) {
        ++current_col_;
        current_display_coord_ = toDisplayCoord(current_row_, current_col_);
        show_selection_rect_ = true;
        update();
    }
}

void PositionViewWidget::confirmCurrentCell()
{
    //将当前选中的格子索引添加到已确认格的集合中，并调用 `update()` 触发重绘
    blocked_cells_.insert(cellIndex(current_row_, current_col_));
    update();
}

void PositionViewWidget::refreshPlannedPath()
{
    // “刷新”不是单纯重画，而是一次完整的状态更新：
    // 1. 把当前选择格确认成灰色禁行格
    // 2. 当前实现仍保留 route_start_* 变量，但固定起点语义建议理解为右下角起始扫描
    // 3. 如有必要，把绿色选择框挪到最近白格
    // 4. 重建整条背景路线
    // 5. 隐藏绿色选择框并触发重绘
    blocked_cells_.insert(cellIndex(current_row_, current_col_));
    route_start_row_ = current_row_;
    route_start_col_ = current_col_;
    //moveSelectionToNearestFreeCell();
    current_display_coord_ = toDisplayCoord(current_row_, current_col_);
    rebuildPlannedPath();
    show_selection_rect_ = false;
    push_flag_ = false;
    update();
}

void PositionViewWidget::delPlannedPath()
{
    // 删除当前绿色选择框所在格子的禁行状态
    blocked_cells_.remove(cellIndex(current_row_, current_col_));
    moveSelectionToNearestFreeCell();
    current_display_coord_ = toDisplayCoord(current_row_, current_col_);
    rebuildPlannedPath();
    show_selection_rect_ = false;
    push_flag_ = false;
    update();
}

QRectF PositionViewWidget::gridRect() const
{
    const qreal side_x = 450.0;
    const qreal side_y = 350.0;
    const qreal left = (width() - side_x) / 2.0;
    const qreal top = (height() - side_y) / 2.0;
    return QRectF(left, top, side_x, side_y);
}

QRectF PositionViewWidget::cellRect(int row, int col) const
{
    //根据行列索引计算对应格子的绘制区域，首先获取网格的矩形区域，然后根据行列数计算每个格子的宽度和高度，最后返回对应格子的矩形区域
    const QRectF grid = gridRect();
    const qreal cell_width = grid.width() / cols_;
    const qreal cell_height = grid.height() / rows_;

    return QRectF(
        grid.left() + col * cell_width,
        grid.top() + row * cell_height,
        cell_width,
        cell_height);
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
int PositionViewWidget::cellIndex(int row, int col) const
{
    return row * cols_ + col;
}

bool PositionViewWidget::isInside(int row, int col) const
{
    return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool PositionViewWidget::isBlocked(int row, int col) const
{
    return blocked_cells_.contains(cellIndex(row, col));
}

QPointF PositionViewWidget::cellCenter(int row, int col) const
{
    return cellRect(row, col).center();
}

PositionViewWidget::DisplayCoord PositionViewWidget::toDisplayCoord(int row, int col) const
{
    // 内部索引仍保持左上角为 (0,0)，这里只在显示层翻转坐标：
    // 右下角显示为 (0,0)，左上角显示为 (rows_-1, cols_-1)。
    return DisplayCoord{rows_ - 1 - row, cols_ - 1 - col};
}

int PositionViewWidget::findTargetIndex(const QVector<GridCell> &cells,
                                        const GridCell &target) const
{
    for (int i = 0; i < cells.size(); ++i) {
        if (cells[i] == target) {
            return i;
        }
    }
    return -1;
}

QSet<int> PositionViewWidget::computeReachableCells(const GridCell &start) const
{
    QSet<int> visited;

    if (!isInside(start.row, start.col) || isBlocked(start.row, start.col)) {
        return visited;
    }

    QQueue<GridCell> queue;
    queue.enqueue(start);
    visited.insert(cellIndex(start.row, start.col));

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    while (!queue.isEmpty()) {
        const GridCell cur = queue.dequeue();

        for (int k = 0; k < 4; ++k) {
            const int nr = cur.row + dr[k];
            const int nc = cur.col + dc[k];

            if (!isInside(nr, nc)) {
                continue;
            }
            if (isBlocked(nr, nc)) {
                continue;
            }

            const int next_index = cellIndex(nr, nc);
            if (visited.contains(next_index)) {
                continue;
            }

            visited.insert(next_index);
            queue.enqueue({nr, nc});
        }
    }

    return visited;
}

bool PositionViewWidget::moveSelectionToNearestFreeCell()
{
    // 这里只修正当前绿色选择框位置，不修改 route_start_*。
    // 这样可以保证：路线起点和用户当前选择位置是两套独立状态。
    if (!isBlocked(current_row_, current_col_)) {
        return true;
    }

    QQueue<GridCell> queue;
    QSet<int> visited;
    const GridCell start{current_row_, current_col_};
    queue.enqueue(start);
    visited.insert(cellIndex(start.row, start.col));

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    while (!queue.isEmpty()) {
        const GridCell cur = queue.dequeue();

        for (int k = 0; k < 4; ++k) {
            const int nr = cur.row + dr[k];
            const int nc = cur.col + dc[k];

            if (!isInside(nr, nc)) {
                continue;
            }

            const int index = cellIndex(nr, nc);
            if (visited.contains(index)) {
                continue;
            }

            if (!isBlocked(nr, nc)) {
                current_row_ = nr;
                current_col_ = nc;
                current_display_coord_ = toDisplayCoord(current_row_, current_col_);
                return true;
            }

            visited.insert(index);
            queue.enqueue({nr, nc});
        }
    }

    return false;
}

QVector<PositionViewWidget::GridCell>
PositionViewWidget::buildSnakeTargets(const QSet<int> &reachable) const
{
    // 当前扫线策略：自下而上；底行从右到左；上一行从左到右；之后交替往复。
    // 这里输出的是“希望访问的目标格顺序”，不是最终补桥后的完整路线。
    QVector<GridCell> targets;

    for (int row = rows_ - 1; row >= 0; --row) {
        const int offset_from_bottom = (rows_ - 1) - row;

        if (offset_from_bottom % 2 == 0) {
            for (int col = cols_ - 1; col >= 0; --col) {
                const int index = cellIndex(row, col);
                if (reachable.contains(index)) {
                    targets.push_back({row, col});
                }
            }
        } else {
            for (int col = 0; col < cols_; ++col) {
                const int index = cellIndex(row, col);
                if (reachable.contains(index)) {
                    targets.push_back({row, col});
                }
            }
        }
    }

    return targets;
}

QVector<PositionViewWidget::GridCell>
PositionViewWidget::findPathBfs(const GridCell &start, const GridCell &goal) const
{
    QVector<GridCell> empty_result;

    if (!isInside(start.row, start.col) || !isInside(goal.row, goal.col)) {
        return empty_result;
    }
    if (isBlocked(start.row, start.col) || isBlocked(goal.row, goal.col)) {
        return empty_result;
    }

    const int total = rows_ * cols_;
    QVector<bool> visited(total, false);
    QVector<int> parent(total, -1);

    QQueue<GridCell> queue;
    queue.enqueue(start);
    visited[cellIndex(start.row, start.col)] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    while (!queue.isEmpty()) {
        const GridCell cur = queue.dequeue();

        if (cur == goal) {
            break;
        }

        for (int k = 0; k < 4; ++k) {
            const int nr = cur.row + dr[k];
            const int nc = cur.col + dc[k];

            if (!isInside(nr, nc)) {
                continue;
            }
            if (isBlocked(nr, nc)) {
                continue;
            }

            const int next_index = cellIndex(nr, nc);
            if (visited[next_index]) {
                continue;
            }

            visited[next_index] = true;
            parent[next_index] = cellIndex(cur.row, cur.col);
            queue.enqueue({nr, nc});
        }
    }

    const int goal_index = cellIndex(goal.row, goal.col);
    if (!visited[goal_index]) {
        return empty_result;
    }

    QVector<GridCell> reversed_path;
    int current = goal_index;

    while (current != -1) {
        const int row = current / cols_;
        const int col = current % cols_;
        reversed_path.push_back({row, col});
        current = parent[current];
    }

    QVector<GridCell> path;
    for (int i = reversed_path.size() - 1; i >= 0; --i) {
        path.push_back(reversed_path[i]);
    }

    return path;
}

void PositionViewWidget::rebuildDisplayPath()
{
    display_path_.clear();
    display_path_.reserve(planned_path_.size());

    for (const auto &cell : planned_path_) {
        display_path_.push_back(toDisplayCoord(cell.row, cell.col));
    }
}

void PositionViewWidget::rebuildPlannedPath()
{
    // 路径重建分为 5 个阶段：
    // 1. 选择合法起点（优先 route_start_*，非法时回退到右下角）
    // 2. 求起点所在连通域的可达白格
    // 3. 生成蛇形目标序列
    // 4. 用 BFS 把目标格逐段拼接成完整路径
    // 5. 从终点补回起点，形成闭环
    planned_path_.clear();

    GridCell start{route_start_row_, route_start_col_};

    if (!isInside(start.row, start.col) || isBlocked(start.row, start.col)) {
        start = {rows_ - 1, cols_ - 1};
    }

    if (!isInside(start.row, start.col) || isBlocked(start.row, start.col)) {
        return;
    }

    const QSet<int> reachable = computeReachableCells(start);
    if (reachable.isEmpty()) {
        return;
    }

    QVector<GridCell> targets = buildSnakeTargets(reachable);
    if (targets.isEmpty()) {
        planned_path_.push_back(start);
        return;
    }

    int start_index = findTargetIndex(targets, start);
    if (start_index < 0) {
        targets.prepend(start);
        start_index = 0;
    }

    QVector<GridCell> ordered_targets;
    for (int i = start_index; i < targets.size(); ++i) {
        ordered_targets.push_back(targets[i]);
    }
    for (int i = 0; i < start_index; ++i) {
        ordered_targets.push_back(targets[i]);
    }

    GridCell current = start;
    planned_path_.push_back(current);

    for (const GridCell &target : ordered_targets) {
        if (target == current) {
            continue;
        }

        const QVector<GridCell> segment = findPathBfs(current, target);
        if (segment.isEmpty()) {
            continue;
        }

        // 跳过 segment[0]，避免与 planned_path_ 尾部重复
        for (int i = 1; i < segment.size(); ++i) {
            planned_path_.push_back(segment[i]);
        }

        current = target;
    }

    if (!(current == start)) {
        const QVector<GridCell> return_segment = findPathBfs(current, start);
        if (!return_segment.isEmpty()) {
            for (int i = 1; i < return_segment.size(); ++i) {
                planned_path_.push_back(return_segment[i]);
            }
        }
    }

    rebuildDisplayPath();
}


/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */


bool PositionViewWidget::pointToCell(const QPointF &pos, int &row, int &col) const
{
    const QRectF grid = gridRect();
    // 首先判断点击位置是否在网格区域内，如果不在则返回 false。
    if (!grid.contains(pos)) {
        return false;
    }

    const qreal cell_width = grid.width() / cols_;
    const qreal cell_height = grid.height() / rows_;

    // 根据点击位置计算对应的格子索引，注意坐标系翻转：右下角是 (0,0)，左上角是 (rows_-1, cols_-1)。
    col = static_cast<int>((pos.x() - grid.left()) / cell_width);
    row = static_cast<int>((pos.y() - grid.top()) / cell_height);

    if (col >= cols_) {
        col = cols_ - 1;
    }
    if (row >= rows_) {
        row = rows_ - 1;
    }

    //判断是否为有效格子（在网格范围内且非禁行格），并返回结果
    return isInside(row, col);
}


void PositionViewWidget::toggleBlockedCell(int row, int col)
{
    //把二维格子坐标转成唯一整数
    const int index = cellIndex(row, col);

    //判断是否为灰格决定添加还是移除
    if (blocked_cells_.contains(index)) {
        blocked_cells_.remove(index);
    } else {
        blocked_cells_.insert(index);
    }
}

/* ------------------------------------------------------------------ */

// 提供一个接口，让外部可以获取预规划路线的坐标点列表（显示坐标系）
QVector<QPoint> PositionViewWidget::plannedPathPoints() const
{
    QVector<QPoint> result;
    result.reserve(display_path_.size());

    for (const auto &cell : display_path_) {
        result.push_back(QPoint(cell.row, cell.col));
    }

    return result;
}

QVector<WorldCoord> PositionViewWidget::plannedWorldPoints() const
{
    QVector<WorldCoord> result;
    result.reserve(display_path_.size());

    const double cell_size_x = 0.5;   // 每个格子在 x 方向对应 0.5m
    const double cell_size_y = 0.5;   // 每个格子在 y 方向对应 0.5m
    const double origin_x = 0.0;      // 栅格原点在真实坐标中的 x
    const double origin_y = 0.0;      // 栅格原点在真实坐标中的 y

    for (const auto &cell : display_path_) {
        WorldCoord point;
        point.x = origin_x + cell.row * cell_size_x;
        point.y = origin_y + cell.col * cell_size_y;
        result.push_back(point);
    }

    return result;
}

void PositionViewWidget::display_select()
{
    push_flag_ = !push_flag_;
    update();
}

void PositionViewWidget::pushFlagresult(bool value)
{
    push_flag_ = value;
    show_selection_rect_ = false;
    update();
}

//update()后进行重画
void PositionViewWidget::paintEvent(QPaintEvent *event)
{
    // 绘制顺序不能随意改：灰格 -> 路线 -> 绿色框 -> 网格线 -> 坐标轴/红点/文字。
    // 这里同时存在两套显示语义：网格路线是栅格坐标系，红点是实时世界坐标映射结果。
    //调用基类的paintEvent函数，确保控件的基本绘制行为得到执行
    QWidget::paintEvent(event);

    //创建一个QPainter对象，在控件上进行绘制
    QPainter painter(this);
    //启用抗锯齿渲染，并填充背景为白色
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), Qt::white);

    // 1) 先计算网格区域
    const QRectF grid = gridRect();

     // 2) 绘制已确认格
    for (int index : blocked_cells_) {
        const int row = index / cols_;
        const int col = index % cols_;
        painter.fillRect(cellRect(row, col), QColor(128, 128, 128, 225));
    }

    if(!push_flag_){
        drawPlannedPath(painter);
    }

    // 3) 绘制当前焦点格
    const QRectF current_rect = cellRect(current_row_, current_col_);
    if (show_selection_rect_) {
        painter.fillRect(current_rect, QColor(0, 255, 0, 0));
        painter.setPen(QPen(Qt::green, 2));
        painter.drawRect(current_rect.adjusted(1, 1, -1, -1));
    }

    // 4) 绘制网格线
    painter.setPen(QPen(Qt::lightGray, 1));
    for (int row = 0; row <= rows_; ++row) {
        const qreal y = grid.top() + row * grid.height() / rows_;
        painter.drawLine(QPointF(grid.left(), y), QPointF(grid.right(), y));
    }
    for (int col = 0; col <= cols_; ++col) {
        const qreal x = grid.left() + col * grid.width() / cols_;
        painter.drawLine(QPointF(x, grid.top()), QPointF(x, grid.bottom()));
    }

    //计算控件中心点的坐标，并以此为基准绘制坐标轴和无人机位置
    // const int center_x = (width() / 2) + 200; //将中心点向右偏移200像素，使无人机位置更靠近右侧
    // const int center_y = (height() / 2) + 150;//将中心点向下偏移150像素，使无人机位置更靠近下侧
    // const QPoint center(center_x, center_y);
    const QPointF origin = cellCenter(rows_ - 1, cols_ - 1);// 右下角格子中心作为坐标原点


    //绘制坐标轴，使用蓝色的细线表示水平和垂直的坐标轴
    // painter.setPen(QPen(Qt::black, 4));
    // painter.drawLine(25, (center_y + 25), 475, (center_y + 20));
    // painter.drawLine((center_x + 25), 25, (center_x + 25), 375);

    //把真实坐标转换成像素坐标
    const int px = static_cast<int>((origin.x() - y_ * scale_));
    const int py = static_cast<int>((origin.y() - x_ * scale_));

    ////绘制无人机起点
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    painter.drawEllipse(origin, 25, 25);

    //绘制无人机位置，使用红色的实心圆表示无人机在二维平面上的位置
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::black);
    painter.drawEllipse(QPoint(px, py), 3, 3);

    //在控件上绘制文本，显示当前的坐标值，使用黑色的文本表示无人机的x、y、z坐标，保留两位小数
    painter.setPen(Qt::black);
    painter.drawText(
        10,
        20,
        QString("x=%1, y=%2, z=%3")
            .arg(x_, 0, 'f', 2)
            .arg(y_, 0, 'f', 2)
            .arg(z_, 0, 'f', 2));

    // 显示当前绿色选择框对应的逻辑坐标：右下角是 (0,0)，左上角是 (6,8)
    // painter.drawText(
    //     150,
    //     20,
    //     QString("grid=(%1,%2)")
    //         .arg(current_display_coord_.row)
    //         .arg(current_display_coord_.col));

    for (int row = 0; row < rows_; ++row) {
        QRectF rect(
            grid.left() - 30,
            grid.top() + row * (grid.height() / rows_),
            35,
            grid.height() / rows_);

        painter.drawText(rect, Qt::AlignCenter, QString("B%1").arg((rows_ + 1) - (row + 1)));
    }

    for (int col = 0; col < cols_; ++col) {
        QRectF rect(
            grid.left() + col * (grid.width() / cols_),
            grid.bottom() + 2,
            grid.width() / cols_,
            20);

        painter.drawText(rect, Qt::AlignCenter, QString("A%1").arg(col + 1));
    }
}

void PositionViewWidget::mousePressEvent(QMouseEvent *event)
{
    int row = -1;
    int col = -1;

    //判断鼠标点击的位置是否合法
    if (!pointToCell(event->localPos(), row, col)) {
        QWidget::mousePressEvent(event);
        return;
    }

    current_row_ = row;
    current_col_ = col;
    current_display_coord_ = toDisplayCoord(current_row_, current_col_);

    // 点击后直接切换格子状态（灰色<->白色），并刷新路线和显示状态
    show_selection_rect_ = false;
    toggleBlockedCell(row, col);

    route_start_row_ = current_row_;
    route_start_col_ = current_col_;

    
    rebuildPlannedPath();
    push_flag_ = false;

    update();
}
