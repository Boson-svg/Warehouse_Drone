#pragma once

#include "drone_warehouse/models.hpp"

#include <QPointF>
#include <QRectF>
#include <QSet>
#include <QVector>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QPainter;

// 动物巡检专用固定二维画板。
// 内部数组以左上角为 (0,0)，对外任务坐标以右下角为 (0,0)。
class AnimalGridView : public QWidget
{
public:
    explicit AnimalGridView(QWidget *parent = nullptr);
    ~AnimalGridView() override = default;

    // 更新无人机实时位置，仅影响画板上的位置标记。
    void setPosition(double x, double y, double z);

    // 将当前显示路线转换成现有任务上传接口使用的世界坐标。
    QVector<WorldCoord> plannedWorldPoints(double altitude, double yaw) const;

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    static constexpr int kRows = 7;
    static constexpr int kColumns = 9;
    static constexpr double kCellSizeM = 0.5;

    struct GridCell
    {
        int row = 0;
        int column = 0;

        bool operator==(const GridCell &other) const
        {
            return row == other.row && column == other.column;
        }
    };

    struct DisplayCoord
    {
        int row = 0;
        int column = 0;
    };

    QRectF gridRect() const;
    QRectF cellRect(int row, int column) const;
    QPointF cellCenter(int row, int column) const;
    int cellIndex(int row, int column) const;
    bool isInside(int row, int column) const;
    bool isBlocked(int row, int column) const;
    bool pointToCell(const QPointF &point, int &row, int &column) const;
    DisplayCoord toDisplayCoord(int row, int column) const;

    QSet<int> computeReachableCells(const GridCell &start) const;
    QVector<GridCell> buildSnakeTargets(const QSet<int> &reachable) const;
    QVector<GridCell> findPathBfs(
        const GridCell &start,
        const GridCell &goal) const;
    int findTargetIndex(
        const QVector<GridCell> &cells,
        const GridCell &target) const;
    void rebuildPlannedPath();

    void drawPlannedPath(QPainter &painter) const;
    void drawDirectionArrow(
        QPainter &painter,
        const QPointF &start,
        const QPointF &end) const;

    double display_x_ = 0.0;
    double display_y_ = 0.0;
    double altitude_ = 0.0;

    int route_start_row_ = kRows - 1;
    int route_start_column_ = kColumns - 1;
    QSet<int> blocked_cells_;
    QVector<GridCell> planned_path_;
};
