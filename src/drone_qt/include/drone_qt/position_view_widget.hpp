#pragma once

#include <QPointF>
#include <QRectF>
#include <QSet>
#include <QWidget>
#include <QVector>
#include <QMetaType>

class QPaintEvent;
class QPainter;
class QMouseEvent;

struct WorldCoord
{
    double x;
    double y;
};

Q_DECLARE_METATYPE(WorldCoord)
Q_DECLARE_METATYPE(QVector<WorldCoord>)

class PositionViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PositionViewWidget(QWidget *parent = nullptr);
    ~PositionViewWidget() override = default;

    //地面端收到位置数据后，调用它刷新坐标
    void setPosition(double x, double y, double z);

    //提供一个接口，让外部可以获取预规划路线的坐标点列表（显示坐标系）
    QVector<QPoint> plannedPathPoints() const;

    //导出反转后的栅格坐标(真实坐标)
    QVector<WorldCoord> plannedWorldPoints() const;

    //对当前行的操作（上下左右）
    void moveSelectionUp();
    void moveSelectionDown();
    void moveSelectionLeft();
    void moveSelectionRight();

    //确认当前选中的单元格
    void confirmCurrentCell();
    //刷新预规划路线显示
    void refreshPlannedPath();
    void delPlannedPath();
    //判断是否上传路线
    void pushFlagresult(bool value);
    //修改路线显示模式
    void display_select();

protected:
    //重写paintEvent函数，在控件上绘制无人机位置的可视化表示
    void paintEvent(QPaintEvent *event) override;

    //重写mousePressEvent函数，处理用户点击事件，更新当前选中的格子，并触发重绘
    void mousePressEvent(QMouseEvent *event) override;

private:
    //预规划路线经过的格子序列（内部索引坐标：左上角是 0,0）
    struct GridCell
    {
        int row;
        int col;

        bool operator==(const GridCell &other) const
        {
            return row == other.row && col == other.col;
        }
    };

    //显示给用户看的逻辑坐标（反转后：右下角是 0,0）
    struct DisplayCoord
    {
        int row;
        int col;
    };

    // 计算网格所在的正方形区域
    QRectF gridRect() const;

    // 计算某一个格子的绘制区域
    QRectF cellRect(int row, int col) const;

    // 将 (row, col) 映射成唯一整数
    int cellIndex(int row, int col) const;

    /* -------绘制预规划路线相关函数------- */
    /*
        1. 清空旧路线。
        2. 检查当前格是否可通行。
        3. 求当前连通区域。
        4. 生成蛇形目标序列。
        5. 依次用 BFS 把目标格串起来。
        6. 保存到 `planned_path_`。
    */

    //判断某格是否为障碍格
    bool isInside(int row, int col) const;
    //判断某格是否为禁行格
    bool isBlocked(int row, int col) const;
    //计算某格中心点的坐标
    QPointF cellCenter(int row, int col) const;
    //把内部网格索引转换成显示给用户看的反转坐标
    DisplayCoord toDisplayCoord(int row, int col) const;
    //从一个格子到另一个格子，按上下左右四连通搜索最短路径
    int findTargetIndex(const QVector<GridCell> &cells, const GridCell &target) const;
    //按蛇形顺序收集所有可达白格
    QVector<GridCell> buildSnakeTargets(const QSet<int> &reachable) const;
    //使用BFS算法寻找从起点到目标的路径，返回经过的格子序列
    QVector<GridCell> findPathBfs(const GridCell &start, const GridCell &goal) const;
    //从当前格子出发做一次 BFS，找出所有可达白格
    QSet<int> computeReachableCells(const GridCell &start) const;
    //寻找距离当前格最近的可通行白格
    bool moveSelectionToNearestFreeCell();
    //根据预规划路线和已确认格子，重建无人机的预规划路径
    void rebuildPlannedPath();
    void rebuildDisplayPath();

    //绘制预规划路线的辅助函数
    void drawPlannedPath(QPainter &painter) const;
    void drawDirectionArrow(QPainter &painter, const QPointF &start, const QPointF &end) const;
    /* ----------------------------------- */

    //根据点击位置计算对应的格子索引，并切换当前选中格子
    bool pointToCell(const QPointF &pos, int &row, int &col) const;
    //负责切换网格状态（空白<->障碍），并触发重绘
    void toggleBlockedCell(int row, int col);

private:
    // 无人机位置的坐标和缩放比例
    double x_{0.0};
    double y_{0.0};
    double z_{0.0};
    double scale_{100.0};

    //网格的行数和列数，以及当前选中的行列索引
    int rows_{7};
    int cols_{9};
    int current_row_{6};
    int current_col_{8};
    // route_start_* 与 current_* 分离保存：前者决定背景路线从哪里开始，后者只表示当前绿色选择框位置
    int route_start_row_{6};               // 预规划路线起点行
    int route_start_col_{8};               // 预规划路线起点列
    QSet<int> blocked_cells_;              // 灰色禁行格
    QVector<GridCell> planned_path_;       // 预规划扫格路线，保存的是最终完整闭合路径而不是目标点列表
    QVector<DisplayCoord> display_path_;   //反转后的路径
    DisplayCoord current_display_coord_{0, 0}; // 当前绿色选择框对应的显示坐标（右下角为 0,0）
    bool show_selection_rect_{true};       // 是否显示绿色选择框
    bool display_selection{true};          // 是否显示路线
    bool push_flag_{false};                //是否上传路线
};
