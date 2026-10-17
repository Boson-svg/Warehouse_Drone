#include "drone_warehouse/animal_inspection_page.hpp"

#include "drone_warehouse/animal_grid_view.hpp"

#include <algorithm>

#include <QAbstractItemView>
#include <QLabel>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QStringList>
#include <QVBoxLayout>

namespace
{
constexpr int kOuterMargin = 20;
constexpr int kPanelGap = 16;
constexpr int kContentTop = 84;
}

AnimalInspectionPage::AnimalInspectionPage(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
}

void AnimalInspectionPage::setupUi()
{
    // Animal 使用固定 7x9 二维栅格画板，点击禁行格和路线绘制仍由 AnimalGridView 负责。
    grid_view_ = new AnimalGridView(this);

    // 右上角只显示视觉伺服完成后的文字记录，不再接收或展示图片本体。
    result_panel_ = new QWidget(this);
    result_panel_->setObjectName("animalResultPanel");
    auto *result_layout = new QVBoxLayout(result_panel_);
    result_layout->setContentsMargins(12, 10, 12, 10);
    result_layout->setSpacing(8);

    auto *result_title = new QLabel("动物识别记录", result_panel_);
    result_title->setObjectName("animalResultTitle");
    result_list_ = new QListWidget(result_panel_);
    result_list_->setSelectionMode(QAbstractItemView::NoSelection);
    result_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    result_list_->setWordWrap(true);
    result_layout->addWidget(result_title);
    result_layout->addWidget(result_list_, 1);

    // Animal 只保留一个运行日志，货物巡检的航点和 AI 日志不会遮挡栅格。
    run_log_panel_ = new QWidget(this);
    run_log_panel_->setObjectName("animalRunLogPanel");
    auto *run_log_layout = new QVBoxLayout(run_log_panel_);
    run_log_panel_->setContentsMargins(5, 5, 5, 5);
    run_log_view_ = new QPlainTextEdit(run_log_panel_);
    run_log_view_->setReadOnly(true);
    run_log_view_->setMaximumBlockCount(1000);
    run_log_layout->addWidget(run_log_view_);

    // Keep the Animal result area visually consistent with the existing quiet
    // status and log panels. Records are text-only and never open an image.
    // 保持 Animal 结果区与现有状态、日志面板的安静深色风格一致。
    result_panel_->setStyleSheet(
        "#animalResultPanel { background: rgba(18, 24, 34, 185);"
        " border: 1px solid rgba(90, 130, 180, 110); border-radius: 8px; }"
        "#animalResultTitle { background: transparent; border: none;"
        " color: #8fe7ff; font-size: 18px; font-weight: 600; }"
        "QListWidget { background: transparent; border: none; color: #d7e3f4;"
        " font-size: 16px; outline: none; }"
        "QListWidget::item { border-bottom: 1px solid rgba(90, 130, 180, 90);"
        " padding: 8px 2px; }");
    run_log_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 0); border: none;"
        "border-radius: 10px; padding: 6px 10px;");

    run_log_view_->appendPlainText("日志初始化成功");
    updatePageGeometry();
}

void AnimalInspectionPage::setPosition(double x, double y, double z)
{
    grid_view_->setPosition(x, y, z);
}

QVector<WorldCoord> AnimalInspectionPage::plannedWorldPoints(
    double altitude, double yaw) const
{
    return grid_view_->plannedWorldPoints(altitude, yaw);
}

void AnimalInspectionPage::appendRunLog(const QString &text)
{
    run_log_view_->appendPlainText(text);
}

void AnimalInspectionPage::clearRunLog()
{
    run_log_view_->clear();
}

void AnimalInspectionPage::appendRecognitionRecord(
    const QString &target_id,
    const QString &detail,
    const QString &time_text)
{
    // 旧 drone_qt 记录还包含图片数据；这里按当前需求只保留三个文字字段，
    // 可选字段为空时不添加对应行。
    QStringList lines;
    lines << QString("目标：%1").arg(target_id);
    if (!detail.trimmed().isEmpty())
    {
        lines << QString("结果：%1").arg(detail.trimmed());
    }
    if (!time_text.trimmed().isEmpty())
    {
        lines << QString("时间：%1").arg(time_text.trimmed());
    }

    auto *item = new QListWidgetItem(lines.join('\n'));
    item->setSizeHint(QSize(0, 30 * lines.size() + 12));
    result_list_->insertItem(0, item);
    while (result_list_->count() > 100)
    {
        delete result_list_->takeItem(result_list_->count() - 1);
    }
}

QRect AnimalInspectionPage::sidebarGeometry() const
{
    const int max_sidebar_width =
        std::max(0, width() - 2 * kOuterMargin - kPanelGap);
    const int sidebar_width = std::min(
        std::clamp(width() / 4, 280, 360), max_sidebar_width);
    const int sidebar_x = std::max(0, width() - kOuterMargin - sidebar_width);
    return QRect(sidebar_x, kContentTop, sidebar_width,
                 std::max(0, height() - kContentTop - kOuterMargin));
}

QRect AnimalInspectionPage::attitudePanelGeometry() const
{
    const QRect sidebar = sidebarGeometry();
    const int status_height = std::min(150, sidebar.height() / 3);
    return QRect(sidebar.x(), sidebar.bottom() - status_height + 1,
                 sidebar.width(), status_height);
}

void AnimalInspectionPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updatePageGeometry();
}

void AnimalInspectionPage::updatePageGeometry()
{
    const QRect sidebar = sidebarGeometry();
    const QRect status = attitudePanelGeometry();
    const int map_width = std::max(0, sidebar.x() - kPanelGap);
    const int upper_height =
        std::max(0, status.y() - kContentTop - 2 * kPanelGap);
    const int result_height = upper_height / 2;
    const int log_height = upper_height - result_height;

    // Animal 使用左侧地图、右侧信息栏。右栏顶部暂时留给动物识别数量。
    // Animal 使用左侧地图、右侧信息栏；姿态状态由 MainWindow 放在右下角。
    grid_view_->setGeometry(0, 0, map_width, height());
    result_panel_->setGeometry(
        sidebar.x(), kContentTop, sidebar.width(), result_height);
    run_log_panel_->setGeometry(
        sidebar.x(), kContentTop + result_height + kPanelGap,
        sidebar.width(), log_height);

    grid_view_->lower();
    result_panel_->raise();
    run_log_panel_->raise();
}
