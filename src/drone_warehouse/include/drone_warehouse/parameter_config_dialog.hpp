#pragma once

#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>

class QButtonGroup;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QSlider;
class QStackedWidget;
class QWidget;


// 参数页面第二列的筛选类别。
enum class ParameterGroup
{
    All,
    Shelf,
    Flight,
    Servo,
    Camera
};

class ParameterConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ParameterConfigDialog(
        const WarehouseConfig &config,
        QWidget *parent = nullptr);

    const WarehouseConfig &savedConfig() const;

signals:
    // “启用”保存成功后立即通知主窗口，不需要关闭参数页面。
    void configApplied(const WarehouseConfig &config);

private:
    // 页面构建与切换。
    void buildUi();
    void buildProjectPage();
    void buildParameterPage();
    void buildParameterFilterPanel();
    void buildNumericKeypad(QWidget *parent);
    void setNumericKeypadEnabled(bool enabled);
    void appendNumericDigit(const QString &digit);
    void backspaceNumericInput();
    void selectMainPage(int page_index);
    void selectProject(InspectionProject project);
    void selectParameterGroup(ParameterGroup group);

    // 参数列表和右侧编辑器。
    void rebuildParameterList(bool preserve_scroll_position = false);
    void addParameterRow(
        const QString &parameter_id,
        const QString &display_name,
        const QString &display_value,
        bool differs_from_default);
    void showParameterEditor(QListWidgetItem *item);
    void commitParameterEdit();
    bool updateParameterFromEditor(QString *error_message);
    void handleApply();

    // 打开页面时保留原配置，所有编辑先写入 working_config_。
    WarehouseConfig original_config_;
    WarehouseConfig working_config_;
    InspectionProject selected_project_ = InspectionProject::Cargo;
    ParameterGroup selected_parameter_group_ = ParameterGroup::All;

    // 左侧页面导航和顶部操作按钮。
    QPushButton *project_page_button_ = nullptr;
    QPushButton *parameter_page_button_ = nullptr;
    QPushButton *close_page_button_ = nullptr;
    QButtonGroup *main_page_button_group_ = nullptr;
    QWidget *parameter_filter_panel_ = nullptr;
    QButtonGroup *parameter_filter_button_group_ = nullptr;
    QWidget *numeric_keypad_ = nullptr;
    bool numeric_editor_active_ = false;
    QPushButton *restore_button_ = nullptr;
    QPushButton *discard_button_ = nullptr;
    QPushButton *apply_button_ = nullptr;

    // 中间页面区域。
    QStackedWidget *page_stack_ = nullptr;
    QWidget *project_page_ = nullptr;
    QWidget *parameter_page_ = nullptr;
    QButtonGroup *project_button_group_ = nullptr;
    QListWidget *parameter_list_ = nullptr;

    // 右侧单参数编辑器。
    QWidget *parameter_editor_ = nullptr;
    QLabel *editor_title_label_ = nullptr;
    QLabel *editor_description_label_ = nullptr;
    QLabel *editor_default_label_ = nullptr;
    QStackedWidget *editor_input_stack_ = nullptr;
    QWidget *editor_line_container_ = nullptr;
    QLineEdit *editor_line_edit_ = nullptr;
    QSlider *editor_camera_slider_ = nullptr;
    QComboBox *editor_combo_box_ = nullptr;
    QPushButton *editor_confirm_button_ = nullptr;
    QString editing_parameter_id_;
    bool camera_slider_active_ = false;
};