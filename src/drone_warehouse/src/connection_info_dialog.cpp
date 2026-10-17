#include "drone_warehouse/connection_info_dialog.hpp"

#include <QButtonGroup>
#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace
{
QString modeDisplayName(ConnectionMode mode)
{
    return mode == ConnectionMode::Telemetry ? "数传" : "WiFi";
}
}

ConnectionInfoDialog::ConnectionInfoDialog(
    const WarehouseConfig &config,
    QWidget *parent)
    : QDialog(parent),
      working_config_(config),
      pending_mode_(config.connection.mode)
{
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);//去掉系统自带的白色标题栏，只保留自定义弹窗内容
    setObjectName("connectionInfoDialog");
    setWindowTitle("连接方式");
    setModal(true);
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);

    buildUi();
    loadConfigIntoControls();
    setPendingMode(pending_mode_);
}

const WarehouseConfig &ConnectionInfoDialog::savedConfig() const
{
    return working_config_;
}

void ConnectionInfoDialog::buildUi()
{
    auto *root_layout = new QHBoxLayout(this);
    root_layout->setContentsMargins(5, 5, 5, 5);
    root_layout->setSpacing(16);

    auto *left_panel = new QWidget(this);
    auto *left_layout = new QVBoxLayout(left_panel);
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->setSpacing(10);

    connection_info_label_ = new QLabel(
        QString("当前连接方式：%1")
            .arg(modeDisplayName(working_config_.connection.mode)),
        left_panel);
    connection_info_label_->setAlignment(Qt::AlignCenter);

    wifi_button_ = new QPushButton("WiFi", left_panel);
    telemetry_button_ = new QPushButton("数传", left_panel);
    save_button_ = new QPushButton("保存", left_panel);
    close_button_ = new QPushButton("关闭", left_panel);

    wifi_button_->setCheckable(true);
    telemetry_button_->setCheckable(true);
    wifi_button_->setMinimumHeight(38);
    telemetry_button_->setMinimumHeight(38);
    save_button_->setMinimumHeight(38);
    close_button_->setMinimumHeight(38);

    // 按钮组保证 WiFi 和数传始终只有一个处于高亮状态。
    mode_button_group_ = new QButtonGroup(this);
    mode_button_group_->setExclusive(true);
    mode_button_group_->addButton(wifi_button_);
    mode_button_group_->addButton(telemetry_button_);

    left_layout->addWidget(connection_info_label_);
    left_layout->addWidget(wifi_button_);
    left_layout->addWidget(telemetry_button_);
    left_layout->addStretch();
    left_layout->addWidget(save_button_);
    left_layout->addWidget(close_button_);

    serial_panel_ = new QFrame(this);
    serial_panel_->setObjectName("serialPanel");
    auto *serial_layout = new QFormLayout(serial_panel_);
    serial_layout->setContentsMargins(14, 14, 14, 14);
    serial_layout->setHorizontalSpacing(12);
    serial_layout->setVerticalSpacing(10);

    auto *serial_title = new QLabel("数传串口参数", serial_panel_);
    serial_title->setAlignment(Qt::AlignCenter);
    port_name_edit_ = new QLineEdit(serial_panel_);
    port_name_edit_->setMinimumWidth(360);
    port_name_edit_->setPlaceholderText("/dev/serial/by-id/...");

    baud_rate_combo_ = new QComboBox(serial_panel_);
    data_bits_combo_ = new QComboBox(serial_panel_);
    parity_combo_ = new QComboBox(serial_panel_);
    stop_bits_combo_ = new QComboBox(serial_panel_);
    flow_control_combo_ = new QComboBox(serial_panel_);

    const QList<int> baud_rates = {
        9600, 19200, 38400, 57600, 115200
    };
    for (int baud_rate : baud_rates)
    {
        baud_rate_combo_->addItem(
            QString::number(baud_rate), baud_rate);
    }

    data_bits_combo_->addItem(
        "5", static_cast<int>(QSerialPort::Data5));
    data_bits_combo_->addItem(
        "6", static_cast<int>(QSerialPort::Data6));
    data_bits_combo_->addItem(
        "7", static_cast<int>(QSerialPort::Data7));
    data_bits_combo_->addItem(
        "8", static_cast<int>(QSerialPort::Data8));

    parity_combo_->addItem(
        "无", static_cast<int>(QSerialPort::NoParity));
    parity_combo_->addItem(
        "偶校验", static_cast<int>(QSerialPort::EvenParity));
    parity_combo_->addItem(
        "奇校验", static_cast<int>(QSerialPort::OddParity));
    parity_combo_->addItem(
        "空格校验", static_cast<int>(QSerialPort::SpaceParity));
    parity_combo_->addItem(
        "标记校验", static_cast<int>(QSerialPort::MarkParity));

    stop_bits_combo_->addItem(
        "1", static_cast<int>(QSerialPort::OneStop));
    stop_bits_combo_->addItem(
        "1.5", static_cast<int>(QSerialPort::OneAndHalfStop));
    stop_bits_combo_->addItem(
        "2", static_cast<int>(QSerialPort::TwoStop));

    flow_control_combo_->addItem(
        "无", static_cast<int>(QSerialPort::NoFlowControl));
    flow_control_combo_->addItem(
        "硬件流控", static_cast<int>(QSerialPort::HardwareControl));
    flow_control_combo_->addItem(
        "软件流控", static_cast<int>(QSerialPort::SoftwareControl));

    serial_layout->addRow(serial_title);
    serial_layout->addRow("设备路径", port_name_edit_);
    serial_layout->addRow("波特率", baud_rate_combo_);
    serial_layout->addRow("数据位", data_bits_combo_);
    serial_layout->addRow("校验位", parity_combo_);
    serial_layout->addRow("停止位", stop_bits_combo_);
    serial_layout->addRow("流控", flow_control_combo_);

    root_layout->addWidget(left_panel);
    root_layout->addWidget(serial_panel_);

    connect(wifi_button_, &QPushButton::clicked, this, [this]() {
        setPendingMode(ConnectionMode::Wifi);
    });
    connect(telemetry_button_, &QPushButton::clicked, this, [this]() {
        setPendingMode(ConnectionMode::Telemetry);
    });
    connect(save_button_, &QPushButton::clicked,
            this, &ConnectionInfoDialog::handleSave);
    connect(close_button_, &QPushButton::clicked,
            this, &ConnectionInfoDialog::reject);

    setStyleSheet(
        "QDialog {"
        "background: #101722;"
        "color: #d7e3f4;"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 12px;"
        "}"
        "#shelfDialogTitle {"
        "font-size: 22px;"
        "font-weight: 600;"
        "color: #e7f3ff;"
        "}"
        "#connectionInfoDialog {"
        "background: #16202d;"
        "color: #d7e3f4;"
        "}"
        "QLabel { color: #d7e3f4; }"
        "QPushButton {"
        "min-width: 110px;"
        "padding: 8px 12px;"
        "border: 1px solid #52667d;"
        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "color: #d7e3f4;"
        "border-radius: 5px;"
        "}"
        "QPushButton:hover { background: #30455e; }"
        "QPushButton:checked {"
        "background: #176b87;"
        "border-color: #5cc8e8;"
        "color: white;"
        "}"
        "#serialPanel {"
        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "border: 1px solid #52667d;"
        "border-radius: 6px;"
        "}"
        "QLineEdit, QComboBox {"
        "min-height: 30px;"
        "padding: 3px 8px;"
        "background: #101923;"
        "color: #e8f0f8;"
        "border: 1px solid #52667d;"
        "border-radius: 4px;"
        "}"
    );
}

void ConnectionInfoDialog::loadConfigIntoControls()
{
    const SerialPortConfig &serial =
        working_config_.connection.telemetry_serial;

    port_name_edit_->setText(serial.port_name);

    // 如果 JSON 中使用了列表外的波特率，也保留并显示该数值。
    if (baud_rate_combo_->findData(serial.baud_rate) < 0)
    {
        baud_rate_combo_->addItem(
            QString::number(serial.baud_rate), serial.baud_rate);
    }

    selectComboData(baud_rate_combo_, serial.baud_rate);
    selectComboData(
        data_bits_combo_, static_cast<int>(serial.data_bits));
    selectComboData(
        parity_combo_, static_cast<int>(serial.parity));
    selectComboData(
        stop_bits_combo_, static_cast<int>(serial.stop_bits));
    selectComboData(
        flow_control_combo_, static_cast<int>(serial.flow_control));
}

void ConnectionInfoDialog::setPendingMode(ConnectionMode mode)
{
    pending_mode_ = mode;
    wifi_button_->setChecked(mode == ConnectionMode::Wifi);
    telemetry_button_->setChecked(mode == ConnectionMode::Telemetry);

    // WiFi 只保留高亮；数传才展开右侧串口配置区域。
    serial_panel_->setVisible(mode == ConnectionMode::Telemetry);
    adjustSize();
}

void ConnectionInfoDialog::handleSave()
{
    WarehouseConfig candidate = working_config_;
    // config.ros 是地面站实际使用的接口；bridge_ros 固定不变。
    applyConnectionModeToRosConfig(candidate, pending_mode_);

    if (pending_mode_ == ConnectionMode::Telemetry)
    {
        if (port_name_edit_->text().trimmed().isEmpty())
        {
            QMessageBox::warning(
                this, "参数错误", "数传串口设备路径不能为空。");
            port_name_edit_->setFocus();
            return;
        }
        candidate.connection.telemetry_serial =
            serialConfigFromControls();
    }

    QString error_message;
    if (!saveWarehouseConfig(candidate, &error_message))
    {
        QMessageBox::warning(
            this, "保存失败", error_message);
        return;
    }

    working_config_ = candidate;

    // accept() 会结束 exec()，因此保存成功后窗口会自动关闭。
    accept();
}

void ConnectionInfoDialog::selectComboData(
    QComboBox *combo,
    int value)
{
    const int index = combo->findData(value);
    if (index >= 0)
    {
        combo->setCurrentIndex(index);
    }
}

SerialPortConfig ConnectionInfoDialog::serialConfigFromControls() const
{
    SerialPortConfig config;
    config.port_name = port_name_edit_->text().trimmed();
    config.baud_rate = baud_rate_combo_->currentData().toInt();
    config.data_bits = static_cast<QSerialPort::DataBits>(
        data_bits_combo_->currentData().toInt());
    config.parity = static_cast<QSerialPort::Parity>(
        parity_combo_->currentData().toInt());
    config.stop_bits = static_cast<QSerialPort::StopBits>(
        stop_bits_combo_->currentData().toInt());
    config.flow_control = static_cast<QSerialPort::FlowControl>(
        flow_control_combo_->currentData().toInt());
    return config;
}