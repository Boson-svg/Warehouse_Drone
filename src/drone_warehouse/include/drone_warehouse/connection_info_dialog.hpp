#pragma once

#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>

class QButtonGroup;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QWidget;

class ConnectionInfoDialog : public QDialog
{
public:
    explicit ConnectionInfoDialog(
        const WarehouseConfig &config,
        QWidget *parent = nullptr);

    // 保存成功后，主窗口通过这个接口取得最新配置。
    const WarehouseConfig &savedConfig() const;

private:
    void buildUi();
    void loadConfigIntoControls();
    void setPendingMode(ConnectionMode mode);
    void handleSave();
    void selectComboData(QComboBox *combo, int value);
    SerialPortConfig serialConfigFromControls() const;

    // 弹窗只修改副本，点击保存成功前不会影响主窗口当前配置。
    WarehouseConfig working_config_;
    ConnectionMode pending_mode_ = ConnectionMode::Wifi;

    QLabel *connection_info_label_ = nullptr;
    QPushButton *wifi_button_ = nullptr;
    QPushButton *telemetry_button_ = nullptr;
    QPushButton *save_button_ = nullptr;
    QPushButton *close_button_ = nullptr;
    QButtonGroup *mode_button_group_ = nullptr;

    QWidget *serial_panel_ = nullptr;
    QLineEdit *port_name_edit_ = nullptr;
    QComboBox *baud_rate_combo_ = nullptr;
    QComboBox *data_bits_combo_ = nullptr;
    QComboBox *parity_combo_ = nullptr;
    QComboBox *stop_bits_combo_ = nullptr;
    QComboBox *flow_control_combo_ = nullptr;
};