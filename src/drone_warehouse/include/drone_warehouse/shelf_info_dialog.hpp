#pragma once

#include "drone_warehouse/models.hpp"
#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>
#include <QIcon>
#include <QVector>
#include <QSerialPort>

class QColor;
class QEvent;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QObject;
class QPushButton;

class ShelfInfoDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ShelfInfoDialog(
        const QVector<ShelfConfig> &shelf_configs,//传入每个货架独立的行列和航点配置
        QWidget *parent = nullptr);

    // 主窗口会把“所有货架的完整弹窗数据”一次性传进来。
    // 这里不再像之前那样分散传很多 QStringList，而是直接传结构化后的 QVector<ShelfPanelData>。
    void setShelfPanelData(const QVector<ShelfPanelData> &shelf_panel_data);//将外部数据一次性传入

    // 参数页修改货架数量或某个货架行列后，主窗口调用这里更新动态网格。
    // 仅替换结构配置，不直接修改主窗口持有的货架业务数据。
    void setShelfConfigs(const QVector<ShelfConfig> &shelf_configs);

signals:
    void slotDoubleClicked(int shelf_index, const QString &side, int row, int col);//双击某个点位时通知主窗口弹图
    void manualStockInScanned(int shelf_index, const QString &side, int row, int col,
                              const QString &category_id, const QString &package_id);
    void manualStockOutRequested(int shelf_index, const QString &side, int row, int col,
                                const QString &category_id, const QString &package_id);

    void setWaypointRequested(int shelf_index, const QString &side, int row, int col);
    void clearWaypointRequested();

private:
    bool eventFilter(QObject *watched, QEvent *event) override;//监听槽位按钮双击事件
    void showShelfInfo(int index);//按索引切换当前货架，再刷新网格和详情

    void switchToFront();//切换到前面网格
    void switchToBack();//切换到后面网格
    void buildSlotGrid();//根据配置创建槽位网格按钮
    void buildShelfButtons();//根据配置创建货架切换按钮
    void updateSlotGrid();//按当前面刷新所有网格按钮显示
    void handleSlotClicked(int row, int col);//点击某个点位后的处理
    void updateSlotDetail(const QString &slot_name, const QString &category_id, const QString &package_id);//刷新下方详情区
    int slotIndex(int row, int col) const;//把行列换算成一维下标

    void setupSerial();
    void startManualStockIn();
    void handleManualStockOut();
    void setWaypoint();
    void clearWaypoint();

    void handleSerialFrame(uint8_t deviceId, uint8_t status, const QByteArray &payload);
    void processManualScanText(const QString &scan_text);
    void uart_write(uint8_t deviceId, uint8_t status, const QByteArray &payload);
    bool uart_read(uint8_t &deviceId, uint8_t &status, QByteArray &payload);

    QByteArray encodeFrame(uint8_t sof1, uint8_t sof2,
                                            uint8_t deviceId, uint8_t status,
                                            const QByteArray &payload) const;

    bool validateFrame(const QByteArray &frame,
                                        uint8_t expectedSof1, uint8_t expectedSof2,
                                        uint8_t &deviceId, uint8_t &status,
                                        QByteArray &payload) const;

private:
    QVector<ShelfConfig> shelf_configs_;

    QLabel *title_label_ = nullptr;//弹窗标题

    QPushButton *stock_in_button_ = nullptr;//入库按钮
    QPushButton *outgoing_button_ = nullptr;//出库按钮
    QPushButton *add_button_ = nullptr;//添加航点按钮
    QPushButton *clear_button_ = nullptr;//清空航点按钮
    QPushButton *close_button_ = nullptr;//关闭弹窗按钮

    QPushButton *front_button_ = nullptr;//切换到前面货位网格
    QPushButton *back_button_ = nullptr;//切换到后面货位网格

    QVector<QPushButton*> shelf_buttons_;//保存当前配置对应的全部货架按钮
    QVector<QPushButton*> slot_buttons_;//保存当前配置对应的全部槽位按钮
    QHBoxLayout *shelf_switch_layout_ = nullptr;//动态货架切换按钮布局
    QGridLayout *slot_grid_layout_ = nullptr;//配置驱动的货位网格布局

    QLabel *slot_value_label_ = nullptr;//当前点击的点位，例如 前面 R1C1
    QLabel *category_value_label_ = nullptr;//当前点位的类别编号
    QLabel *package_value_label_ = nullptr;//当前点位的包裹编号

    QSerialPort serial_;

    QVector<ShelfPanelData> shelf_panel_data_;//所有货架的弹窗展示数据

    QByteArray raw_serial_buffer_;
    bool waiting_manual_scan_result_ = false;
    bool manual_scan_ack_received_ = false;
    int pending_shelf_index_ = -1;
    QString pending_side_;
    int pending_row_ = -1;
    int pending_col_ = -1;

    QString current_side_ = "front";//当前查看的是前面还是后面
    int current_slot_row_ = 0;//当前选中的行
    int current_slot_col_ = 0;//当前选中的列
    int current_shelf_index_ = 0;//当前显示的是第几个货架

    int stock_outgoing_ = -1;//0代表是入库按钮触发扫码，1则为出库
};
