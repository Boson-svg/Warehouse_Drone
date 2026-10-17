#pragma once

#include <QMainWindow>
#include "drone_qt/position_view_widget.hpp"
#include <QVector>
#include <QMap>

class QLabel;
class QPushButton;
class RosManager;
class QString;

class QListWidget;
class QListWidgetItem;
class ImagePreviewDialog;
class PositionViewWidget;

class QSerialPort;
class QByteArray;

class QPlainTextEdit;

class QComboBox;

class QTimer;

//声明类的定义
class MainWindow : public QMainWindow
{
    public:
        //构造函数和析构函数
        explicit MainWindow(QWidget *parent = nullptr);
        ~MainWindow() override = default;
    
    private:
        //负责创建界面
        void setupUi();
        //负责信号槽连接
        void setupConnections();
        //负责结果显示
        void updateCommandResult(bool success,const QString &message);

        //负责状态更新
        void updateStatus(
            bool connected,
            float battery_percent,
            int flight_mode,
            bool armed,
            const QString &task_name);

        //负责action更新
        void action_updateStatus(
            bool task_running,
            int action_step,
            int action_num,
            const QString &action_name);

        void updateDelta(double dx,double dy,double dyaw,bool valid);

        //定义一个结构体，用于存储条形码捕获记录，包括条形码数据、图像数据、图像格式和时间文本等信息
        struct BarcodeRecord {
            QString barcode;
            QByteArray image_data;
            QString image_format;
            QString time_text;};

        //负责添加条形码捕获记录到界面上的列表控件中，并在点击列表项时显示图像预览对话框
        void appendBarcodeRecord(
            const QString &barcode,
            const QByteArray &image_data,
            const QString &image_format,
            const QString &time_text); 

        //计数模式用
        void appendVisionBarcodeCount(
            const QString &barcode,
            const QString &time_text);

        enum class BarcodeDisplayMode
        {
            Summary,//计数模式
            Detail//明细模式
        };

        //当用户双击列表项时，显示对应的图像预览对话框
        void showBarcodeImage(QListWidgetItem *item);

        //负责更新条形码捕获列表的高度，以适应内容的变化
        void updateBarcodeListHeight();

        //统一重建识别结果区显示
        void refreshBarcodeList();
        //下拉框切换时更新模式
        void handleBarcodeDisplayModeChanged(int index);

        //负责确认控制程序会传的状态
        void updatePathReadyState(bool ready);   

        //负责确认控制程序会传的路线
        void updateWorldGroupState(const QVector<WorldCoord> &points);   

        //负责起飞前判断
        void handleStartButtonClicked();

        //串口基础配置函数
        void setupSerialPort();
        //读取串口数据
        void handleSerialReadyRead();
        //对串口数据进行处理
        void processSerialLine(const QString &line);
        //根据串口数据触发动作
        void triggerDirectionCommand(const QString &command);


        QString current_flight_mode = "UNKNOWN";//当前飞行模式字符串

        //界面元素和ROS管理器的成员变量
        QLabel *status_label_{nullptr};//状态标签
        QLabel *result_label_{nullptr};//结果标签
        QLabel *battery_label_{nullptr};//电量标签
        QLabel *mode_label_{nullptr};//模式标签
        QLabel *armed_label_{nullptr};//解锁状态标签
        QLabel *connection_label_{nullptr};//连接状态标签
        QLabel *action_label_{nullptr};//动作标签
        QLabel *progress_label_{nullptr};//进度标签
        QLabel *progress_percent_label_{nullptr};//百分比进度标签
        QLabel *dx_indicator_label_{nullptr};//dx指示灯标签
        QLabel *dy_indicator_label_{nullptr};//dy指示灯标签
        QLabel *dyaw_indicator_label_{nullptr};//dyaw指示灯标签
        QLabel *dx_value_label_{nullptr};//dx数值标签
        QLabel *dy_value_label_{nullptr};//dy数值标签
        QLabel *dyaw_value_label_{nullptr};//dyaw数值标签

        QPushButton *start_button_{nullptr};//开始按钮
        QPushButton *stop_button_{nullptr};//停止按钮
        QPushButton *true_button_{nullptr};//选择按钮
        QPushButton *del_button_{nullptr};//取消按钮
        QPushButton *clear_button_{nullptr};//清空按钮
        QPushButton *refresh_button_{nullptr};//刷新按钮
        QPushButton *display_button_{nullptr};//显示按钮
        QPushButton *push_button_{nullptr};//上传按钮

        QPushButton *up_button_{nullptr};//上按钮
        QPushButton *down_button_{nullptr};//下按钮
        QPushButton *left_button_{nullptr};//左按钮
        QPushButton *right_button_{nullptr};//右按钮

        QPlainTextEdit *run_log_view_{nullptr};

        QComboBox *barcode_mode_combo_{nullptr};//识别结果区上方的下拉框
        BarcodeDisplayMode barcode_display_mode_{BarcodeDisplayMode::Summary};

        QMap<QString, int> vision_barcode_counts_;//计数模式容器

        QSerialPort *serial_port_{nullptr};//串口对象
        QByteArray serial_buffer_;//串口缓冲区

        QListWidget *barcode_list_{nullptr};//条形码捕获列表控件
        QVector<BarcodeRecord> barcode_records_;//存储条形码捕获记录的向量
        ImagePreviewDialog *image_preview_dialog_{nullptr};//图像预览对话框

        RosManager *ros_manager_{nullptr};//ROS管理器
        PositionViewWidget *position_view_{nullptr};//位置显示控件

        QTimer *exit_long_press_timer_{nullptr};
        bool long_press_triggered_{false};

        int stop_press_token_{0};
        bool stop_button_pressed_{false};

        bool path_ready_{false};  //确认控制程序路线是否准备好
        bool waiting_task_result_{false};//防止开始按钮反复触发逻辑
        bool waiting_push_result_{false};//防止上传按钮反复触发逻辑

        bool task_running_{false};
        float progress_{0.0};//任务进度
        bool delta_result_{true};//是否打印compare数据
        bool push_flag_{false};//是否上传路线

        bool unlock_flag_{false};//是否是从开锁到解锁的状态
        bool auto_stop_flag_{false};//判断是否自动执行过stop
        int disarm_stable_count_{0};    // 连续收到 armed == false 的次数
};