#pragma once

#include <QColor>
#include <QRectF>
#include <QSerialPort>
#include <QString>
#include <QVector>

// 判断无人机当前位姿属于货架哪一面的区域。
struct ShelfPoseRegionConfig
{
    QString side;          // 货架面："front" 或 "back"。
    double x_min = 0.0;    // 进入该货架面的最小 X 坐标。
    double x_max = 0.0;    // 进入该货架面的最大 X 坐标。
    double yaw_min = 0.0;  // 进入该货架面的最小航向角，单位：度。
    double yaw_max = 0.0;  // 进入该货架面的最大航向角，单位：度。
};

// 单个货架的低频可调配置。
struct ShelfConfig
{
    QString code;                 // 场景货架编号，例如 "A01"。
    QString display_name;         // 地面站显示名称，例如“货架1”。
    QString front_slot_prefix;    // 正面位置码前缀，例如 "A"。
    QString back_slot_prefix;     // 背面位置码前缀，例如 "B"。

    QRectF base_rect;              // 场景货架底面：X、Y、宽度、长度。
    double height = 0.0;           // 场景货架高度，使用场景坐标单位。
    QColor scene_color;            // 场景绘制颜色。
    QString button_status_color;   // 货架按钮默认状态颜色。

    double front_waypoint_y_m = 0.0; // 正面航点 Y 坐标，单位：米。
    double back_waypoint_y_m = 0.0;  // 背面航点 Y 坐标，单位：米。

    int rows = 0;       // 当前货架每一面的槽位行数。
    int columns = 0;    // 当前货架每一面的槽位列数。
    QVector<double> waypoint_row_z_m;    // 当前货架每一行航点高度，单位：米。
    QVector<double> waypoint_front_x_m;  // 当前货架正面每一列航点 X，单位：米。
    QVector<double> waypoint_back_x_m;   // 当前货架背面每一列航点 X，单位：米。

    QVector<ShelfPoseRegionConfig> pose_regions; // 该货架的位姿识别区域。

    int slotCountPerSide() const
    {
        return rows * columns;
    }
};

// 所有货架共用的参数只保留正面和背面的航向。
// 位姿映射范围暂时保留为内部兼容数据，不再显示在参数配置界面。
struct SlotGridConfig
{
    double front_yaw_rad = 0.0; // 正面航点航向，单位：弧度。
    double back_yaw_rad = 0.0;  // 背面航点航向，单位：弧度。

    double pose_y_min = 0.0; // 位姿映射到槽位列时，Y 坐标最小值。
    double pose_y_max = 0.0; // 位姿映射到槽位列时，Y 坐标最大值。
    double pose_z_min = 0.0; // 位姿映射到槽位行时，Z 坐标最小值。
    double pose_z_max = 0.0; // 位姿映射到槽位行时，Z 坐标最大值。
};

// 一组 ROS 节点、话题和服务名称，可分别用于地面站或串口桥接进程。
struct RosTopicConfig
{
    QString vision_servo_status;      // Visual-servo status topic.
    QString node_name;                // 地面站 ROS 节点名。
    QString drone_status;             // 无人机状态话题。
    QString task_status;              // 任务执行状态话题。
    QString path_ready;               // 航线准备完成话题。
    QString return_world_group;       // 返回规划航线话题。
    QString barcode_capture;          // 带图片的扫码结果话题。
    QString vision_barcode;           // 无线链路转发的扫码结果话题。
    QString local_position;           // 无人机本地位姿话题。
    QString car_local_position;       // 无人车本地位姿话题。
    QString car_keypad_s4_pressed;    // 小车 S4 按键状态话题：按下时单次 true，常态 false。
    QString car_route_state;          // 小车固定路线任务状态话题。
    QString car_control_mode;         // 小车手动暂停和恢复控制话题。
    QString pose_delta;               // 位姿和航向误差话题。
    QString industrial_camera_params; // 地面站发布完整工业相机参数的话题。

    QString start_task_service;       // 启动任务服务。
    QString stop_push_service;        // 停止任务服务。
    QString start_offboard_service;   // 启动 Offboard 服务。
    QString upload_mission_service;   // 上传任务参数和航点服务。
};

// ground_link_bridge 使用的通信串口配置；扫码串口不在这里管理。
struct SerialPortConfig
{
    QString port_name; // 串口设备路径，例如 /dev/serial/by-id/...。
    qint32 baud_rate = QSerialPort::Baud115200; // 波特率。
    QSerialPort::DataBits data_bits = QSerialPort::Data8; // 数据位。
    QSerialPort::Parity parity = QSerialPort::NoParity; // 校验位。
    QSerialPort::StopBits stop_bits = QSerialPort::OneStop; // 停止位。
    QSerialPort::FlowControl flow_control = QSerialPort::NoFlowControl; // 流控方式。
};

// 地面站启动时选择的通信链路。
enum class ConnectionMode
{
    Wifi,      // 通过 WiFi 直连无人机 ROS 话题。
    Telemetry  // 通过 ground_link_bridge 数传串口转发。
};

// 当前启用的巡检项目。该选择会写入 warehouse_config.json，重启后仍然生效。
enum class InspectionProject
{
    Cargo,  // 货物巡检：使用现有仓库 SceneView。
    Animal, // 动物巡检：使用固定二维栅格画板。
    Collaboration // 空地协同
};

// 连接方式和数传串口放在同一个结构中，方便界面整体保存。
struct ConnectionConfig
{
    ConnectionMode mode = ConnectionMode::Wifi; // 当前保存的连接方式。
    SerialPortConfig telemetry_serial;          // 数传连接使用的通信串口。
};

// 地面站保存并整组发布的工业相机参数；数值范围与当前相机驱动一致。
// 这里只做安全的零值初始化；真正的代码默认值统一写在 createDefaultWarehouseConfig()。
struct IndustrialCameraConfig
{
    bool auto_exposure = false; // 是否启用自动曝光；关闭后使用手动曝光时间。
    int exposure_absolute = 0; // 手动曝光时间，范围：1..10000。
    bool auto_exposure_priority = false; // 自动曝光时是否允许降低帧率。
    int gain = 0; // 图像增益，范围：0..190。
    int brightness = 0; // 图像亮度，范围：0..255。
    int contrast = 0; // 图像对比度，范围：0..128。
    int saturation = 0; // 图像饱和度，范围：0..128。
    int gamma = 0; // 图像 Gamma 校正值，范围：0..255。
    int sharpness = 0; // 图像锐度，范围：0..255。
    int backlight_compensation = 0; // 逆光补偿，范围：16..160。
    bool auto_white_balance = false; // 是否启用自动白平衡；关闭后使用手动色温。
    int white_balance_temperature = 0; // 手动白平衡色温，范围：2800..6500 K。
    quint8 power_line_frequency = 0; // 防闪烁频率：0=关闭，1=50 Hz，2=60 Hz。
    bool auto_focus = false; // 是否启用自动对焦；关闭后使用手动焦点。
    int focus_absolute = 0; // 手动焦点位置，范围：0..1023。
    int zoom_absolute = 0; // 相机变焦值，范围：100..200。
};

// 视觉伺服全局参数。
// 这里只做安全的零值初始化；真正的代码默认值统一写在 createDefaultWarehouseConfig()。
struct VisualServoConfig
{
    bool enabled = false; // 是否启用相机伺服；代码默认值在 createDefaultWarehouseConfig() 中设为 true。
    QString target_id; // 目标 ID；留空表示锁定第一个符合条件的目标。
    bool require_confirmed = false; // 是否要求视觉端将目标标记为稳定确认。
    QString image_x_axis; // 图像水平误差映射到的机体系轴：x、y 或 z。
    QString image_y_axis; // 图像垂直误差映射到的机体系轴，不能与 X 映射轴相同。
    double image_x_sign = 0.0; // 图像水平误差到机体运动方向的符号，只能为 -1 或 1。
    double image_y_sign = 0.0; // 图像垂直误差到机体运动方向的符号，只能为 -1 或 1。
    double kp_x = 0.0; // 图像 X 误差的 PID 比例增益。
    double ki_x = 0.0; // 图像 X 误差的 PID 积分增益。
    double kd_x = 0.0; // 图像 X 误差的 PID 微分增益。
    double kp_y = 0.0; // 图像 Y 误差的 PID 比例增益。
    double ki_y = 0.0; // 图像 Y 误差的 PID 积分增益。
    double kd_y = 0.0; // 图像 Y 误差的 PID 微分增益。
    double integral_limit = 0.0; // 积分累计量的绝对值上限。
    double filter_alpha = 0.0; // 低通滤波系数，范围：0..1。
    double enter_tolerance_x = 0.0; // X 误差进入对准状态的阈值。
    double enter_tolerance_y = 0.0; // Y 误差进入对准状态的阈值。
    double exit_tolerance_x = 0.0; // X 误差退出对准状态的阈值。
    double exit_tolerance_y = 0.0; // Y 误差退出对准状态的阈值。
    double settle_time_s = 0.0; // 连续保持对准后判定成功的时间，单位：秒。
    double acquire_timeout_s = 0.0; // 等待首个有效目标的超时，单位：秒。
    double lost_timeout_s = 0.0; // 允许目标连续丢失的时间，单位：秒。
    double overall_timeout_s = 0.0; // 单次视觉伺服动作总超时，单位：秒。
    double max_body_speed_mps = 0.0; // PID 输出的单轴机体系速度上限，单位：米/秒。
    bool continue_on_timeout = false; // 超时后是否继续执行后续任务动作。
};
struct MissionConfig
{
    double takeoff_altitude = 0.0; // 起飞高度，单位：米。
    double move_altitude = 0.0; // 移动高度，单位：米。
    double start_altitude = 0.0; // 解锁/任务起始高度，单位：米。
    double yaw = 0.0; // 默认任务航向。
    double tolerance = 0.0; // 航点到达误差容忍值。
    double yaw_tolerance_deg = 0.0; // 航向到达容差，单位：度。
    double max_xy_speed_mps = 0.0; // 最大水平移动速度，单位：米/秒。
    double max_z_speed_mps = 0.0; // 最大竖直移动速度，单位：米/秒。
    double max_yaw_rate_deg_s = 0.0; // 最大航向角速度，单位：度/秒。
    double takeoff_hover_duration = 0.0; // 起飞后悬停时间，单位：秒。
    double landing_hover_duration = 0.0; // 降落前悬停时间，单位：秒。
    double move_hover_duration = 0.0; // 移动点之间悬停时间，单位：秒。
    bool add_hover_between_takeoff = false; // 是否在起飞阶段加入悬停。
    bool add_hover_between_landing = false; // 是否在降落阶段加入悬停。
    bool add_hover_between_moves = false; // 是否在移动点之间加入悬停。
    bool auto_start_mission = false; // 上传完成后是否自动开始任务。
    bool compress_waypoint_segments = true; // 航点飞行是否压缩连续直线段。
    bool compress_non_waypoint_segments = false; // 非航点任务是否压缩连续直线段。
    QString frame; // 任务航点使用的坐标系名称。
};

// 地面站全部仓库配置。
struct WarehouseConfig
{
    InspectionProject inspection_project = InspectionProject::Cargo; // 当前巡检项目。
    QVector<ShelfConfig> shelves; // 所有货架配置。
    SlotGridConfig slot_grid;     // 共用槽位结构和航点映射。
    RosTopicConfig ros;           // warehouse_gcs 直连 WiFi 时使用的 ROS 接口。
    RosTopicConfig bridge_ros;    // ground_link_bridge 对外提供的串口转发 ROS 接口。
    ConnectionConfig connection;   // 当前连接方式和数传串口参数。
    MissionConfig mission;        // 上传任务时使用的飞行和动作编排参数。
    VisualServoConfig visual_servo; // 视觉伺服全局默认参数。
    IndustrialCameraConfig industrial_camera; // 保存并发布的完整工业相机参数。
};
// 返回编译进程序的固定默认值；不读取 JSON，恢复默认值和首次创建配置文件都使用它。
WarehouseConfig createDefaultWarehouseConfig();

// 根据下标创建一份完整的新货架默认值。参数页增加货架数量时也调用这里，
// 避免默认编号、尺寸、行列和航点数组散落在界面代码中。
ShelfConfig createDefaultShelfConfig(int shelf_index);

// 校验货架、槽位、ROS、通信串口和任务参数；配置有效时返回空字符串。
QString validateWarehouseConfig(const WarehouseConfig &config);

// 保存连接方式前更新 config.ros；bridge_ros 始终保持不变。
void applyConnectionModeToRosConfig(
    WarehouseConfig &config,
    ConnectionMode mode);

// 两个程序共用的数据目录。可通过 WAREHOUSE_GCS_DATA_DIR 环境变量覆盖。
QString warehouseDataDirectory();

// 仓库配置文件和槽位运行数据文件的完整路径。
QString warehouseConfigFilePath();
QString shelfPanelDataFilePath();

// 将仓库配置原子写入 JSON 文件。
bool saveWarehouseConfig(const WarehouseConfig &config,
                         QString *error_message = nullptr);

// 启动时读取 JSON；文件不存在时会用代码默认值创建一份。
bool loadWarehouseConfig(WarehouseConfig &config,
                         QString *error_message = nullptr);
