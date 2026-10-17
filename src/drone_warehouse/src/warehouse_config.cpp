#include "drone_warehouse/warehouse_config.hpp"

#include "drone_warehouse/color_palette.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSet>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
QString validateRosConfig(const RosTopicConfig &config, const QString &owner)
{
    const QStringList required_values = {
        config.node_name,
        config.drone_status,
        config.task_status,
        config.path_ready,
        config.return_world_group,
        config.barcode_capture,
        config.vision_barcode,
        config.vision_servo_status,
        config.local_position,
        config.car_local_position,
        config.car_keypad_s4_pressed,
        config.car_route_state,
        config.car_control_mode,
        config.pose_delta,
        config.industrial_camera_params,
        config.start_task_service,
        config.stop_push_service,
        config.start_offboard_service,
        config.upload_mission_service
    };

    for (const QString &value : required_values)
    {
        if (value.trimmed().isEmpty())
        {
            return owner + " has an empty ROS node, topic, or service name";
        }
    }
    return {};
}
}

/*************************************************************/

/**************************参数默认值**************************/

ShelfConfig createDefaultShelfConfig(int shelf_index)
{
    const int safe_index = qBound(0, shelf_index, 19);
    ShelfConfig shelf;
    shelf.code = QString("A%1").arg(safe_index + 1, 2, 10, QLatin1Char('0'));
    shelf.display_name = shelf.code;
    shelf.front_slot_prefix = shelf.code + "F";
    shelf.back_slot_prefix = shelf.code + "B";

    // 画板默认按每行 5 个货架排列，新增货架不会覆盖已有货架。
    // Number shelves from right to left: A01 uses the former A02 position,
    // A02 uses the former A01 position, and later shelves continue leftward.
    const int scene_column = safe_index % 5;
    const int scene_row = safe_index / 5;
    shelf.base_rect = QRectF(
        60.0 - scene_column * 150.0,
        -100.0 + scene_row * 220.0,
        30.0,
        150.0);
    shelf.height = 160.0;
    shelf.scene_color =
        ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    shelf.button_status_color = "#7f8c9a";

    shelf.front_waypoint_y_m = safe_index * 1.5;
    shelf.back_waypoint_y_m = shelf.front_waypoint_y_m + 1.5;
    shelf.rows = 4;
    shelf.columns = 3;
    shelf.waypoint_row_z_m = {1.380, 1.000, 0.620, 0.200};
    shelf.waypoint_front_x_m = {0.75, 1.25, 1.75};
    shelf.waypoint_back_x_m = {1.75, 1.25, 0.75};
    return shelf;
}

WarehouseConfig createDefaultWarehouseConfig()
{
    // 这里构造的是编译进程序的代码默认值，不读取 warehouse_config.json。
    // 用户保存配置只会改变 JSON；再次调用本函数仍会得到这套固定值。
    WarehouseConfig config;

    // 所有货架只共用正面和背面航向，行列及坐标属于各自货架。
    config.slot_grid.front_yaw_rad = 1.57;
    config.slot_grid.back_yaw_rad = 4.71;

    // 旧的位姿识别逻辑仍使用这组内部固定范围，但不再显示为可调参数。
    config.slot_grid.pose_y_min = -100.0;
    config.slot_grid.pose_y_max = 50.0;
    config.slot_grid.pose_z_min = 0.0;
    config.slot_grid.pose_z_max = 160.0;

    ShelfConfig shelf1 = createDefaultShelfConfig(0);
    shelf1.pose_regions = {
        {"front", 110.0, 140.0, 80.0, 100.0},
        {"back", -10.0, 10.0, -100.0, -80.0}
    };

    ShelfConfig shelf2 = createDefaultShelfConfig(1);
    shelf2.pose_regions = {
        {"front", -10.0, 10.0, 80.0, 100.0},
        {"back", -140.0, -110.0, -100.0, -80.0}
    };

    config.shelves = {shelf1, shelf2};

    // warehouse_gcs 默认通过 WiFi 直连下列 ROS 接口。
    config.ros.node_name = "ground_qt_node";
    config.ros.drone_status = "/drone/status";
    config.ros.task_status = "/drone/task/status";
    config.ros.path_ready = "/drone/control/path_ready";
    config.ros.return_world_group = "/drone/return/world_group";
    config.ros.barcode_capture = "/drone/barcode_capture";
    config.ros.vision_barcode = "/drone/vision/barcode";
    config.ros.vision_servo_status = "/control/vision_servo/status";
    config.ros.local_position = "/drone/local_position";
    config.ros.car_local_position = "/car/local_position";
    config.ros.car_keypad_s4_pressed = "/keypad/s4_pressed";
    config.ros.car_route_state = "/route/state";
    config.ros.car_control_mode = "/control/mode";
    config.ros.pose_delta = "/drone/pose_yaw_compare/delta";
    config.ros.industrial_camera_params = "/industrial_camera/params";
    config.ros.start_task_service = "/drone/start_task";
    config.ros.stop_push_service = "/drone/stop_push";
    config.ros.start_offboard_service = "/drone/start_offboard";
    config.ros.upload_mission_service = "/drone/upload_mission_summary";

    // ground_link_bridge 使用独立的 /serial 命名空间转发串口通信。
    config.bridge_ros.node_name = "ground_link_bridge";
    config.bridge_ros.drone_status = "/serial/drone/status";
    config.bridge_ros.task_status = "/serial/drone/task/status";
    config.bridge_ros.path_ready = "/serial/drone/control/path_ready";
    config.bridge_ros.return_world_group = "/serial/drone/return/world_group";
    config.bridge_ros.barcode_capture = "/drone/barcode_capture";
    config.bridge_ros.vision_barcode = "/serial/drone/vision/barcode";
    config.bridge_ros.vision_servo_status = "/serial/control/vision_servo/status";
    config.bridge_ros.local_position = "/serial/drone/local_position";
    config.bridge_ros.car_local_position = "/car/local_position";
    config.bridge_ros.car_keypad_s4_pressed = "/car/keypad/s4_pressed";
    config.bridge_ros.car_route_state = "/route/state";
    config.bridge_ros.car_control_mode = "/car/control_mode";
    config.bridge_ros.pose_delta = "/serial/drone/pose_yaw_compare/delta";
    config.bridge_ros.industrial_camera_params = "/serial/industrial_camera/params";
    config.bridge_ros.start_task_service = "/serial/drone/start_task";
    config.bridge_ros.stop_push_service = "/serial/drone/stop_push";
    config.bridge_ros.start_offboard_service = "/serial/drone/start_offboard";
    config.bridge_ros.upload_mission_service = "/serial/drone/upload_mission_summary";

    // 默认使用 WiFi；数传串口保留原 ground_link_bridge 参数。
    // 扫码串口仍由 ShelfInfoDialog 独立管理，不属于这里的连接设置。
    config.connection.mode = ConnectionMode::Wifi;
    config.connection.telemetry_serial.port_name = "/dev/warehouse_drone_serial";
    config.connection.telemetry_serial.baud_rate = QSerialPort::Baud115200;
    config.connection.telemetry_serial.data_bits = QSerialPort::Data8;
    config.connection.telemetry_serial.parity = QSerialPort::NoParity;
    config.connection.telemetry_serial.stop_bits = QSerialPort::OneStop;
    config.connection.telemetry_serial.flow_control = QSerialPort::NoFlowControl;

    // 任务飞行基础参数。
    config.mission.takeoff_altitude = 0.0;
    config.mission.move_altitude = 1.2;
    config.mission.start_altitude = 0.0;
    config.mission.yaw = 0.0;
    config.mission.tolerance = 0.10;
    config.mission.yaw_tolerance_deg = 4.0;
    config.mission.max_xy_speed_mps = 0.50;
    config.mission.max_z_speed_mps = 0.30;
    config.mission.max_yaw_rate_deg_s = 40.0;
    config.mission.takeoff_hover_duration = 0.0;
    config.mission.landing_hover_duration = 1.0;
    config.mission.move_hover_duration = 3.0;
    config.mission.add_hover_between_takeoff = false;
    config.mission.add_hover_between_landing = true;
    config.mission.add_hover_between_moves = true;
    config.mission.auto_start_mission = false;
    config.mission.compress_waypoint_segments = true;
    config.mission.compress_non_waypoint_segments = false;
    config.mission.frame = "world_body";

    // 视觉伺服固定默认值。修改这里后需要重新编译，运行时 JSON 不会覆盖这套基准值。
    config.visual_servo.enabled = true;
    config.visual_servo.target_id.clear();
    config.visual_servo.require_confirmed = true;
    config.visual_servo.image_x_axis = "y";
    config.visual_servo.image_y_axis = "z";
    config.visual_servo.image_x_sign = -1.0;
    config.visual_servo.image_y_sign = -1.0;
    config.visual_servo.kp_x = 0.35;
    config.visual_servo.ki_x = 0.0;
    config.visual_servo.kd_x = 0.02;
    config.visual_servo.kp_y = 0.35;
    config.visual_servo.ki_y = 0.0;
    config.visual_servo.kd_y = 0.02;
    config.visual_servo.integral_limit = 0.5;
    config.visual_servo.filter_alpha = 0.35;
    config.visual_servo.enter_tolerance_x = 0.04;
    config.visual_servo.enter_tolerance_y = 0.04;
    config.visual_servo.exit_tolerance_x = 0.07;
    config.visual_servo.exit_tolerance_y = 0.07;
    config.visual_servo.settle_time_s = 0.6;
    config.visual_servo.acquire_timeout_s = 5.0;
    config.visual_servo.lost_timeout_s = 1.0;
    config.visual_servo.overall_timeout_s = 20.0;
    config.visual_servo.max_body_speed_mps = 0.20;
    config.visual_servo.continue_on_timeout = true;

    // 工业相机固定默认值；范围必须与相机驱动和 validateWarehouseConfig() 一致。
    config.industrial_camera.auto_exposure = true;
    config.industrial_camera.exposure_absolute = 156;
    config.industrial_camera.auto_exposure_priority = false;
    config.industrial_camera.gain = 130;
    config.industrial_camera.brightness = 128;
    config.industrial_camera.contrast = 65;
    config.industrial_camera.saturation = 90;
    config.industrial_camera.gamma = 130;
    config.industrial_camera.sharpness = 128;
    config.industrial_camera.backlight_compensation = 48;
    config.industrial_camera.auto_white_balance = true;
    config.industrial_camera.white_balance_temperature = 4650;
    config.industrial_camera.power_line_frequency = 1;
    config.industrial_camera.auto_focus = true;
    config.industrial_camera.focus_absolute = 0;
    config.industrial_camera.zoom_absolute = 100;

    return config;
}

/************************************************************/

/*************************字段校验函数*************************/

QString validateWarehouseConfig(const WarehouseConfig &config)
{
    if (config.shelves.isEmpty() || config.shelves.size() > 20)
    {
        return "shelf count must be between 1 and 20";
    }
    if (config.slot_grid.pose_y_max <= config.slot_grid.pose_y_min ||
        config.slot_grid.pose_z_max <= config.slot_grid.pose_z_min)
    {
        return "slot pose mapping ranges are invalid";
    }

    if (config.connection.mode != ConnectionMode::Wifi &&
        config.connection.mode != ConnectionMode::Telemetry)
    {
        return "connection mode is invalid";
    }

    const SerialPortConfig &telemetry_serial =
        config.connection.telemetry_serial;
    if (telemetry_serial.port_name.trimmed().isEmpty() ||
        telemetry_serial.baud_rate <= 0 ||
        telemetry_serial.data_bits == QSerialPort::UnknownDataBits ||
        telemetry_serial.parity == QSerialPort::UnknownParity ||
        telemetry_serial.stop_bits == QSerialPort::UnknownStopBits ||
        telemetry_serial.flow_control == QSerialPort::UnknownFlowControl)
    {
        return "ground_link_bridge has invalid serial port settings";
    }

    const VisualServoConfig &visual = config.visual_servo;
    const QVector<double> numeric_values = {
        config.mission.takeoff_altitude,
        config.mission.move_altitude,
        config.mission.start_altitude,
        config.mission.yaw,
        config.mission.tolerance,
        config.mission.yaw_tolerance_deg,
        config.mission.max_xy_speed_mps,
        config.mission.max_z_speed_mps,
        config.mission.max_yaw_rate_deg_s,
        config.mission.takeoff_hover_duration,
        config.mission.landing_hover_duration,
        config.mission.move_hover_duration,
        visual.image_x_sign,
        visual.image_y_sign,
        visual.kp_x,
        visual.ki_x,
        visual.kd_x,
        visual.kp_y,
        visual.ki_y,
        visual.kd_y,
        visual.integral_limit,
        visual.filter_alpha,
        visual.enter_tolerance_x,
        visual.enter_tolerance_y,
        visual.exit_tolerance_x,
        visual.exit_tolerance_y,
        visual.settle_time_s,
        visual.acquire_timeout_s,
        visual.lost_timeout_s,
        visual.overall_timeout_s,
        visual.max_body_speed_mps
    };
    for (double value : numeric_values)
    {
        if (!std::isfinite(value))
        {
            return "mission config contains a non-finite number";
        }
    }

    if (config.mission.takeoff_altitude < 0.0 ||
        config.mission.move_altitude < 0.0 ||
        config.mission.start_altitude < 0.0 ||
        config.mission.tolerance <= 0.0 ||
        config.mission.yaw_tolerance_deg <= 0.0 ||
        config.mission.max_xy_speed_mps <= 0.0 ||
        config.mission.max_z_speed_mps <= 0.0 ||
        config.mission.max_yaw_rate_deg_s <= 0.0 ||
        config.mission.takeoff_hover_duration < 0.0 ||
        config.mission.landing_hover_duration < 0.0 ||
        config.mission.move_hover_duration < 0.0 ||
        config.mission.frame.trimmed().isEmpty())
    {
        return "mission flight settings are invalid";
    }

    const QStringList valid_axes = {"x", "y", "z"};
    if (!valid_axes.contains(visual.image_x_axis.toLower()) ||
        !valid_axes.contains(visual.image_y_axis.toLower()) ||
        visual.image_x_axis.compare(visual.image_y_axis, Qt::CaseInsensitive) == 0 ||
        (visual.image_x_sign != -1.0 && visual.image_x_sign != 1.0) ||
        (visual.image_y_sign != -1.0 && visual.image_y_sign != 1.0) ||
        visual.integral_limit <= 0.0 ||
        visual.filter_alpha < 0.0 || visual.filter_alpha > 1.0 ||
        visual.enter_tolerance_x <= 0.0 ||
        visual.enter_tolerance_y <= 0.0 ||
        visual.exit_tolerance_x < visual.enter_tolerance_x ||
        visual.exit_tolerance_y < visual.enter_tolerance_y ||
        visual.settle_time_s < 0.0 ||
        visual.acquire_timeout_s <= 0.0 ||
        visual.lost_timeout_s <= 0.0 ||
        visual.overall_timeout_s < visual.acquire_timeout_s ||
        visual.overall_timeout_s < visual.lost_timeout_s ||
        visual.max_body_speed_mps <= 0.0)
    {
        return "visual-servo settings are invalid";
    }
    const IndustrialCameraConfig &camera = config.industrial_camera;
    if (camera.exposure_absolute < 1 || camera.exposure_absolute > 10000 ||
        camera.gain < 0 || camera.gain > 190 ||
        camera.brightness < 0 || camera.brightness > 255 ||
        camera.contrast < 0 || camera.contrast > 128 ||
        camera.saturation < 0 || camera.saturation > 128 ||
        camera.gamma < 0 || camera.gamma > 255 ||
        camera.sharpness < 0 || camera.sharpness > 255 ||
        camera.backlight_compensation < 16 ||
        camera.backlight_compensation > 160 ||
        camera.white_balance_temperature < 2800 ||
        camera.white_balance_temperature > 6500 ||
        camera.power_line_frequency > 2 ||
        camera.focus_absolute < 0 || camera.focus_absolute > 1023 ||
        camera.zoom_absolute < 100 || camera.zoom_absolute > 200)
    {
        return "industrial-camera settings are outside supported ranges";
    }
    QSet<QString> shelf_codes;
    QSet<QString> slot_prefixes;
    for (const ShelfConfig &shelf : config.shelves)
    {
        const QString shelf_code = shelf.code.trimmed().toUpper();
        bool allowed_code = false;
        for (int code_index = 1; code_index <= 20; ++code_index)
        {
            const QString allowed =
                QString("A%1").arg(code_index, 2, 10, QLatin1Char('0'));
            if (shelf_code == allowed)
            {
                allowed_code = true;
                break;
            }
        }
        if (!allowed_code || shelf.display_name.trimmed().isEmpty())
        {
            return "shelf code must be selected from A01 to A20";
        }
        if (shelf_codes.contains(shelf_code))
        {
            return shelf.code + " has a duplicate shelf code";
        }
        shelf_codes.insert(shelf_code);
        if (!shelf.scene_color.isValid() ||
            !QColor(shelf.button_status_color).isValid() || shelf.height <= 0.0 ||
            shelf.base_rect.width() <= 0.0 || shelf.base_rect.height() <= 0.0)
        {
            return shelf.code + " has invalid geometry, height, or internal color";
        }
        if (shelf.rows <= 0 || shelf.columns <= 0 ||
            shelf.waypoint_row_z_m.size() != shelf.rows ||
            shelf.waypoint_front_x_m.size() != shelf.columns ||
            shelf.waypoint_back_x_m.size() != shelf.columns)
        {
            return shelf.code + " has invalid rows, columns, or waypoint coordinates";
        }
        const auto all_finite = [](const QVector<double> &values) {
            return std::all_of(values.cbegin(), values.cend(),
                               [](double value) { return std::isfinite(value); });
        };
        if (!all_finite(shelf.waypoint_row_z_m) ||
            !all_finite(shelf.waypoint_front_x_m) ||
            !all_finite(shelf.waypoint_back_x_m) ||
            !std::isfinite(shelf.front_waypoint_y_m) ||
            !std::isfinite(shelf.back_waypoint_y_m))
        {
            return shelf.code + " contains a non-finite waypoint coordinate";
        }

        const QString front_prefix = shelf.front_slot_prefix.trimmed().toUpper();
        const QString back_prefix = shelf.back_slot_prefix.trimmed().toUpper();
        if (front_prefix.isEmpty() || back_prefix.isEmpty() ||
            slot_prefixes.contains(front_prefix) || slot_prefixes.contains(back_prefix) ||
            front_prefix == back_prefix)
        {
            return shelf.code + " has an invalid or duplicate slot prefix";
        }
        slot_prefixes.insert(front_prefix);
        slot_prefixes.insert(back_prefix);

        QSet<QString> pose_sides;
        for (const ShelfPoseRegionConfig &region : shelf.pose_regions)
        {
            if ((region.side != "front" && region.side != "back") ||
                region.x_max < region.x_min || region.yaw_max < region.yaw_min)
            {
                return shelf.code + " has an invalid pose recognition region";
            }
            pose_sides.insert(region.side);
        }
        if (!shelf.pose_regions.isEmpty() &&
            (!pose_sides.contains("front") || !pose_sides.contains("back")))
        {
            return shelf.code + " requires front and back pose recognition regions";
        }
    }

    QString error = validateRosConfig(config.ros, "warehouse_gcs");
    if (!error.isEmpty())
    {
        return error;
    }
    return validateRosConfig(config.bridge_ros, "ground_link_bridge");
}

/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/




namespace
{
/***********************读JSON字段函数**************************/
bool jsonError(QString *error_message, const QString &message)
{
    if (error_message)
    {
        *error_message = message;
    }
    return false;
}

bool readObject(const QJsonObject &parent,
                const QString &key,
                QJsonObject &value,
                QString *error_message)
{
    const QJsonValue json_value = parent.value(key);
    if (!json_value.isObject())
    {
        return jsonError(error_message, key + " 必须是 JSON 对象");
    }
    value = json_value.toObject();
    return true;
}

bool readArray(const QJsonObject &parent,
               const QString &key,
               QJsonArray &value,
               QString *error_message)
{
    const QJsonValue json_value = parent.value(key);
    if (!json_value.isArray())
    {
        return jsonError(error_message, key + " 必须是 JSON 数组");
    }
    value = json_value.toArray();
    return true;
}

bool readString(const QJsonObject &object,
                const QString &key,
                QString &value,
                QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isString())
    {
        return jsonError(error_message, key + " 必须是字符串");
    }
    value = json_value.toString();
    return true;
}

bool readDouble(const QJsonObject &object,
                const QString &key,
                double &value,
                QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isDouble())
    {
        return jsonError(error_message, key + " 必须是数字");
    }
    value = json_value.toDouble();
    return true;
}

bool readInt(const QJsonObject &object,
             const QString &key,
             int &value,
             QString *error_message)
{
    double number = 0.0;
    if (!readDouble(object, key, number, error_message))
    {
        return false;
    }
    if (!std::isfinite(number) || std::floor(number) != number ||
        number < static_cast<double>(std::numeric_limits<int>::min()) ||
        number > static_cast<double>(std::numeric_limits<int>::max()))
    {
        return jsonError(error_message, key + " 必须是整数");
    }
    value = static_cast<int>(number);
    return true;
}


bool readBool(const QJsonObject &object,
              const QString &key,
              bool &value,
              QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isBool())
    {
        return jsonError(error_message, key + " 必须是 true 或 false");
    }
    value = json_value.toBool();
    return true;
}

// JSON 使用稳定的英文值，避免界面中文名称变化后旧配置无法读取。
QString inspectionProjectToString(InspectionProject project)
{
    switch (project)
    {
    case InspectionProject::Cargo:
        return "cargo";
    case InspectionProject::Animal:
        return "animal";
    case InspectionProject::Collaboration:
        return "collaboration";
    }

    return "cargo";
}

/************************************************************/

/***********************巡检项目相关函数**********************/

bool inspectionProjectFromString(const QString &value,
                                 InspectionProject &project,
                                 QString *error_message)
{
    if (value == "cargo")
    {
        project = InspectionProject::Cargo;
        return true;
    }
    if (value == "animal")
    {
        project = InspectionProject::Animal;
        return true;
    }
    if (value == "collaboration")
    {
        project = InspectionProject::Collaboration;
        return true;
    }
    return jsonError(
        error_message,
        "inspection_project 无效");
}

QJsonArray doubleVectorToJson(const QVector<double> &values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

bool doubleVectorFromJson(const QJsonObject &object,
                          const QString &key,
                          QVector<double> &values,
                          QString *error_message)
{
    QJsonArray array;
    if (!readArray(object, key, array, error_message))
    {
        return false;
    }

    QVector<double> loaded_values;
    loaded_values.reserve(array.size());
    for (int index = 0; index < array.size(); ++index)
    {
        if (!array.at(index).isDouble())
        {
            return jsonError(
                error_message,
                QString("%1[%2] 必须是数字").arg(key).arg(index));
        }
        loaded_values.push_back(array.at(index).toDouble());
    }
    values = loaded_values;
    return true;
}

/************************************************************/

/***********************ros通信相关函数***********************/

QJsonObject rosConfigToJson(const RosTopicConfig &config)
{
    QJsonObject object;
    object.insert("node_name", config.node_name);
    object.insert("drone_status", config.drone_status);
    object.insert("task_status", config.task_status);
    object.insert("path_ready", config.path_ready);
    object.insert("return_world_group", config.return_world_group);
    object.insert("barcode_capture", config.barcode_capture);
    object.insert("vision_barcode", config.vision_barcode);
    object.insert("vision_servo_status", config.vision_servo_status);
    object.insert("local_position", config.local_position);
    object.insert("car_local_position", config.car_local_position);
    object.insert("car_keypad_s4_pressed", config.car_keypad_s4_pressed);
    object.insert("car_route_state", config.car_route_state);
    object.insert("car_control_mode", config.car_control_mode);
    object.insert("pose_delta", config.pose_delta);
    object.insert("industrial_camera_params", config.industrial_camera_params);
    object.insert("start_task_service", config.start_task_service);
    object.insert("stop_push_service", config.stop_push_service);
    object.insert("start_offboard_service", config.start_offboard_service);
    object.insert("upload_mission_service", config.upload_mission_service);
    return object;
}

bool rosConfigFromJson(const QJsonObject &object,
                       RosTopicConfig &config,
                       QString *error_message)
{
    const bool core_valid =
        readString(object, "node_name", config.node_name, error_message) &&
        readString(object, "drone_status", config.drone_status, error_message) &&
        readString(object, "task_status", config.task_status, error_message) &&
        readString(object, "path_ready", config.path_ready, error_message) &&
        readString(object, "return_world_group", config.return_world_group, error_message) &&
        readString(object, "barcode_capture", config.barcode_capture, error_message) &&
        readString(object, "vision_barcode", config.vision_barcode, error_message) &&
        readString(object, "local_position", config.local_position, error_message) &&
        readString(object, "pose_delta", config.pose_delta, error_message) &&
        readString(object, "start_task_service", config.start_task_service, error_message) &&
        readString(object, "stop_push_service", config.stop_push_service, error_message) &&
        readString(object, "start_offboard_service", config.start_offboard_service, error_message) &&
        readString(object, "upload_mission_service", config.upload_mission_service, error_message);
    if (!core_valid)
    {
        return false;
    }

    if (object.contains("vision_servo_status") &&
        !readString(object, "vision_servo_status",
                    config.vision_servo_status, error_message))
    {
        return false;
    }

    // Older JSON versions keep the current code defaults for new topics.
    if (object.contains("car_local_position") &&
        !readString(object, "car_local_position",
                    config.car_local_position, error_message))
    {
        return false;
    }

    if (object.contains("car_keypad_s4_pressed") &&
        !readString(object, "car_keypad_s4_pressed",
                    config.car_keypad_s4_pressed, error_message))
    {
        return false;
    }

    if (object.contains("car_route_state") &&
        !readString(object, "car_route_state",
                    config.car_route_state, error_message))
    {
        return false;
    }

    if (object.contains("car_control_mode") &&
        !readString(object, "car_control_mode",
                    config.car_control_mode, error_message))
    {
        return false;
    }

    return !object.contains("industrial_camera_params") ||
           readString(object, "industrial_camera_params",
                      config.industrial_camera_params, error_message);
}

/************************************************************/

/***********************数传串口相关函数**********************/

QString parityToString(QSerialPort::Parity parity)
{
    switch (parity)
    {
    case QSerialPort::NoParity:
        return "none";
    case QSerialPort::EvenParity:
        return "even";
    case QSerialPort::OddParity:
        return "odd";
    case QSerialPort::SpaceParity:
        return "space";
    case QSerialPort::MarkParity:
        return "mark";
    default:
        return "unknown";
    }
}

bool parityFromString(const QString &text,
                      QSerialPort::Parity &parity,
                      QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "none")
    {
        parity = QSerialPort::NoParity;
    }
    else if (value == "even")
    {
        parity = QSerialPort::EvenParity;
    }
    else if (value == "odd")
    {
        parity = QSerialPort::OddParity;
    }
    else if (value == "space")
    {
        parity = QSerialPort::SpaceParity;
    }
    else if (value == "mark")
    {
        parity = QSerialPort::MarkParity;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.parity 取值无效");
    }
    return true;
}

QString flowControlToString(QSerialPort::FlowControl flow_control)
{
    switch (flow_control)
    {
    case QSerialPort::NoFlowControl:
        return "none";
    case QSerialPort::HardwareControl:
        return "hardware";
    case QSerialPort::SoftwareControl:
        return "software";
    default:
        return "unknown";
    }
}

bool flowControlFromString(const QString &text,
                           QSerialPort::FlowControl &flow_control,
                           QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "none")
    {
        flow_control = QSerialPort::NoFlowControl;
    }
    else if (value == "hardware")
    {
        flow_control = QSerialPort::HardwareControl;
    }
    else if (value == "software")
    {
        flow_control = QSerialPort::SoftwareControl;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.flow_control 取值无效");
    }
    return true;
}

double stopBitsToJsonNumber(QSerialPort::StopBits stop_bits)
{
    switch (stop_bits)
    {
    case QSerialPort::OneStop:
        return 1.0;
    case QSerialPort::OneAndHalfStop:
        return 1.5;
    case QSerialPort::TwoStop:
        return 2.0;
    default:
        return -1.0;
    }
}

bool stopBitsFromJsonNumber(double value,
                            QSerialPort::StopBits &stop_bits,
                            QString *error_message)
{
    if (value == 1.0)
    {
        stop_bits = QSerialPort::OneStop;
    }
    else if (value == 1.5)
    {
        stop_bits = QSerialPort::OneAndHalfStop;
    }
    else if (value == 2.0)
    {
        stop_bits = QSerialPort::TwoStop;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.stop_bits 只能是 1、1.5 或 2");
    }
    return true;
}

QJsonObject serialConfigToJson(const SerialPortConfig &config)
{
    QJsonObject object;
    object.insert("port_name", config.port_name);
    object.insert("baud_rate", config.baud_rate);
    object.insert("data_bits", static_cast<int>(config.data_bits));
    object.insert("parity", parityToString(config.parity));
    object.insert("stop_bits", stopBitsToJsonNumber(config.stop_bits));
    object.insert("flow_control", flowControlToString(config.flow_control));
    return object;
}

bool serialConfigFromJson(const QJsonObject &object,
                          SerialPortConfig &config,
                          QString *error_message)
{
    int baud_rate = 0;
    int data_bits = 0;
    QString parity;
    double stop_bits = 0.0;
    QString flow_control;
    if (!readString(object, "port_name", config.port_name, error_message) ||
        !readInt(object, "baud_rate", baud_rate, error_message) ||
        !readInt(object, "data_bits", data_bits, error_message) ||
        !readString(object, "parity", parity, error_message) ||
        !readDouble(object, "stop_bits", stop_bits, error_message) ||
        !readString(object, "flow_control", flow_control, error_message))
    {
        return false;
    }

    config.baud_rate = baud_rate;
    switch (data_bits)
    {
    case 5:
        config.data_bits = QSerialPort::Data5;
        break;
    case 6:
        config.data_bits = QSerialPort::Data6;
        break;
    case 7:
        config.data_bits = QSerialPort::Data7;
        break;
    case 8:
        config.data_bits = QSerialPort::Data8;
        break;
    default:
        return jsonError(error_message, "connection.telemetry_serial.data_bits 只能是 5、6、7 或 8");
    }

    return parityFromString(parity, config.parity, error_message) &&
           stopBitsFromJsonNumber(stop_bits, config.stop_bits, error_message) &&
           flowControlFromString(flow_control, config.flow_control, error_message);
}

QString connectionModeToString(ConnectionMode mode)
{
    switch (mode)
    {
    case ConnectionMode::Wifi:
        return "wifi";
    case ConnectionMode::Telemetry:
        return "telemetry";
    }
    return "unknown";
}

bool connectionModeFromString(const QString &text,
                              ConnectionMode &mode,
                              QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "wifi")
    {
        mode = ConnectionMode::Wifi;
        return true;
    }
    if (value == "telemetry")
    {
        mode = ConnectionMode::Telemetry;
        return true;
    }
    return jsonError(
        error_message,
        "connection.mode 只能是 wifi 或 telemetry");
}

QJsonObject connectionConfigToJson(const ConnectionConfig &config)
{
    QJsonObject object;
    object.insert("mode", connectionModeToString(config.mode));
    object.insert(
        "telemetry_serial",
        serialConfigToJson(config.telemetry_serial));
    return object;
}

bool connectionConfigFromJson(const QJsonObject &object,
                              ConnectionConfig &config,
                              QString *error_message)
{
    QString mode_text;
    QJsonObject serial_object;
    if (!readString(object, "mode", mode_text, error_message) ||
        !readObject(
            object, "telemetry_serial", serial_object, error_message))
    {
        return false;
    }

    return connectionModeFromString(
               mode_text, config.mode, error_message) &&
           serialConfigFromJson(
               serial_object,
               config.telemetry_serial,
               error_message);
}

/************************************************************/

/***********************飞行任务相关函数**********************/

QJsonObject missionConfigToJson(const MissionConfig &config)
{
    QJsonObject object;
    object.insert("takeoff_altitude", config.takeoff_altitude);
    object.insert("move_altitude", config.move_altitude);
    object.insert("start_altitude", config.start_altitude);
    object.insert("yaw", config.yaw);
    object.insert("tolerance", config.tolerance);
    object.insert("yaw_tolerance_deg", config.yaw_tolerance_deg);
    object.insert("max_xy_speed_mps", config.max_xy_speed_mps);
    object.insert("max_z_speed_mps", config.max_z_speed_mps);
    object.insert("max_yaw_rate_deg_s", config.max_yaw_rate_deg_s);
    object.insert("takeoff_hover_duration", config.takeoff_hover_duration);
    object.insert("landing_hover_duration", config.landing_hover_duration);
    object.insert("move_hover_duration", config.move_hover_duration);
    object.insert("add_hover_between_takeoff", config.add_hover_between_takeoff);
    object.insert("add_hover_between_landing", config.add_hover_between_landing);
    object.insert("add_hover_between_moves", config.add_hover_between_moves);
    object.insert("auto_start_mission", config.auto_start_mission);
    object.insert("compress_waypoint_segments", config.compress_waypoint_segments);
    object.insert("compress_non_waypoint_segments", config.compress_non_waypoint_segments);
    object.insert("frame", config.frame);
    return object;
}

bool missionConfigFromJson(const QJsonObject &object,
                           MissionConfig &config,
                           QString *error_message)
{
    const bool common_valid =
        readDouble(object, "takeoff_altitude", config.takeoff_altitude, error_message) &&
        readDouble(object, "move_altitude", config.move_altitude, error_message) &&
        readDouble(object, "start_altitude", config.start_altitude, error_message) &&
        readDouble(object, "yaw", config.yaw, error_message) &&
        readDouble(object, "tolerance", config.tolerance, error_message) &&
        readDouble(object, "takeoff_hover_duration", config.takeoff_hover_duration, error_message) &&
        readDouble(object, "landing_hover_duration", config.landing_hover_duration, error_message) &&
        readDouble(object, "move_hover_duration", config.move_hover_duration, error_message) &&
        readBool(object, "add_hover_between_takeoff", config.add_hover_between_takeoff, error_message) &&
        readBool(object, "add_hover_between_landing", config.add_hover_between_landing, error_message) &&
        readBool(object, "add_hover_between_moves", config.add_hover_between_moves, error_message) &&
        readBool(object, "auto_start_mission", config.auto_start_mission, error_message) &&
        readBool(object, "compress_waypoint_segments", config.compress_waypoint_segments, error_message) &&
        readBool(object, "compress_non_waypoint_segments", config.compress_non_waypoint_segments, error_message) &&
        readString(object, "frame", config.frame, error_message);
    if (!common_valid)
    {
        return false;
    }

    // Versions 1 and 2 did not contain the four global move limits.
    if (!object.contains("yaw_tolerance_deg"))
    {
        return true;
    }
    return readDouble(object, "yaw_tolerance_deg", config.yaw_tolerance_deg, error_message) &&
           readDouble(object, "max_xy_speed_mps", config.max_xy_speed_mps, error_message) &&
           readDouble(object, "max_z_speed_mps", config.max_z_speed_mps, error_message) &&
           readDouble(object, "max_yaw_rate_deg_s", config.max_yaw_rate_deg_s, error_message);
}

/************************************************************/

/***********************相机伺服相关函数**********************/

QJsonObject visualServoConfigToJson(const VisualServoConfig &config)
{
    QJsonObject object;
    object.insert("enabled", config.enabled);
    object.insert("target_id", config.target_id);
    object.insert("require_confirmed", config.require_confirmed);
    object.insert("image_x_axis", config.image_x_axis);
    object.insert("image_y_axis", config.image_y_axis);
    object.insert("image_x_sign", config.image_x_sign);
    object.insert("image_y_sign", config.image_y_sign);
    object.insert("kp_x", config.kp_x);
    object.insert("ki_x", config.ki_x);
    object.insert("kd_x", config.kd_x);
    object.insert("kp_y", config.kp_y);
    object.insert("ki_y", config.ki_y);
    object.insert("kd_y", config.kd_y);
    object.insert("integral_limit", config.integral_limit);
    object.insert("filter_alpha", config.filter_alpha);
    object.insert("enter_tolerance_x", config.enter_tolerance_x);
    object.insert("enter_tolerance_y", config.enter_tolerance_y);
    object.insert("exit_tolerance_x", config.exit_tolerance_x);
    object.insert("exit_tolerance_y", config.exit_tolerance_y);
    object.insert("settle_time_s", config.settle_time_s);
    object.insert("acquire_timeout_s", config.acquire_timeout_s);
    object.insert("lost_timeout_s", config.lost_timeout_s);
    object.insert("overall_timeout_s", config.overall_timeout_s);
    object.insert("max_body_speed_mps", config.max_body_speed_mps);
    object.insert("continue_on_timeout", config.continue_on_timeout);
    return object;
}

bool visualServoConfigFromJson(const QJsonObject &object,
                               VisualServoConfig &config,
                               QString *error_message)
{
    // Older warehouse_config.json files do not contain enabled. In that case,
    // keep the code default created by createDefaultWarehouseConfig().
    if (object.contains("enabled") &&
        !readBool(object, "enabled", config.enabled, error_message))
    {
        return false;
    }

    return readString(object, "target_id", config.target_id, error_message) &&
           readBool(object, "require_confirmed", config.require_confirmed, error_message) &&
           readString(object, "image_x_axis", config.image_x_axis, error_message) &&
           readString(object, "image_y_axis", config.image_y_axis, error_message) &&
           readDouble(object, "image_x_sign", config.image_x_sign, error_message) &&
           readDouble(object, "image_y_sign", config.image_y_sign, error_message) &&
           readDouble(object, "kp_x", config.kp_x, error_message) &&
           readDouble(object, "ki_x", config.ki_x, error_message) &&
           readDouble(object, "kd_x", config.kd_x, error_message) &&
           readDouble(object, "kp_y", config.kp_y, error_message) &&
           readDouble(object, "ki_y", config.ki_y, error_message) &&
           readDouble(object, "kd_y", config.kd_y, error_message) &&
           readDouble(object, "integral_limit", config.integral_limit, error_message) &&
           readDouble(object, "filter_alpha", config.filter_alpha, error_message) &&
           readDouble(object, "enter_tolerance_x", config.enter_tolerance_x, error_message) &&
           readDouble(object, "enter_tolerance_y", config.enter_tolerance_y, error_message) &&
           readDouble(object, "exit_tolerance_x", config.exit_tolerance_x, error_message) &&
           readDouble(object, "exit_tolerance_y", config.exit_tolerance_y, error_message) &&
           readDouble(object, "settle_time_s", config.settle_time_s, error_message) &&
           readDouble(object, "acquire_timeout_s", config.acquire_timeout_s, error_message) &&
           readDouble(object, "lost_timeout_s", config.lost_timeout_s, error_message) &&
           readDouble(object, "overall_timeout_s", config.overall_timeout_s, error_message) &&
           readDouble(object, "max_body_speed_mps", config.max_body_speed_mps, error_message) &&
           readBool(object, "continue_on_timeout", config.continue_on_timeout, error_message);
}

/************************************************************/

/***********************工业相机相关函数**********************/

QJsonObject industrialCameraConfigToJson(const IndustrialCameraConfig &config)
{
    QJsonObject object;
    object.insert("auto_exposure", config.auto_exposure);
    object.insert("exposure_absolute", config.exposure_absolute);
    object.insert("auto_exposure_priority", config.auto_exposure_priority);
    object.insert("gain", config.gain);
    object.insert("brightness", config.brightness);
    object.insert("contrast", config.contrast);
    object.insert("saturation", config.saturation);
    object.insert("gamma", config.gamma);
    object.insert("sharpness", config.sharpness);
    object.insert("backlight_compensation", config.backlight_compensation);
    object.insert("auto_white_balance", config.auto_white_balance);
    object.insert("white_balance_temperature", config.white_balance_temperature);
    object.insert("power_line_frequency", config.power_line_frequency);
    object.insert("auto_focus", config.auto_focus);
    object.insert("focus_absolute", config.focus_absolute);
    object.insert("zoom_absolute", config.zoom_absolute);
    return object;
}

bool industrialCameraConfigFromJson(const QJsonObject &object,
                                    IndustrialCameraConfig &config,
                                    QString *error_message)
{
    int power_line_frequency = 0;
    const bool valid =
        readBool(object, "auto_exposure", config.auto_exposure, error_message) &&
        readInt(object, "exposure_absolute", config.exposure_absolute, error_message) &&
        readBool(object, "auto_exposure_priority", config.auto_exposure_priority, error_message) &&
        readInt(object, "gain", config.gain, error_message) &&
        readInt(object, "brightness", config.brightness, error_message) &&
        readInt(object, "contrast", config.contrast, error_message) &&
        readInt(object, "saturation", config.saturation, error_message) &&
        readInt(object, "gamma", config.gamma, error_message) &&
        readInt(object, "sharpness", config.sharpness, error_message) &&
        readInt(object, "backlight_compensation", config.backlight_compensation, error_message) &&
        readBool(object, "auto_white_balance", config.auto_white_balance, error_message) &&
        readInt(object, "white_balance_temperature", config.white_balance_temperature, error_message) &&
        readInt(object, "power_line_frequency", power_line_frequency, error_message) &&
        readBool(object, "auto_focus", config.auto_focus, error_message) &&
        readInt(object, "focus_absolute", config.focus_absolute, error_message) &&
        readInt(object, "zoom_absolute", config.zoom_absolute, error_message);
    if (!valid || power_line_frequency < 0 || power_line_frequency > 255)
    {
        return valid ? jsonError(error_message, "power_line_frequency 超出 uint8 范围") : false;
    }
    config.power_line_frequency = static_cast<quint8>(power_line_frequency);
    return true;
}

/************************************************************/

/**************************JSON全部写*************************/

QJsonObject warehouseConfigToJson(const WarehouseConfig &config)
{
    QJsonArray shelves;
    for (const ShelfConfig &shelf : config.shelves)
    {
        QJsonObject rect;
        rect.insert("x", shelf.base_rect.x());
        rect.insert("y", shelf.base_rect.y());
        rect.insert("width", shelf.base_rect.width());
        rect.insert("length", shelf.base_rect.height());

        QJsonObject shelf_object;
        shelf_object.insert("code", shelf.code);
        shelf_object.insert("rows", shelf.rows);
        shelf_object.insert("columns", shelf.columns);
        shelf_object.insert("base_rect", rect);
        shelf_object.insert("height", shelf.height);
        shelf_object.insert("front_waypoint_y_m", shelf.front_waypoint_y_m);
        shelf_object.insert("back_waypoint_y_m", shelf.back_waypoint_y_m);
        shelf_object.insert("waypoint_row_z_m", doubleVectorToJson(shelf.waypoint_row_z_m));
        shelf_object.insert("waypoint_front_x_m", doubleVectorToJson(shelf.waypoint_front_x_m));
        shelf_object.insert("waypoint_back_x_m", doubleVectorToJson(shelf.waypoint_back_x_m));
        shelves.append(shelf_object);
    }

    QJsonObject slot_grid_object;
    slot_grid_object.insert("front_yaw_rad", config.slot_grid.front_yaw_rad);
    slot_grid_object.insert("back_yaw_rad", config.slot_grid.back_yaw_rad);

    QJsonObject root;
    root.insert("version", 6);
    root.insert("inspection_project",
                inspectionProjectToString(config.inspection_project));
    root.insert("shelf_count", config.shelves.size());
    root.insert("shelves", shelves);
    root.insert("slots", slot_grid_object);
    root.insert("ros", rosConfigToJson(config.ros));
    root.insert("bridge_ros", rosConfigToJson(config.bridge_ros));
    root.insert("connection", connectionConfigToJson(config.connection));
    root.insert("mission", missionConfigToJson(config.mission));
    root.insert("visual_servo", visualServoConfigToJson(config.visual_servo));
    root.insert("industrial_camera",
                industrialCameraConfigToJson(config.industrial_camera));
    return root;
}
bool warehouseConfigFromJson(const QJsonObject &root,
                             WarehouseConfig &config,
                             QString *error_message)
{
    int version = 0;
    if (!readInt(root, "version", version, error_message))
    {
        return false;
    }
    if (version < 1 || version > 6)
    {
        return jsonError(
            error_message,
            QString("不支持 warehouse_config.json 版本 %1").arg(version));
    }

    QJsonArray shelves_array;
    QJsonObject slots_object;
    QJsonObject ros_object;
    QJsonObject bridge_ros_object;
    QJsonObject mission_object;
    QJsonObject visual_servo_object;
    QJsonObject industrial_camera_object;
    QString inspection_project_text;
    if (version >= 5 &&
        !readString(root, "inspection_project",
                    inspection_project_text, error_message))
    {
        return false;
    }
    if (!readArray(root, "shelves", shelves_array, error_message) ||
        !readObject(root, "slots", slots_object, error_message) ||
        !readObject(root, "ros", ros_object, error_message) ||
        !readObject(root, "bridge_ros", bridge_ros_object, error_message) ||
        !readObject(root, "mission", mission_object, error_message))
    {
        return false;
    }
    if (version == 6)
    {
        int shelf_count = 0;
        if (!readInt(root, "shelf_count", shelf_count, error_message) ||
            shelf_count != shelves_array.size())
        {
            return jsonError(error_message, "shelf_count 与 shelves 数量不一致");
        }
    }

    if (version >= 3 &&
        !readObject(root, "visual_servo", visual_servo_object, error_message))
    {
        return false;
    }
    if (version >= 4 &&
        !readObject(root, "industrial_camera", industrial_camera_object, error_message))
    {
        return false;
    }

    // 版本 1 只有 bridge_serial，读取时自动迁移为默认 WiFi 模式。
    ConnectionConfig connection;
    if (version == 1)
    {
        QJsonObject legacy_serial_object;
        if (!readObject(root, "bridge_serial", legacy_serial_object, error_message) ||
            !serialConfigFromJson(legacy_serial_object,
                                  connection.telemetry_serial,
                                  error_message))
        {
            return false;
        }
        connection.mode = ConnectionMode::Wifi;
    }
    else
    {
        QJsonObject connection_object;
        if (!readObject(root, "connection", connection_object, error_message) ||
            !connectionConfigFromJson(connection_object, connection, error_message))
        {
            return false;
        }
    }

    WarehouseConfig loaded_config = createDefaultWarehouseConfig();
    SlotGridConfig slot_grid = loaded_config.slot_grid;
    if (!readDouble(slots_object, "front_yaw_rad",
                    slot_grid.front_yaw_rad, error_message) ||
        !readDouble(slots_object, "back_yaw_rad",
                    slot_grid.back_yaw_rad, error_message))
    {
        return false;
    }

    int legacy_rows = 0;
    int legacy_columns = 0;
    QVector<double> legacy_row_z;
    QVector<double> legacy_front_x;
    QVector<double> legacy_back_x;
    if (version <= 5)
    {
        if (!readInt(slots_object, "rows", legacy_rows, error_message) ||
            !readInt(slots_object, "columns", legacy_columns, error_message) ||
            !doubleVectorFromJson(slots_object, "waypoint_row_z_m",
                                  legacy_row_z, error_message) ||
            !doubleVectorFromJson(slots_object, "waypoint_front_x_m",
                                  legacy_front_x, error_message) ||
            !doubleVectorFromJson(slots_object, "waypoint_back_x_m",
                                  legacy_back_x, error_message) ||
            !readDouble(slots_object, "pose_y_min", slot_grid.pose_y_min, error_message) ||
            !readDouble(slots_object, "pose_y_max", slot_grid.pose_y_max, error_message) ||
            !readDouble(slots_object, "pose_z_min", slot_grid.pose_z_min, error_message) ||
            !readDouble(slots_object, "pose_z_max", slot_grid.pose_z_max, error_message))
        {
            return false;
        }
    }

    QVector<ShelfConfig> shelves;
    shelves.reserve(shelves_array.size());
    for (int shelf_index = 0; shelf_index < shelves_array.size(); ++shelf_index)
    {
        if (!shelves_array.at(shelf_index).isObject())
        {
            return jsonError(
                error_message,
                QString("shelves[%1] 必须是 JSON 对象").arg(shelf_index));
        }

        ShelfConfig shelf = shelf_index < loaded_config.shelves.size()
            ? loaded_config.shelves.at(shelf_index)
            : createDefaultShelfConfig(shelf_index);
        const QJsonObject shelf_object = shelves_array.at(shelf_index).toObject();
        QJsonObject rect_object;
        if (!readString(shelf_object, "code", shelf.code, error_message) ||
            !readObject(shelf_object, "base_rect", rect_object, error_message) ||
            !readDouble(shelf_object, "height", shelf.height, error_message) ||
            !readDouble(shelf_object, "front_waypoint_y_m",
                        shelf.front_waypoint_y_m, error_message) ||
            !readDouble(shelf_object, "back_waypoint_y_m",
                        shelf.back_waypoint_y_m, error_message))
        {
            return false;
        }

        double rect_x = 0.0;
        double rect_y = 0.0;
        double rect_width = 0.0;
        double rect_length = 0.0;
        if (!readDouble(rect_object, "x", rect_x, error_message) ||
            !readDouble(rect_object, "y", rect_y, error_message) ||
            !readDouble(rect_object, "width", rect_width, error_message) ||
            !readDouble(rect_object, "length", rect_length, error_message))
        {
            return false;
        }
        shelf.base_rect = QRectF(rect_x, rect_y, rect_width, rect_length);

        // Migrate only coordinates that still match the old automatic layout.
        // Manually adjusted shelf positions must remain unchanged.
        const int legacy_scene_column = shelf_index % 5;
        const int legacy_scene_row = shelf_index / 5;
        const double legacy_x = -90.0 + legacy_scene_column * 150.0;
        const double legacy_y = -100.0 + legacy_scene_row * 220.0;
        constexpr double coordinate_epsilon = 1e-6;
        if (std::abs(rect_x - legacy_x) <= coordinate_epsilon &&
            std::abs(rect_y - legacy_y) <= coordinate_epsilon &&
            std::abs(rect_width - 30.0) <= coordinate_epsilon &&
            std::abs(rect_length - 150.0) <= coordinate_epsilon)
        {
            shelf.base_rect.moveLeft(
                60.0 - legacy_scene_column * 150.0);
        }

        if (version == 6)
        {
            if (!readInt(shelf_object, "rows", shelf.rows, error_message) ||
                !readInt(shelf_object, "columns", shelf.columns, error_message) ||
                !doubleVectorFromJson(shelf_object, "waypoint_row_z_m",
                                      shelf.waypoint_row_z_m, error_message) ||
                !doubleVectorFromJson(shelf_object, "waypoint_front_x_m",
                                      shelf.waypoint_front_x_m, error_message) ||
                !doubleVectorFromJson(shelf_object, "waypoint_back_x_m",
                                      shelf.waypoint_back_x_m, error_message))
            {
                return false;
            }
            shelf.display_name = shelf.code;
            shelf.front_slot_prefix = shelf.code + "F";
            shelf.back_slot_prefix = shelf.code + "B";
        }
        else
        {
            QJsonArray pose_regions_array;
            QString scene_color;
            if (!readString(shelf_object, "display_name", shelf.display_name, error_message) ||
                !readString(shelf_object, "front_slot_prefix",
                            shelf.front_slot_prefix, error_message) ||
                !readString(shelf_object, "back_slot_prefix",
                            shelf.back_slot_prefix, error_message) ||
                !readString(shelf_object, "scene_color", scene_color, error_message) ||
                !readString(shelf_object, "button_status_color",
                            shelf.button_status_color, error_message) ||
                !readArray(shelf_object, "pose_regions",
                           pose_regions_array, error_message))
            {
                return false;
            }
            shelf.scene_color = QColor(scene_color);
            shelf.pose_regions.clear();
            for (int region_index = 0;
                 region_index < pose_regions_array.size();
                 ++region_index)
            {
                if (!pose_regions_array.at(region_index).isObject())
                {
                    return jsonError(
                        error_message,
                        QString("pose_regions[%1] 必须是 JSON 对象").arg(region_index));
                }
                const QJsonObject region_object =
                    pose_regions_array.at(region_index).toObject();
                ShelfPoseRegionConfig region;
                if (!readString(region_object, "side", region.side, error_message) ||
                    !readDouble(region_object, "x_min", region.x_min, error_message) ||
                    !readDouble(region_object, "x_max", region.x_max, error_message) ||
                    !readDouble(region_object, "yaw_min", region.yaw_min, error_message) ||
                    !readDouble(region_object, "yaw_max", region.yaw_max, error_message))
                {
                    return false;
                }
                shelf.pose_regions.push_back(region);
            }
            shelf.rows = legacy_rows;
            shelf.columns = legacy_columns;
            shelf.waypoint_row_z_m = legacy_row_z;
            shelf.waypoint_front_x_m = legacy_front_x;
            shelf.waypoint_back_x_m = legacy_back_x;
        }
        shelves.push_back(shelf);
    }

    loaded_config.shelves = shelves;
    loaded_config.slot_grid = slot_grid;
    loaded_config.connection = connection;
    // 版本 1-4 没有项目字段，沿用代码默认的 Cargo。
    if (version >= 5 &&
        !inspectionProjectFromString(
            inspection_project_text,
            loaded_config.inspection_project,
            error_message))
    {
        return false;
    }
    if (!rosConfigFromJson(ros_object, loaded_config.ros, error_message) ||
        !rosConfigFromJson(bridge_ros_object, loaded_config.bridge_ros, error_message) ||
        !missionConfigFromJson(mission_object, loaded_config.mission, error_message) ||
        (version >= 3 &&
         !visualServoConfigFromJson(
             visual_servo_object, loaded_config.visual_servo, error_message)) ||
        (version >= 4 &&
         !industrialCameraConfigFromJson(
             industrial_camera_object, loaded_config.industrial_camera, error_message)))
    {
        return false;
    }

    config = loaded_config;
    return true;
}
/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/

}

/************************************************************/
/*********************以下为提供外部函数***********************/

/*********************数传配置映射ros参数*********************/

void applyConnectionModeToRosConfig(
    WarehouseConfig &config,
    ConnectionMode mode)
{
    config.connection.mode = mode;

    if (mode == ConnectionMode::Telemetry)
    {
        // 数传时，地面站改为订阅/调用 bridge_ros 中的 /serial 接口。
        // 节点名仍属于地面站自身，不能替换成 ground_link_bridge。
        const QString ground_station_node_name = config.ros.node_name;
        config.ros = config.bridge_ros;
        config.ros.node_name = ground_station_node_name;
        return;
    }

    // WiFi 时恢复直连接口。只移除开头的 /serial，其他名称保持原样。
    auto remove_serial_prefix = [](const QString &name) {
        const QString prefix = "/serial";
        if (name == prefix)
        {
            return QString("/");
        }
        if (name.startsWith(prefix + "/"))
        {
            return name.mid(prefix.size());
        }
        return name;
    };

    config.ros.drone_status =
        remove_serial_prefix(config.ros.drone_status);
    config.ros.task_status =
        remove_serial_prefix(config.ros.task_status);
    config.ros.path_ready =
        remove_serial_prefix(config.ros.path_ready);
    config.ros.return_world_group =
        remove_serial_prefix(config.ros.return_world_group);
    config.ros.barcode_capture =
        remove_serial_prefix(config.ros.barcode_capture);
    config.ros.vision_barcode =
        remove_serial_prefix(config.ros.vision_barcode);
    config.ros.vision_servo_status =
        remove_serial_prefix(config.ros.vision_servo_status);
    config.ros.local_position =
        remove_serial_prefix(config.ros.local_position);
    config.ros.car_local_position =
        remove_serial_prefix(config.ros.car_local_position);
    config.ros.car_keypad_s4_pressed =
        remove_serial_prefix(config.ros.car_keypad_s4_pressed);
    config.ros.car_route_state =
        remove_serial_prefix(config.ros.car_route_state);
    config.ros.car_control_mode =
        remove_serial_prefix(config.ros.car_control_mode);
    config.ros.pose_delta =
        remove_serial_prefix(config.ros.pose_delta);
    config.ros.industrial_camera_params =
        remove_serial_prefix(config.ros.industrial_camera_params);
    config.ros.start_task_service =
        remove_serial_prefix(config.ros.start_task_service);
    config.ros.stop_push_service =
        remove_serial_prefix(config.ros.stop_push_service);
    config.ros.start_offboard_service =
        remove_serial_prefix(config.ros.start_offboard_service);
    config.ros.upload_mission_service =
        remove_serial_prefix(config.ros.upload_mission_service);
}

/************************************************************/

/***************************参数目录**************************/

QString warehouseDataDirectory()
{
    const QString configured_directory =
        qEnvironmentVariable("WAREHOUSE_GCS_DATA_DIR").trimmed();
    if (!configured_directory.isEmpty())
    {
        return QDir(configured_directory).absolutePath();
    }
    return QDir(QDir::homePath()).filePath("warehouse_gcs_data");
}

QString warehouseConfigFilePath()
{
    return QDir(warehouseDataDirectory()).filePath("warehouse_config.json");
}

QString shelfPanelDataFilePath()
{
    return QDir(warehouseDataDirectory()).filePath("shelf_panel_data.json");
}

/************************************************************/

/*****************JSONWarehouseConfig写入函数*****************/

bool saveWarehouseConfig(const WarehouseConfig &config, QString *error_message)
{
    const QString validation_error = validateWarehouseConfig(config);
    if (!validation_error.isEmpty())
    {
        return jsonError(
            error_message,
            "仓库配置校验失败：" + validation_error);
    }

    QDir data_directory(warehouseDataDirectory());
    if (!data_directory.exists() && !data_directory.mkpath("."))
    {
        return jsonError(
            error_message,
            "无法创建仓库数据目录：" + data_directory.absolutePath());
    }

    QSaveFile file(warehouseConfigFilePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        return jsonError(
            error_message,
            QString("无法打开配置文件 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }

    const QByteArray json_bytes =
        QJsonDocument(warehouseConfigToJson(config))
            .toJson(QJsonDocument::Indented);
    if (file.write(json_bytes) != json_bytes.size())
    {
        const QString write_error = file.errorString();
        file.cancelWriting();
        return jsonError(
            error_message,
            QString("配置文件写入不完整 %1：%2")
                .arg(file.fileName(), write_error));
    }

    if (!file.commit())
    {
        return jsonError(
            error_message,
            QString("配置文件提交失败 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }
    return true;
}

/************************************************************/

/*****************JSONWarehouseConfig读取函数*****************/

bool loadWarehouseConfig(WarehouseConfig &config, QString *error_message)
{
    config = createDefaultWarehouseConfig();

    QFile file(warehouseConfigFilePath());
    if (!file.exists())
    {
        return saveWarehouseConfig(config, error_message);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        return jsonError(
            error_message,
            QString("无法读取配置文件 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }

    QJsonParseError parse_error;
    const QJsonDocument document =
        QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError ||
        !document.isObject())
    {
        return jsonError(
            error_message,
            QString("配置 JSON 解析错误 %1：%2")
                .arg(file.fileName(), parse_error.errorString()));
    }

    // Start from current defaults so old JSON versions receive new settings.
    WarehouseConfig loaded_config = createDefaultWarehouseConfig();
    if (!warehouseConfigFromJson(
            document.object(), loaded_config, error_message))
    {
        if (error_message)
        {
            *error_message =
                QString("%1：%2").arg(file.fileName(), *error_message);
        }
        return false;
    }

    const QString validation_error =
        validateWarehouseConfig(loaded_config);
    if (!validation_error.isEmpty())
    {
        return jsonError(
            error_message,
            QString("%1：配置校验失败：%2")
                .arg(file.fileName(), validation_error));
    }

    config = loaded_config;
    return true;
}