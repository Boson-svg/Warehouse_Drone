#include "drone_warehouse/parameter_config_dialog.hpp"

#include <QButtonGroup>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollBar>
#include <QSet>
#include <QSlider>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QStyle>

#include <cmath>
#include <functional>
#include <utility>

namespace
{
// 参数页面的数据流：
// 1. 构造函数复制传入配置，original_config_ 用于丢弃，working_config_ 用于临时编辑。
// 2. parameterDefinitions() 集中描述所有参数，列表和编辑器都不再自己判断参数 ID。
// 3. rebuildParameterList() 遍历参数表，从 working_config_ 读取当前值并创建列表行。
// 4. 用户点击一行后，showParameterEditor() 根据参数表决定使用输入框还是下拉框。
// 5. 单项“确定”只更新 working_config_；页面顶部“启用”才校验并写入 JSON。
// 6. “丢弃”恢复打开窗口前的配置，“恢复默认值”只修改临时配置，仍需点击“启用”。
struct ProjectDefinition
{
    InspectionProject type; // 程序内部使用的项目枚举。
    QString name;           // 项目按钮显示文字。
};

const QVector<ProjectDefinition> projects = {
    // 项目按钮动态创建；以后增加巡检项目，只需要在这里增加一项。
    {InspectionProject::Cargo, "货物巡检"},
    {InspectionProject::Animal, "动物巡检"},
    {InspectionProject::Collaboration, "空地协同"}
};

struct ParameterFilterDefinition
{
    ParameterGroup group; // 筛选类别。
    QString name;         // 第二列按钮显示的英文名称。
};

const QVector<ParameterFilterDefinition> parameter_filters = {
    // 筛选按钮同样动态创建；显示顺序就是这里的排列顺序。
    {ParameterGroup::All, "ALL"},
    {ParameterGroup::Shelf, "SHELF"},
    {ParameterGroup::Flight, "FLIGHT"},
    {ParameterGroup::Servo, "SERVO"},
    {ParameterGroup::Camera, "CAMERA"}
};

// 决定右侧编辑器使用哪一种控件，以及如何格式化列表中的值。
enum class ParameterEditorType
{
    Text,    // 普通字符串，使用 QLineEdit。
    Number,  // 浮点数，使用 QLineEdit，列表中按小数位格式化。
    Integer, // 整数，使用 QLineEdit。
    Choice   // 布尔值或枚举，使用 QComboBox。
};

struct ParameterChoice
{
    QString text;  // 界面显示文字。
    QString value; // 实际写入配置的值。
};

// 一项 ParameterDefinition 对应参数列表中的一行。
// 它把原来分散在多个函数中的名称、说明、类型、读写方式集中到一起。
struct ParameterDefinition
{
    QString id;          // 唯一 ID，同时保存到 QListWidgetItem::UserRole。
    ParameterGroup group = ParameterGroup::All; // 所属筛选类别，与 ID 字符串无关。
    // id 只是参数页面内部使用的唯一查找键，不会靠拆分字符串来定位字段。
    // 真正的字段对应关系由下方 read/write 中保存的结构体成员指针决定。
    QString name;        // 参数列表和右侧编辑器显示的中文名称。
    QString description; // 参数用途、范围和单位说明。
    ParameterEditorType editor_type = ParameterEditorType::Text; // 编辑控件类型。
    QString unit;        // 只用于列表显示，例如 m、m/s、K。
    int display_decimals = 3; // 浮点数在列表中保留的小数位数。
    QString empty_display;    // 原始值为空时列表显示的替代文字。
    QString placeholder;      // 文本输入框没有内容时显示的提示。
    QVector<ParameterChoice> choices; // 下拉框的显示文字和实际值。

    // read 负责从 WarehouseConfig 中找到对应成员，并转换成不带单位的字符串。
    std::function<QString(const WarehouseConfig &)> read;
    // write 负责解析输入并写入候选配置；整体范围校验稍后统一进行。
    std::function<bool(WarehouseConfig &, const QString &, QString *)> write;
    // editable 为空表示始终可编辑；否则根据当前配置决定是否允许编辑。
    std::function<bool(const WarehouseConfig &)> editable;
    QString disabled_reason; // 不可编辑时追加到参数说明后的原因。
    bool use_integer_slider = false; // 相机整数参数可同时使用滑块和输入框。
    int slider_minimum = 0;
    int slider_maximum = 0;

    // 右侧输入框和保存逻辑使用原始值，不包含单位和中文选项名称。
    QString rawValue(const WarehouseConfig &config) const
    {
        return read ? read(config) : QString();
    }

    // 参数列表使用显示值：枚举转换为中文，数字补齐小数位，并追加单位。
    QString displayValue(const WarehouseConfig &config) const
    {
        const QString raw = rawValue(config);
        if (raw.isEmpty() && !empty_display.isEmpty())
        {
            return empty_display;
        }

        for (const ParameterChoice &choice : choices)
        {
            if (choice.value == raw)
            {
                return choice.text;
            }
        }

        QString value = raw;
        if (editor_type == ParameterEditorType::Number)
        {
            bool ok = false;
            const double number = raw.toDouble(&ok);
            if (ok)
            {
                value = QString::number(number, 'f', display_decimals);
            }
        }
        return unit.isEmpty() ? value : value + " " + unit;
    }
};

QVector<ParameterChoice> boolChoices()
{
    return {{"启用", "true"}, {"关闭", "false"}};
}

QVector<ParameterChoice> frameChoices()
{
    return {{"world_body", "world_body"}, {"body", "body"}, {"world_enu", "world_enu"}};
}

QVector<ParameterChoice> axisChoices()
{
    return {{"X 轴", "x"}, {"Y 轴", "y"}, {"Z 轴", "z"}};
}

QVector<ParameterChoice> signChoices()
{
    return {{"正向 (+1)", "1"}, {"反向 (-1)", "-1"}};
}

QVector<ParameterChoice> powerLineChoices()
{
    return {{"关闭", "0"}, {"50 Hz", "1"}, {"60 Hz", "2"}};
}

void setError(QString *error_message, const QString &message)
{
    if (error_message)
    {
        *error_message = message;
    }
}

// 参数类别根据 WarehouseConfig 中的实际结构体类型确定，而不是解析 ID 字符串。
template <typename Section>
ParameterGroup parameterGroupForSection()
{
    return ParameterGroup::All;
}

template <>
ParameterGroup parameterGroupForSection<MissionConfig>()
{
    return ParameterGroup::Flight;
}

template <>
ParameterGroup parameterGroupForSection<VisualServoConfig>()
{
    return ParameterGroup::Servo;
}

template <>
ParameterGroup parameterGroupForSection<IndustrialCameraConfig>()
{
    return ParameterGroup::Camera;
}

// 以下工厂函数用于生成不同数据类型的参数定义。
// section 指向 WarehouseConfig 中的分组，例如 mission；member 指向分组内的具体字段。
// 因此参数表可以直接写成员地址，不再为每个 ID 编写一套 if/else 读写代码。
template <typename Section>
ParameterDefinition doubleParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    const QString &unit,
    Section WarehouseConfig::*section,
    double Section::*member,
    int display_decimals = 3)
{
    ParameterDefinition definition;
    definition.id = id;
    definition.group = parameterGroupForSection<Section>();
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Number;
    definition.unit = unit;
    definition.display_decimals = display_decimals;
    definition.read = [section, member](const WarehouseConfig &config) {
        return QString::number((config.*section).*member, 'g', 15);
    };
    definition.write = [section, member](WarehouseConfig &config,
                                         const QString &raw,
                                         QString *error_message) {
        bool ok = false;
        const double value = raw.toDouble(&ok);
        if (!ok || !std::isfinite(value))
        {
            setError(error_message, "请输入有效有限数值");
            return false;
        }
        (config.*section).*member = value;
        return true;
    };
    return definition;
}

template <typename Section>
ParameterDefinition boolParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    Section WarehouseConfig::*section,
    bool Section::*member)
{
    ParameterDefinition definition;
    definition.id = id;
    definition.group = parameterGroupForSection<Section>();
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Choice;
    definition.choices = boolChoices();
    definition.read = [section, member](const WarehouseConfig &config) {
        return (config.*section).*member ? QString("true") : QString("false");
    };
    definition.write = [section, member](WarehouseConfig &config,
                                         const QString &raw,
                                         QString *error_message) {
        if (raw != "true" && raw != "false")
        {
            setError(error_message, "布尔选项无效");
            return false;
        }
        (config.*section).*member = raw == "true";
        return true;
    };
    return definition;
}

template <typename Section>
ParameterDefinition textParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    Section WarehouseConfig::*section,
    QString Section::*member,
    const QString &empty_display = {},
    const QString &placeholder = {})
{
    ParameterDefinition definition;
    definition.id = id;
    definition.group = parameterGroupForSection<Section>();
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Text;
    definition.empty_display = empty_display;
    definition.placeholder = placeholder;
    definition.read = [section, member](const WarehouseConfig &config) {
        return (config.*section).*member;
    };
    definition.write = [section, member](WarehouseConfig &config,
                                         const QString &raw,
                                         QString *) {
        (config.*section).*member = raw;
        return true;
    };
    return definition;
}

template <typename Section>
ParameterDefinition stringChoiceParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    Section WarehouseConfig::*section,
    QString Section::*member,
    const QVector<ParameterChoice> &choices)
{
    ParameterDefinition definition =
        textParameter(id, name, description, section, member);
    definition.editor_type = ParameterEditorType::Choice;
    definition.choices = choices;
    definition.write = [section, member, choices](WarehouseConfig &config,
                                                  const QString &raw,
                                                  QString *error_message) {
        for (const ParameterChoice &choice : choices)
        {
            if (choice.value == raw)
            {
                (config.*section).*member = raw;
                return true;
            }
        }
        setError(error_message, "选项无效");
        return false;
    };
    return definition;
}

template <typename Section>
ParameterDefinition integerParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    const QString &unit,
    Section WarehouseConfig::*section,
    int Section::*member)
{
    ParameterDefinition definition;
    definition.id = id;
    definition.group = parameterGroupForSection<Section>();
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Integer;
    definition.unit = unit;
    definition.read = [section, member](const WarehouseConfig &config) {
        return QString::number((config.*section).*member);
    };
    definition.write = [section, member](WarehouseConfig &config,
                                         const QString &raw,
                                         QString *error_message) {
        bool ok = false;
        const int value = raw.toInt(&ok);
        if (!ok)
        {
            setError(error_message, "请输入有效整数");
            return false;
        }
        (config.*section).*member = value;
        return true;
    };
    return definition;
}

template <typename Section>
ParameterDefinition doubleChoiceParameter(
    const QString &id,
    const QString &name,
    const QString &description,
    Section WarehouseConfig::*section,
    double Section::*member,
    const QVector<ParameterChoice> &choices)
{
    ParameterDefinition definition =
        doubleParameter(id, name, description, {}, section, member);
    definition.editor_type = ParameterEditorType::Choice;
    definition.choices = choices;
    definition.write = [section, member, choices](WarehouseConfig &config,
                                                  const QString &raw,
                                                  QString *error_message) {
        for (const ParameterChoice &choice : choices)
        {
            if (choice.value == raw)
            {
                (config.*section).*member = raw.toDouble();
                return true;
            }
        }
        setError(error_message, "方向选项无效");
        return false;
    };
    return definition;
}

ParameterDefinition powerLineParameter()
{
    ParameterDefinition definition;
    definition.id = "industrial_camera_power_line_frequency";
    definition.group = ParameterGroup::Camera;
    definition.name = "防闪烁";
    definition.description = "防止灯光引起画面闪烁：关闭、50 Hz 或 60 Hz。";
    definition.editor_type = ParameterEditorType::Choice;
    definition.choices = powerLineChoices();
    definition.read = [](const WarehouseConfig &config) {
        return QString::number(config.industrial_camera.power_line_frequency);
    };
    definition.write = [](WarehouseConfig &config,
                           const QString &raw,
                           QString *error_message) {
        bool ok = false;
        const int value = raw.toInt(&ok);
        if (!ok || value < 0 || value > 2)
        {
            setError(error_message, "防闪烁频率无效");
            return false;
        }
        config.industrial_camera.power_line_frequency = static_cast<quint8>(value);
        return true;
    };
    return definition;
}

// 给手动相机参数附加可编辑条件，例如自动曝光开启时禁止修改手动曝光。
ParameterDefinition disableWhileAutomatic(
    ParameterDefinition definition,
    std::function<bool(const WarehouseConfig &)> editable)
{
    definition.editable = std::move(editable);
    definition.disabled_reason = "当前对应的自动模式已启用，请先关闭自动模式。";
    return definition;
}

// 给相机整数参数附加滑块范围；输入框仍然保留，便于输入精确数值。
ParameterDefinition withIntegerSlider(
    ParameterDefinition definition,
    int minimum,
    int maximum)
{
    definition.use_integer_slider = true;
    definition.slider_minimum = minimum;
    definition.slider_maximum = maximum;
    return definition;
}

const QVector<ParameterDefinition> &baseParameterDefinitions()
{
    // 所有参数只在这里登记。列表、编辑器、默认值和写回逻辑都会读取此表。
    //
    // 新增参数时的步骤：
    // 1. 先在 warehouse_config.hpp 的对应结构体中增加字段，并设置默认值。
    // 2. 根据字段类型，在下面选择 doubleParameter、integerParameter、
    //    boolParameter、textParameter 或 stringChoiceParameter 登记一次。
    // 3. 在 validateWarehouseConfig() 中补充取值范围检查，并在 JSON 读写处加入字段。
    // ParameterConfigDialog 的其他函数不需要再修改。
    static const QVector<ParameterDefinition> definitions = {
        // 任务飞行参数。
        doubleParameter("takeoff_altitude", "起飞高度", "任务文件中的起飞高度，单位为米。", "m", &WarehouseConfig::mission, &MissionConfig::takeoff_altitude, 2),
        doubleParameter("move_altitude", "移动高度", "生成货架巡检航点时使用的默认飞行高度，单位为米。", "m", &WarehouseConfig::mission, &MissionConfig::move_altitude, 2),
        doubleParameter("start_altitude", "任务起始高度", "takeoff 动作的目标高度，单位为米。", "m", &WarehouseConfig::mission, &MissionConfig::start_altitude, 2),
        doubleParameter("yaw", "任务航向", "生成任务时使用的默认航向角，单位为弧度。", "rad", &WarehouseConfig::mission, &MissionConfig::yaw, 2),
        doubleParameter("tolerance", "航点容差", "move 动作判定位置到达时允许的距离误差，单位为米。", "m", &WarehouseConfig::mission, &MissionConfig::tolerance, 2),
        doubleParameter("yaw_tolerance_deg", "航向容差", "move 动作判定航向到达时允许的角度误差。", "deg", &WarehouseConfig::mission, &MissionConfig::yaw_tolerance_deg, 2),
        doubleParameter("max_xy_speed_mps", "最大水平速度", "move 动作在水平 X/Y 方向的最大速度。", "m/s", &WarehouseConfig::mission, &MissionConfig::max_xy_speed_mps, 2),
        doubleParameter("max_z_speed_mps", "最大垂直速度", "move 动作在竖直 Z 方向的最大速度。", "m/s", &WarehouseConfig::mission, &MissionConfig::max_z_speed_mps, 2),
        doubleParameter("max_yaw_rate_deg_s", "最大航向速度", "move 动作允许的最大航向角速度。", "deg/s", &WarehouseConfig::mission, &MissionConfig::max_yaw_rate_deg_s, 2),
        doubleParameter("takeoff_hover_duration", "起飞悬停", "起飞后插入悬停动作时的持续时间。", "s", &WarehouseConfig::mission, &MissionConfig::takeoff_hover_duration, 2),
        doubleParameter("landing_hover_duration", "降落悬停", "降落前插入悬停动作时的持续时间。", "s", &WarehouseConfig::mission, &MissionConfig::landing_hover_duration, 2),
        doubleParameter("move_hover_duration", "移动悬停", "每个移动动作后插入悬停时的持续时间。", "s", &WarehouseConfig::mission, &MissionConfig::move_hover_duration, 2),
        boolParameter("add_hover_between_takeoff", "起飞悬停开关", "决定是否在起飞动作后加入悬停。", &WarehouseConfig::mission, &MissionConfig::add_hover_between_takeoff),
        boolParameter("add_hover_between_landing", "降落悬停开关", "决定是否在降落动作前加入悬停。", &WarehouseConfig::mission, &MissionConfig::add_hover_between_landing),
        boolParameter("add_hover_between_moves", "移动悬停开关", "决定是否在移动动作之间加入悬停。", &WarehouseConfig::mission, &MissionConfig::add_hover_between_moves),
        boolParameter("auto_start_mission", "自动启动", "任务上传成功后是否自动开始执行。", &WarehouseConfig::mission, &MissionConfig::auto_start_mission),
        boolParameter("compress_waypoint_segments", "压缩货架航点", "是否压缩货架航点中的连续共线段。", &WarehouseConfig::mission, &MissionConfig::compress_waypoint_segments),
        boolParameter("compress_non_waypoint_segments", "压缩其他航点", "是否压缩非货架航点中的连续共线段。", &WarehouseConfig::mission, &MissionConfig::compress_non_waypoint_segments),
        stringChoiceParameter("frame", "任务坐标系", "任务航点使用的参考坐标系。", &WarehouseConfig::mission, &MissionConfig::frame, frameChoices()),

        // 视觉伺服参数。
        boolParameter("visual_servo_enabled", "视觉伺服开关", "决定是否启用相机伺服", &WarehouseConfig::visual_servo, &VisualServoConfig::enabled),
        textParameter("visual_servo_target_id", "视觉目标 ID", "指定需要跟踪的视觉目标 ID；留空时锁定首个符合条件的目标。", &WarehouseConfig::visual_servo, &VisualServoConfig::target_id, "自动锁定", "留空时自动锁定首个目标"),
        boolParameter("visual_servo_require_confirmed", "稳定确认", "启用后只接受视觉端已稳定确认的目标。", &WarehouseConfig::visual_servo, &VisualServoConfig::require_confirmed),
        stringChoiceParameter("visual_servo_image_x_axis", "X 映射轴", "图像水平误差映射到无人机机体系的哪个轴。", &WarehouseConfig::visual_servo, &VisualServoConfig::image_x_axis, axisChoices()),
        stringChoiceParameter("visual_servo_image_y_axis", "Y 映射轴", "图像垂直误差映射到无人机机体系的哪个轴，不能与图像 X 映射轴相同。", &WarehouseConfig::visual_servo, &VisualServoConfig::image_y_axis, axisChoices()),
        doubleChoiceParameter("visual_servo_image_x_sign", "X 方向", "图像水平误差映射到机体运动方向时使用的正负号。", &WarehouseConfig::visual_servo, &VisualServoConfig::image_x_sign, signChoices()),
        doubleChoiceParameter("visual_servo_image_y_sign", "Y 方向", "图像垂直误差映射到机体运动方向时使用的正负号。", &WarehouseConfig::visual_servo, &VisualServoConfig::image_y_sign, signChoices()),
        doubleParameter("visual_servo_kp_x", "X 比例", "图像 X 误差的 PID 比例增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::kp_x),
        doubleParameter("visual_servo_ki_x", "X 积分", "图像 X 误差的 PID 积分增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::ki_x),
        doubleParameter("visual_servo_kd_x", "X 微分", "图像 X 误差的 PID 微分增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::kd_x),
        doubleParameter("visual_servo_kp_y", "Y 比例", "图像 Y 误差的 PID 比例增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::kp_y),
        doubleParameter("visual_servo_ki_y", "Y 积分", "图像 Y 误差的 PID 积分增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::ki_y),
        doubleParameter("visual_servo_kd_y", "Y 微分", "图像 Y 误差的 PID 微分增益。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::kd_y),
        doubleParameter("visual_servo_integral_limit", "积分上限", "两轴积分累计量的绝对值上限，用于防止积分饱和。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::integral_limit),
        doubleParameter("visual_servo_filter_alpha", "滤波系数", "误差低通滤波系数，范围 0 到 1；越大响应越快。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::filter_alpha),
        doubleParameter("visual_servo_enter_tolerance_x", "X 进入容差", "图像 X 误差进入对准状态的阈值。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::enter_tolerance_x),
        doubleParameter("visual_servo_enter_tolerance_y", "Y 进入容差", "图像 Y 误差进入对准状态的阈值。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::enter_tolerance_y),
        doubleParameter("visual_servo_exit_tolerance_x", "X 退出容差", "已对准后图像 X 误差退出对准状态的阈值。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::exit_tolerance_x),
        doubleParameter("visual_servo_exit_tolerance_y", "Y 退出容差", "已对准后图像 Y 误差退出对准状态的阈值。", {}, &WarehouseConfig::visual_servo, &VisualServoConfig::exit_tolerance_y),
        doubleParameter("visual_servo_settle_time_s", "稳定时间", "误差持续位于对准范围内多久后判定成功。", "s", &WarehouseConfig::visual_servo, &VisualServoConfig::settle_time_s),
        doubleParameter("visual_servo_acquire_timeout_s", "获取超时", "动作开始后等待首个有效目标的最长时间。", "s", &WarehouseConfig::visual_servo, &VisualServoConfig::acquire_timeout_s),
        doubleParameter("visual_servo_lost_timeout_s", "丢失超时", "跟踪过程中允许目标连续丢失的最长时间。", "s", &WarehouseConfig::visual_servo, &VisualServoConfig::lost_timeout_s),
        doubleParameter("visual_servo_overall_timeout_s", "总超时", "单次视觉伺服动作允许执行的总时长。", "s", &WarehouseConfig::visual_servo, &VisualServoConfig::overall_timeout_s),
        doubleParameter("visual_servo_max_body_speed_mps", "最大机体速度", "视觉 PID 输出的单轴机体系速度上限。", "m/s", &WarehouseConfig::visual_servo, &VisualServoConfig::max_body_speed_mps),
        boolParameter("visual_servo_continue_on_timeout", "超时继续", "视觉伺服超时后是否继续执行后续任务动作。", &WarehouseConfig::visual_servo, &VisualServoConfig::continue_on_timeout),

        // 工业相机参数。
        boolParameter("industrial_camera_auto_exposure", "自动曝光", "自动曝光开关；启用时手动曝光时间不会写入相机。", &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::auto_exposure),
        disableWhileAutomatic(withIntegerSlider(integerParameter("industrial_camera_exposure_absolute", "曝光时间", "手动曝光时间，范围 1 到 10000。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::exposure_absolute), 1, 10000), [](const WarehouseConfig &config) { return !config.industrial_camera.auto_exposure; }),
        boolParameter("industrial_camera_auto_exposure_priority", "曝光优先", "自动曝光时是否允许降低帧率来提高画面亮度。", &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::auto_exposure_priority),
        withIntegerSlider(integerParameter("industrial_camera_gain", "增益", "图像增益，范围 0 到 190；过高会增加噪点。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::gain), 0, 190),
        withIntegerSlider(integerParameter("industrial_camera_brightness", "亮度", "图像亮度处理值，范围 0 到 255。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::brightness), 0, 255),
        withIntegerSlider(integerParameter("industrial_camera_contrast", "对比度", "图像对比度，范围 0 到 128。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::contrast), 0, 128),
        withIntegerSlider(integerParameter("industrial_camera_saturation", "饱和度", "图像色彩饱和度，范围 0 到 128。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::saturation), 0, 128),
        withIntegerSlider(integerParameter("industrial_camera_gamma", "Gamma", "图像中间亮度校正值，范围 0 到 255。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::gamma), 0, 255),
        withIntegerSlider(integerParameter("industrial_camera_sharpness", "锐度", "图像锐化强度，范围 0 到 255。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::sharpness), 0, 255),
        withIntegerSlider(integerParameter("industrial_camera_backlight_compensation", "逆光补偿", "逆光补偿值，范围 16 到 160。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::backlight_compensation), 16, 160),
        boolParameter("industrial_camera_auto_white_balance", "自动白平衡", "自动白平衡开关；启用时手动色温不会写入相机。", &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::auto_white_balance),
        disableWhileAutomatic(withIntegerSlider(integerParameter("industrial_camera_white_balance_temperature", "白平衡色温", "手动白平衡色温，范围 2800 到 6500 K。", "K", &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::white_balance_temperature), 2800, 6500), [](const WarehouseConfig &config) { return !config.industrial_camera.auto_white_balance; }),
        powerLineParameter(),
        boolParameter("industrial_camera_auto_focus", "自动对焦", "自动对焦开关；启用时手动焦点不会写入相机。", &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::auto_focus),
        disableWhileAutomatic(withIntegerSlider(integerParameter("industrial_camera_focus_absolute", "焦点", "手动焦点位置，范围 0 到 1023。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::focus_absolute), 0, 1023), [](const WarehouseConfig &config) { return !config.industrial_camera.auto_focus; }),
        withIntegerSlider(integerParameter("industrial_camera_zoom_absolute", "变焦", "相机变焦值，范围 100 到 200。", {}, &WarehouseConfig::industrial_camera, &IndustrialCameraConfig::zoom_absolute), 100, 200)
    };
    return definitions;
}

// SHELF 参数按当前货架数量、每个货架自己的行列数动态生成。
void resizeCoordinateVector(QVector<double> &values, int new_size, double fallback)
{
    const int old_size = values.size();
    const double fill_value = values.isEmpty() ? fallback : values.constLast();
    values.resize(new_size);
    for (int index = old_size; index < new_size; ++index)
    {
        values[index] = fill_value;
    }
}

ParameterDefinition shelfCountParameter()
{
    ParameterDefinition definition;
    definition.id = "shelf_count";
    definition.group = ParameterGroup::Shelf;
    definition.name = "货架数量";
    definition.description = "选择仓库当前使用的货架数量；增加时自动创建完整默认货架，减少时移除末尾货架。";
    definition.editor_type = ParameterEditorType::Choice;
    for (int count = 1; count <= 20; ++count)
    {
        definition.choices.push_back({QString::number(count), QString::number(count)});
    }
    definition.read = [](const WarehouseConfig &config) {
        return QString::number(config.shelves.size());
    };
    definition.write = [](WarehouseConfig &config,
                          const QString &raw,
                          QString *error_message) {
        bool ok = false;
        const int count = raw.toInt(&ok);
        if (!ok || count < 1 || count > 20)
        {
            setError(error_message, "货架数量必须在 1 到 20 之间");
            return false;
        }
        if (count < config.shelves.size())
        {
            config.shelves.resize(count);
            return true;
        }

        QSet<QString> used_codes;
        for (const ShelfConfig &shelf : config.shelves)
        {
            used_codes.insert(shelf.code.trimmed().toUpper());
        }
        while (config.shelves.size() < count)
        {
            const int shelf_index = config.shelves.size();
            ShelfConfig shelf = createDefaultShelfConfig(shelf_index);
            for (int code_index = 1; code_index <= 20; ++code_index)
            {
                const QString candidate =
                    QString("A%1").arg(code_index, 2, 10, QLatin1Char('0'));
                if (!used_codes.contains(candidate))
                {
                    shelf.code = candidate;
                    shelf.display_name = candidate;
                    shelf.front_slot_prefix = candidate + "F";
                    shelf.back_slot_prefix = candidate + "B";
                    used_codes.insert(candidate);
                    break;
                }
            }
            config.shelves.push_back(shelf);
        }
        return true;
    };
    return definition;
}

ParameterDefinition globalYawParameter(bool front)
{
    ParameterDefinition definition;
    definition.id = front ? "shelf_front_yaw_rad" : "shelf_back_yaw_rad";
    definition.group = ParameterGroup::Shelf;
    definition.name = front ? "正面航向" : "背面航向";
    definition.description = front
        ? "所有货架正面槽位生成航点时使用的无人机航向。"
        : "所有货架背面槽位生成航点时使用的无人机航向。";
    definition.editor_type = ParameterEditorType::Number;
    definition.unit = "rad";
    definition.display_decimals = 3;
    definition.read = [front](const WarehouseConfig &config) {
        return QString::number(front ? config.slot_grid.front_yaw_rad
                                     : config.slot_grid.back_yaw_rad,
                               'g', 15);
    };
    definition.write = [front](WarehouseConfig &config,
                               const QString &raw,
                               QString *error_message) {
        bool ok = false;
        const double value = raw.toDouble(&ok);
        if (!ok || !std::isfinite(value))
        {
            setError(error_message, "请输入有效有限数值");
            return false;
        }
        if (front)
        {
            config.slot_grid.front_yaw_rad = value;
        }
        else
        {
            config.slot_grid.back_yaw_rad = value;
        }
        return true;
    };
    return definition;
}

ParameterDefinition shelfCodeParameter(int shelf_index, const QString &prefix)
{
    ParameterDefinition definition;
    definition.id = QString("shelf_%1_code").arg(shelf_index + 1);
    definition.group = ParameterGroup::Shelf;
    definition.name = prefix + " 货架名称";
    definition.description = "货架名称只能从 A01 到 A20 中选择，并且不能与其他货架重复。";
    definition.editor_type = ParameterEditorType::Choice;
    for (int code_index = 1; code_index <= 20; ++code_index)
    {
        const QString code =
            QString("A%1").arg(code_index, 2, 10, QLatin1Char('0'));
        definition.choices.push_back({code, code});
    }
    definition.read = [shelf_index](const WarehouseConfig &config) {
        return shelf_index >= 0 && shelf_index < config.shelves.size()
            ? config.shelves.at(shelf_index).code
            : QString();
    };
    definition.write = [shelf_index](WarehouseConfig &config,
                                     const QString &raw,
                                     QString *error_message) {
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            setError(error_message, "货架索引已经失效，请重新选择参数");
            return false;
        }
        const QString code = raw.trimmed().toUpper();
        bool allowed = false;
        for (int code_index = 1; code_index <= 20; ++code_index)
        {
            if (code == QString("A%1").arg(
                            code_index, 2, 10, QLatin1Char('0')))
            {
                allowed = true;
                break;
            }
        }
        if (!allowed)
        {
            setError(error_message, "货架名称必须从 A01 到 A20 中选择");
            return false;
        }
        ShelfConfig &shelf = config.shelves[shelf_index];
        shelf.code = code;
        shelf.display_name = code;
        shelf.front_slot_prefix = code + "F";
        shelf.back_slot_prefix = code + "B";
        return true;
    };
    return definition;
}

ParameterDefinition shelfSizeParameter(
    int shelf_index,
    bool rows,
    const QString &prefix)
{
    ParameterDefinition definition;
    definition.id = QString("shelf_%1_%2")
                        .arg(shelf_index + 1)
                        .arg(rows ? "rows" : "columns");
    definition.group = ParameterGroup::Shelf;
    definition.name = prefix + (rows ? " 货架行数" : " 货架列数");
    definition.description = rows
        ? "当前货架每一面的槽位行数；修改后同步调整该货架的 R 行高数组。"
        : "当前货架每一面的槽位列数；修改后同步调整该货架正背面的 C 列 X 数组。";
    definition.editor_type = ParameterEditorType::Integer;
    definition.read = [shelf_index, rows](const WarehouseConfig &config) {
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            return QString();
        }
        const ShelfConfig &shelf = config.shelves.at(shelf_index);
        return QString::number(rows ? shelf.rows : shelf.columns);
    };
    definition.write = [shelf_index, rows](WarehouseConfig &config,
                                           const QString &raw,
                                           QString *error_message) {
        bool ok = false;
        const int value = raw.toInt(&ok);
        if (!ok || value <= 0 || value > 20)
        {
            setError(error_message, "货架行列数必须是 1 到 20 的整数");
            return false;
        }
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            setError(error_message, "货架索引已经失效，请重新选择参数");
            return false;
        }
        ShelfConfig &shelf = config.shelves[shelf_index];
        if (rows)
        {
            shelf.rows = value;
            resizeCoordinateVector(shelf.waypoint_row_z_m, value, 0.200);
        }
        else
        {
            shelf.columns = value;
            resizeCoordinateVector(shelf.waypoint_front_x_m, value, 0.0);
            resizeCoordinateVector(shelf.waypoint_back_x_m, value, 0.0);
        }
        return true;
    };
    return definition;
}

ParameterDefinition shelfDoubleParameter(
    int shelf_index,
    const QString &id_suffix,
    const QString &name,
    const QString &description,
    const QString &unit,
    const std::function<double(const ShelfConfig &)> &read_value,
    const std::function<void(ShelfConfig &, double)> &write_value,
    int display_decimals = 3)
{
    ParameterDefinition definition;
    definition.id = QString("shelf_%1_%2").arg(shelf_index + 1).arg(id_suffix);
    definition.group = ParameterGroup::Shelf;
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Number;
    definition.unit = unit;
    definition.display_decimals = display_decimals;
    definition.read = [shelf_index, read_value](const WarehouseConfig &config) {
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            return QString();
        }
        return QString::number(read_value(config.shelves.at(shelf_index)), 'g', 15);
    };
    definition.write = [shelf_index, write_value](WarehouseConfig &config,
                                                  const QString &raw,
                                                  QString *error_message) {
        bool ok = false;
        const double value = raw.toDouble(&ok);
        if (!ok || !std::isfinite(value))
        {
            setError(error_message, "请输入有效有限数值");
            return false;
        }
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            setError(error_message, "货架索引已经失效，请重新选择参数");
            return false;
        }
        write_value(config.shelves[shelf_index], value);
        return true;
    };
    return definition;
}

ParameterDefinition shelfVectorParameter(
    int shelf_index,
    int value_index,
    const QString &id_suffix,
    const QString &name,
    const QString &description,
    QVector<double> ShelfConfig::*member)
{
    ParameterDefinition definition;
    definition.id = QString("shelf_%1_%2").arg(shelf_index + 1).arg(id_suffix);
    definition.group = ParameterGroup::Shelf;
    definition.name = name;
    definition.description = description;
    definition.editor_type = ParameterEditorType::Number;
    definition.unit = "m";
    definition.display_decimals = 3;
    definition.read = [shelf_index, value_index, member](const WarehouseConfig &config) {
        if (shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            return QString();
        }
        const QVector<double> &values = config.shelves.at(shelf_index).*member;
        return value_index >= 0 && value_index < values.size()
            ? QString::number(values.at(value_index), 'g', 15)
            : QString();
    };
    definition.write = [shelf_index, value_index, member](
                           WarehouseConfig &config,
                           const QString &raw,
                           QString *error_message) {
        bool ok = false;
        const double value = raw.toDouble(&ok);
        if (!ok || !std::isfinite(value) ||
            shelf_index < 0 || shelf_index >= config.shelves.size())
        {
            setError(error_message, "货架坐标值或索引无效");
            return false;
        }
        QVector<double> &values = config.shelves[shelf_index].*member;
        if (value_index < 0 || value_index >= values.size())
        {
            setError(error_message, "货架坐标索引已经失效，请重新选择参数");
            return false;
        }
        values[value_index] = value;
        return true;
    };
    return definition;
}

QVector<ParameterDefinition> parameterDefinitions(const WarehouseConfig &config)
{
    QVector<ParameterDefinition> definitions;

    // SHELF 全局参数严格限制为货架数量、正面航向和背面航向。
    definitions.push_back(shelfCountParameter());
    definitions.push_back(globalYawParameter(true));
    definitions.push_back(globalYawParameter(false));

    for (int shelf_index = 0; shelf_index < config.shelves.size(); ++shelf_index)
    {
        const ShelfConfig &shelf = config.shelves.at(shelf_index);
        const QString prefix = shelf.code.isEmpty()
            ? QString("货架%1").arg(shelf_index + 1)
            : shelf.code;
        const auto shelf_name = [&prefix](const QString &field) {
            return prefix + " " + field;
        };

        definitions.push_back(shelfCodeParameter(shelf_index, prefix));
        definitions.push_back(shelfSizeParameter(shelf_index, true, prefix));
        definitions.push_back(shelfSizeParameter(shelf_index, false, prefix));

        for (int row = 0; row < shelf.waypoint_row_z_m.size(); ++row)
        {
            definitions.push_back(shelfVectorParameter(
                shelf_index, row,
                QString("row_%1_waypoint_z").arg(row + 1),
                shelf_name(QString("R%1 航点高度").arg(row + 1)),
                QString("%1 货架第 %2 行槽位生成航点时使用的 Z 高度。")
                    .arg(prefix).arg(row + 1),
                &ShelfConfig::waypoint_row_z_m));
        }
        for (int col = 0; col < shelf.waypoint_front_x_m.size(); ++col)
        {
            definitions.push_back(shelfVectorParameter(
                shelf_index, col,
                QString("front_col_%1_waypoint_x").arg(col + 1),
                shelf_name(QString("正面 C%1 航点 X").arg(col + 1)),
                QString("%1 货架正面第 %2 列槽位生成航点时使用的 X 坐标。")
                    .arg(prefix).arg(col + 1),
                &ShelfConfig::waypoint_front_x_m));
        }
        for (int col = 0; col < shelf.waypoint_back_x_m.size(); ++col)
        {
            definitions.push_back(shelfVectorParameter(
                shelf_index, col,
                QString("back_col_%1_waypoint_x").arg(col + 1),
                shelf_name(QString("背面 C%1 航点 X").arg(col + 1)),
                QString("%1 货架背面第 %2 列槽位生成航点时使用的 X 坐标。")
                    .arg(prefix).arg(col + 1),
                &ShelfConfig::waypoint_back_x_m));
        }

        definitions.push_back(shelfDoubleParameter(
            shelf_index, "base_x", shelf_name("货架 X"),
            "货架底面矩形左上角在货物巡检画板中的 X 坐标。", {},
            [](const ShelfConfig &value) { return value.base_rect.x(); },
            [](ShelfConfig &value, double number) { value.base_rect.moveLeft(number); }, 2));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "base_y", shelf_name("货架 Y"),
            "货架底面矩形左上角在货物巡检画板中的 Y 坐标。", {},
            [](const ShelfConfig &value) { return value.base_rect.y(); },
            [](ShelfConfig &value, double number) { value.base_rect.moveTop(number); }, 2));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "base_width", shelf_name("货架宽度"),
            "货架底面矩形在货物巡检画板中的宽度。", {},
            [](const ShelfConfig &value) { return value.base_rect.width(); },
            [](ShelfConfig &value, double number) { value.base_rect.setWidth(number); }, 2));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "base_length", shelf_name("货架长度"),
            "货架底面矩形在货物巡检画板中的长度。", {},
            [](const ShelfConfig &value) { return value.base_rect.height(); },
            [](ShelfConfig &value, double number) { value.base_rect.setHeight(number); }, 2));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "height", shelf_name("货架高度"),
            "货物巡检画板中货架的立体高度。", {},
            [](const ShelfConfig &value) { return value.height; },
            [](ShelfConfig &value, double number) { value.height = number; }, 2));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "front_waypoint_y_m", shelf_name("正面航点 Y"),
            "点击当前货架正面槽位时生成航点所使用的 Y 坐标。", "m",
            [](const ShelfConfig &value) { return value.front_waypoint_y_m; },
            [](ShelfConfig &value, double number) { value.front_waypoint_y_m = number; }));
        definitions.push_back(shelfDoubleParameter(
            shelf_index, "back_waypoint_y_m", shelf_name("背面航点 Y"),
            "点击当前货架背面槽位时生成航点所使用的 Y 坐标。", "m",
            [](const ShelfConfig &value) { return value.back_waypoint_y_m; },
            [](ShelfConfig &value, double number) { value.back_waypoint_y_m = number; }));
    }

    definitions += baseParameterDefinitions();
    return definitions;
}
// QListWidgetItem 只保存参数 ID。动态参数表每次按 working_config_ 重建，
// 因此这里复制找到的定义，不能返回临时 QVector 内元素的悬空指针。
bool findParameterDefinition(
    const QString &id,
    const WarehouseConfig &config,
    ParameterDefinition *result)
{
    for (const ParameterDefinition &definition : parameterDefinitions(config))
    {
        if (definition.id == id)
        {
            if (result)
            {
                *result = definition;
            }
            return true;
        }
    }
    return false;
}

}

ParameterConfigDialog::ParameterConfigDialog(
    const WarehouseConfig &config,
    QWidget *parent)
    : QDialog(parent),
      original_config_(config), // 永远代表打开窗口时的配置。
      working_config_(config),  // 页面内的所有修改先写入这份副本。
      selected_project_(config.inspection_project)
{
    setWindowTitle("参数设置");
    setWindowFlag(Qt::FramelessWindowHint, true);

    buildUi();

    // 进入页面默认显示参数设置。
    selectMainPage(1);
}

void ParameterConfigDialog::buildUi()
{
    // buildUi 只负责页面骨架和信号连接，参数内容由统一参数表负责。
    auto *root_layout = new QVBoxLayout(this);

    auto *top_layout = new QHBoxLayout;
    auto *page_title = new QLabel("智航参数", this);
    page_title->setObjectName("parameterPageTitle");
    top_layout->addWidget(page_title);
    top_layout->addStretch();

    restore_button_ = new QPushButton("恢复默认值", this);
    discard_button_ = new QPushButton("丢弃", this);
    apply_button_ = new QPushButton("启用", this);
    for (QPushButton *button : {restore_button_, discard_button_, apply_button_})
    {
        button->setMinimumSize(112, 44);
    }

    top_layout->addWidget(restore_button_);
    top_layout->addWidget(discard_button_);
    top_layout->addWidget(apply_button_);

    auto *body_layout = new QHBoxLayout;
    auto *navigation_panel = new QWidget(this);
    auto *navigation_layout = new QVBoxLayout(navigation_panel);
    navigation_layout->setContentsMargins(0, 0, 0, 0);
    navigation_layout->setSpacing(8);
    auto *navigation_top_layout = new QHBoxLayout;
    navigation_top_layout->setContentsMargins(0, 0, 0, 0);
    navigation_top_layout->setSpacing(8);


    auto *left_panel = new QWidget(this);
    auto *left_layout = new QVBoxLayout(left_panel);
    left_panel->setObjectName("leftPanel");

    project_page_button_ = new QPushButton("项目选择", left_panel);
    parameter_page_button_ = new QPushButton("参数设置", left_panel);
    close_page_button_ = new QPushButton("关闭", left_panel);
    for (QPushButton *button :
         {project_page_button_, parameter_page_button_, close_page_button_})
    {
        button->setMinimumSize(150, 52);
    }

    //设置按钮为可选中
    project_page_button_->setCheckable(true);
    parameter_page_button_->setCheckable(true);
    close_page_button_->setCheckable(true);

    main_page_button_group_ = new QButtonGroup(this);//主界面按钮组
    main_page_button_group_->setExclusive(true);//设置按钮为互斥模式
    main_page_button_group_->addButton(project_page_button_);
    main_page_button_group_->addButton(parameter_page_button_);
    main_page_button_group_->addButton(close_page_button_);

    left_layout->addWidget(project_page_button_);
    left_layout->addWidget(parameter_page_button_);
    left_layout->addWidget(close_page_button_);
    left_layout->addStretch();

    // 第二列是参数类别筛选栏，按钮由 parameter_filters 动态创建。
    parameter_filter_panel_ = new QWidget(this);
    parameter_filter_panel_->setObjectName("parameterFilterPanel");
    buildParameterFilterPanel();

    navigation_top_layout->addWidget(left_panel);
    navigation_top_layout->addWidget(parameter_filter_panel_);
    navigation_layout->addLayout(navigation_top_layout, 1);
    buildNumericKeypad(navigation_panel);
    navigation_layout->addWidget(numeric_keypad_);

    //主界面堆叠窗口，用于切换项目选择和参数编辑页面
    page_stack_ = new QStackedWidget(this);
    buildProjectPage();//构建项目选择界面
    buildParameterPage();//构建参数修改界面

    body_layout->addWidget(navigation_panel);
    body_layout->addWidget(page_stack_, 1);

    root_layout->addLayout(top_layout);
    root_layout->addLayout(body_layout, 1);

    connect(project_page_button_, &QPushButton::clicked,
            this, [this]() { selectMainPage(0); });

    connect(parameter_page_button_, &QPushButton::clicked,
            this, [this]() { selectMainPage(1); });

    connect(close_page_button_, &QPushButton::clicked,
            this, &QDialog::reject);

    // connect(parameter_page_button_, &QPushButton::clicked,
    //         this, [this]() {dialog.exec()});

    // 恢复默认值调用 createDefaultWarehouseConfig()，得到代码内固定默认值。
    // 它不是打开窗口前的值；打开窗口前的值只由丢弃按钮恢复。
    // 此操作只更新 working_config_，仍需点击启用才会写入 JSON。
    connect(restore_button_, &QPushButton::clicked,
        this, [this]() {
            const WarehouseConfig defaults =
                createDefaultWarehouseConfig();

            // 恢复范围由当前参数筛选类别决定；项目选择是独立设置，不在这里修改。
            switch (selected_parameter_group_)
            {
            case ParameterGroup::Shelf:
                working_config_.shelves = defaults.shelves;
                working_config_.slot_grid = defaults.slot_grid;
                break;
            case ParameterGroup::Flight:
                working_config_.mission = defaults.mission;
                break;
            case ParameterGroup::Servo:
                working_config_.visual_servo = defaults.visual_servo;
                break;
            case ParameterGroup::Camera:
                working_config_.industrial_camera = defaults.industrial_camera;
                break;
            case ParameterGroup::All:
                working_config_.shelves = defaults.shelves;
                working_config_.slot_grid = defaults.slot_grid;
                working_config_.mission = defaults.mission;
                working_config_.visual_servo = defaults.visual_servo;
                working_config_.industrial_camera = defaults.industrial_camera;
                break;
            }
            editing_parameter_id_.clear();
            parameter_editor_->hide();
            camera_slider_active_ = false;
            setNumericKeypadEnabled(false);
            rebuildParameterList(true);
        });

    // 丢弃时恢复原配置并直接关闭窗口。
    // 当前交互保留“恢复原配置”，但不再关闭参数窗口。
    connect(discard_button_, &QPushButton::clicked,
            this, [this]() {
                working_config_ = original_config_;
                editing_parameter_id_.clear();
                parameter_editor_->hide();
                camera_slider_active_ = false;
                setNumericKeypadEnabled(false);
                selectProject(working_config_.inspection_project);
            });

    // 启用时才执行最终校验和 JSON 保存。
    connect(apply_button_, &QPushButton::clicked,
            this, &ParameterConfigDialog::handleApply);

    setStyleSheet(
        "QDialog { background: #101722; color: #d7e3f4; }"
        "#parameterPageTitle { font-size: 24px; font-weight: 600; color: #f1f6fb; }"
        "#leftPanel, #parameterFilterPanel { background: #1b2736; border: 1px solid #34485e; border-radius: 6px; }"
        "#numericKeypad { background: #1b2736; border: 1px solid #34485e; border-radius: 6px; }"
        "QPushButton { min-height: 40px; padding: 0 16px; font-size: 17px;"
        "  color: #dce8f4; background: #26384b; border: 1px solid #47627d; border-radius: 6px; }"
        "QPushButton:hover { background: #31485f; }"
        "QPushButton:checked, QPushButton:pressed { color: #071018; background: #5bc0be; border-color: #76d4d1; }"
        "QListWidget { background: #111b27; border: 1px solid #30465c; font-size: 18px; }"
        "QListWidget::item:selected { background: #24485b; }"
        // 主窗口给 QWidget 设置了深色背景。这里显式把 QLabel 设为透明，
        // 避免参数说明、修改值和默认值文字后面出现一块黑色矩形。
        "QLabel { background: transparent; border: none; font-size: 17px; }"
        "#parameterEditor { background: #172332; border-left: 1px solid #40566d; }"
        "#parameterEditorTitle { font-size: 21px; font-weight: 600; color: #f1f6fb; }"
        "#parameterDescription { color: #b9c7d5; line-height: 1.4; }"
        "#parameterDefaultValue { color: #8fa3b7; padding-top: 8px; }"
        "#modifiedParameterValue { color: #ff6b6b; font-weight: 600; }"
        "#parameterEditorClose { padding: 0; font-size: 24px; }"
        "QLineEdit, QComboBox { min-height: 42px; padding: 0 10px; font-size: 22px;"
        "  color: #eef5fb; background: #0d1620; border: 1px solid #526d87; border-radius: 4px; }"
        "QLineEdit:disabled, QComboBox:disabled { color: #6f8294; background: #151d26; }"
        "QSlider#cameraParameterSlider::groove:horizontal { height: 8px;"
        "  background: #30465c; border-radius: 4px; }"
        "QSlider#cameraParameterSlider::sub-page:horizontal { background: #5bc0be; border-radius: 4px; }"
        "QSlider#cameraParameterSlider::handle:horizontal { width: 26px; height: 26px;"
        "  margin: -9px 0; background: #eef5fb; border: 2px solid #5bc0be; border-radius: 13px; }"
        "QSlider#cameraParameterSlider::handle:horizontal:disabled { background: #657789; border-color: #46596b; }"
    );
}

const WarehouseConfig &
ParameterConfigDialog::savedConfig() const
{
    return working_config_;
}

void ParameterConfigDialog::buildNumericKeypad(QWidget *parent)
{
    numeric_keypad_ = new QWidget(parent);
    numeric_keypad_->setObjectName("numericKeypad");
    numeric_keypad_->setFixedHeight(236);
    auto *layout = new QGridLayout(numeric_keypad_);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setHorizontalSpacing(8);
    layout->setVerticalSpacing(8);

    for (int digit = 1; digit <= 9; ++digit)
    {
        const QString text = QString::number(digit);
        auto *button = new QPushButton(text, numeric_keypad_);
        button->setMinimumSize(0, 48);
        connect(button, &QPushButton::clicked, this,
                [this, text]() { appendNumericDigit(text); });
        layout->addWidget(button, (digit - 1) / 3, (digit - 1) % 3);
    }

    auto *backspace_button = new QPushButton(numeric_keypad_);
    backspace_button->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
    backspace_button->setIconSize(QSize(24, 24));
    backspace_button->setToolTip("回退");
    connect(backspace_button, &QPushButton::clicked,
            this, &ParameterConfigDialog::backspaceNumericInput);

    auto *zero_button = new QPushButton("0", numeric_keypad_);
    connect(zero_button, &QPushButton::clicked, this,
            [this]() { appendNumericDigit("0"); });

    auto *decimal_button = new QPushButton(".", numeric_keypad_);
    decimal_button->setToolTip("小数点（仅浮点参数可用）");
    connect(decimal_button, &QPushButton::clicked, this,
            [this]() { appendNumericDigit("."); });

    for (QPushButton *button :
         {backspace_button, zero_button, decimal_button})
    {
        button->setMinimumSize(0, 48);
    }
    layout->addWidget(backspace_button, 3, 0);
    layout->addWidget(zero_button, 3, 1);
    layout->addWidget(decimal_button, 3, 2);
    setNumericKeypadEnabled(false);
}

void ParameterConfigDialog::setNumericKeypadEnabled(bool enabled)
{
    numeric_editor_active_ = enabled;
    if (numeric_keypad_)
    {
        numeric_keypad_->setEnabled(enabled);
    }
}

void ParameterConfigDialog::appendNumericDigit(const QString &digit)
{
    if (!numeric_editor_active_ || !editor_line_edit_ ||
        !editor_line_edit_->isEnabled())
    {
        return;
    }
    if (digit == ".")
    {
        ParameterDefinition definition;
        if (!findParameterDefinition(
                editing_parameter_id_, working_config_, &definition) ||
            definition.editor_type != ParameterEditorType::Number ||
            editor_line_edit_->text().contains('.'))
        {
            return;
        }
        if (editor_line_edit_->text().isEmpty())
        {
            editor_line_edit_->insert("0");
        }
    }
    editor_line_edit_->insert(digit);
    editor_line_edit_->setFocus();
}

void ParameterConfigDialog::backspaceNumericInput()
{
    if (!numeric_editor_active_ || !editor_line_edit_ ||
        !editor_line_edit_->isEnabled())
    {
        return;
    }
    editor_line_edit_->backspace();
    editor_line_edit_->setFocus();
}

void ParameterConfigDialog::selectMainPage(int page_index)
{
    project_page_button_->setChecked(page_index == 0);//设置按钮选中状态
    parameter_page_button_->setChecked(page_index == 1);
    page_stack_->setCurrentIndex(page_index);//切换堆叠窗口页面

    // 项目选择页面不需要参数筛选，进入参数页面后再显示第二列。
    parameter_filter_panel_->setVisible(page_index == 1);
    if (numeric_keypad_) {
        numeric_keypad_->setVisible(page_index == 1);
    }
}

void ParameterConfigDialog::buildParameterFilterPanel()
{
    auto *layout = new QVBoxLayout(parameter_filter_panel_);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(10);

    parameter_filter_button_group_ = new QButtonGroup(this);
    parameter_filter_button_group_->setExclusive(true);

    // 根据 parameter_filters 的内容和数量动态创建筛选按钮。
    for (const ParameterFilterDefinition &filter : parameter_filters)
    {
        auto *button = new QPushButton(filter.name, parameter_filter_panel_);
        button->setCheckable(true);
        button->setMinimumSize(128, 48);
        parameter_filter_button_group_->addButton(
            button, static_cast<int>(filter.group));

        connect(button, &QPushButton::clicked, this,
                [this, group = filter.group]() {
                    selectParameterGroup(group);
                });
        layout->addWidget(button);
    }

    layout->addStretch();
    selectParameterGroup(ParameterGroup::All);
}

void ParameterConfigDialog::selectParameterGroup(ParameterGroup group)
{
    selected_parameter_group_ = group;
    // 切换类别后，旧列表行已经不再有效，同时关闭旧参数的右侧编辑器。
    // 构建筛选栏时参数列表和编辑器尚未创建，因此这里需要先判断指针。
    editing_parameter_id_.clear();
    if (parameter_list_)
    {
        parameter_list_->clearSelection();
    }
    if (parameter_editor_)
    {
        parameter_editor_->hide();
    }
    camera_slider_active_ = false;
    setNumericKeypadEnabled(false);

    // QButtonGroup 使用枚举值作为按钮 ID，保证当前筛选按钮保持高亮。
    QAbstractButton *button = parameter_filter_button_group_->button(
        static_cast<int>(group));
    if (button)
    {
        button->setChecked(true);
    }

    // parameter_list_ 尚未创建时，此函数会安全返回；创建完成后会再次重建。
    rebuildParameterList();
}

void ParameterConfigDialog::buildProjectPage()
{
    project_page_ = new QWidget(page_stack_);

    auto *layout = new QVBoxLayout(project_page_);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(12);

    auto *title_label = new QLabel("巡检项目", project_page_);
    layout->addWidget(title_label);

    project_button_group_ = new QButtonGroup(this);
    project_button_group_->setExclusive(true);//设置按钮为互斥模式

    for (const ProjectDefinition &project : projects)
    {
        //循环创建按钮，创建按钮时写入属性
        auto *button =
            new QPushButton(project.name, project_page_);

        button->setCheckable(true);
        button->setMinimumHeight(48);

        //按钮组保证巡检项目始终只有一个处于高亮状态
        project_button_group_->addButton(
            button,
            static_cast<int>(project.type));

        connect(button, &QPushButton::clicked,
                this,
                [this, type = project.type]() {
                    selectProject(type);
                });

        layout->addWidget(button);
    }

    layout->addStretch();
    page_stack_->addWidget(project_page_);

    // 高亮 JSON 中保存的项目；旧配置由加载器默认迁移为 Cargo。
    selectProject(selected_project_);
}

void ParameterConfigDialog::selectProject(
    InspectionProject project)
{
    selected_project_ = project;
    // 项目选择和其他参数一样先写临时配置，点击“启用”后再保存到 JSON。
    working_config_.inspection_project = project;

    //按照当前选择的巡检项目高亮对应按钮
    QAbstractButton *button =
        project_button_group_->button(
            static_cast<int>(project));

    if (button)
    {
        button->setChecked(true);
    }

    // 当前货物和动物暂时显示相同参数，后续再拆分。
    rebuildParameterList();
}

void ParameterConfigDialog::buildParameterPage()
{
    //参数编辑页面
    parameter_page_ = new QWidget(page_stack_);

    auto *layout = new QHBoxLayout(parameter_page_);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    parameter_list_ = new QListWidget(parameter_page_);//参数列表
    parameter_list_->setFrameShape(QFrame::NoFrame);//边框去除
    parameter_list_->setSpacing(0);//行间距
    parameter_list_->setHorizontalScrollBarPolicy(//只允许纵向滑动
        Qt::ScrollBarAlwaysOff);

    parameter_editor_ = new QWidget(parameter_page_);//参数编辑器
    parameter_editor_->setObjectName("parameterEditor");
    parameter_editor_->setFixedWidth(420);

    auto *editor_layout = new QVBoxLayout(parameter_editor_);
    editor_layout->setContentsMargins(24, 20, 24, 24);
    editor_layout->setSpacing(16);

    auto *editor_header = new QHBoxLayout;
    editor_title_label_ = new QLabel(parameter_editor_);
    editor_title_label_->setObjectName("parameterEditorTitle");
    auto *editor_close_button = new QPushButton("×", parameter_editor_);
    editor_close_button->setObjectName("parameterEditorClose");
    editor_close_button->setFixedSize(40, 40);
    editor_header->addWidget(editor_title_label_, 1);
    editor_header->addWidget(editor_close_button);

    editor_description_label_ = new QLabel(parameter_editor_);
    editor_description_label_->setWordWrap(true);
    editor_description_label_->setObjectName("parameterDescription");

    auto *value_title = new QLabel("修改值", parameter_editor_);
    editor_input_stack_ = new QStackedWidget(parameter_editor_);
    // QStackedWidget 默认会沿纵向扩展。固定输入区域高度，避免它占满右侧编辑器中部。
    editor_input_stack_->setFixedHeight(48);
    editor_line_container_ = new QWidget(editor_input_stack_);
    auto *line_input_layout = new QHBoxLayout(editor_line_container_);
    line_input_layout->setContentsMargins(0, 0, 0, 0);
    line_input_layout->setSpacing(14);
    editor_camera_slider_ = new QSlider(Qt::Horizontal, editor_line_container_);
    editor_camera_slider_->setObjectName("cameraParameterSlider");
    editor_camera_slider_->setMinimumWidth(180);
    editor_line_edit_ = new QLineEdit(editor_line_container_);
    editor_line_edit_->setFixedHeight(46);
    line_input_layout->addWidget(editor_camera_slider_, 1);
    line_input_layout->addWidget(editor_line_edit_);
    editor_combo_box_ = new QComboBox(editor_input_stack_);
    editor_combo_box_->setFixedHeight(46);
    editor_input_stack_->addWidget(editor_line_container_);
    editor_input_stack_->addWidget(editor_combo_box_);

    editor_default_label_ = new QLabel(parameter_editor_);
    editor_default_label_->setObjectName("parameterDefaultValue");
    editor_default_label_->setWordWrap(true);

    editor_confirm_button_ = new QPushButton("确定", parameter_editor_);
    editor_confirm_button_->setMinimumHeight(44);

    editor_layout->addLayout(editor_header);
    editor_layout->addWidget(editor_description_label_);
    editor_layout->addSpacing(8);
    editor_layout->addWidget(value_title);
    editor_layout->addWidget(editor_input_stack_);
    editor_layout->addStretch();
    // 默认值固定显示在右侧底部区域，便于修改前随时对照。
    editor_layout->addWidget(editor_default_label_);
    editor_layout->addWidget(editor_confirm_button_);
    parameter_editor_->hide();//选择参数前不显示右侧编辑区域

    layout->addWidget(parameter_list_, 1);
    layout->addWidget(parameter_editor_);

    page_stack_->addWidget(parameter_page_);

    connect(parameter_list_, &QListWidget::itemClicked,
            this, &ParameterConfigDialog::showParameterEditor);
    connect(editor_close_button, &QPushButton::clicked,
            this, [this]() {
                editing_parameter_id_.clear();
                parameter_list_->clearSelection();
                parameter_editor_->hide();
                camera_slider_active_ = false;
                setNumericKeypadEnabled(false);
            });
    connect(editor_confirm_button_, &QPushButton::clicked,
            this, &ParameterConfigDialog::commitParameterEdit);
    connect(editor_line_edit_, &QLineEdit::returnPressed,
            this, &ParameterConfigDialog::commitParameterEdit);
    // 相机滑块和数值输入框双向同步；真正写入临时配置仍由“确定”触发。
    connect(editor_camera_slider_, &QSlider::valueChanged,
            this, [this](int value) {
                if (camera_slider_active_)
                {
                    editor_line_edit_->setText(QString::number(value));
                }
            });
    connect(editor_line_edit_, &QLineEdit::textChanged,
            this, [this](const QString &text) {
                if (!camera_slider_active_)
                {
                    return;
                }
                bool ok = false;
                const int value = text.toInt(&ok);
                if (ok && value >= editor_camera_slider_->minimum() &&
                    value <= editor_camera_slider_->maximum())
                {
                    editor_camera_slider_->setValue(value);
                }
            });

    rebuildParameterList();
}

void ParameterConfigDialog::rebuildParameterList(bool preserve_scroll_position)
{
    if (!parameter_list_)
    {
        return;
    }

    // 每次确认单项、恢复默认值或切换项目后都重建列表，确保右侧数值是最新的。
    // 手动确认单项时保存当前滚动值，避免列表重建后跳回第一行。
    const int previous_scroll_value = preserve_scroll_position
        ? parameter_list_->verticalScrollBar()->value()
        : 0;
    parameter_list_->clear();

    // 代码默认值每次都重新由 createDefaultWarehouseConfig() 构造，
    // 不读取 JSON，也不使用 original_config_，因此不会被用户保存的值覆盖。
    const WarehouseConfig defaults = createDefaultWarehouseConfig();
    WarehouseConfig extended_defaults = defaults;
    while (extended_defaults.shelves.size() < working_config_.shelves.size())
    {
        extended_defaults.shelves.push_back(
            createDefaultShelfConfig(extended_defaults.shelves.size()));
    }

    // 参数顺序、名称和显示格式全部来自统一参数表。
    for (const ParameterDefinition &definition : parameterDefinitions(working_config_))
    {
        if (selected_parameter_group_ != ParameterGroup::All &&
            definition.group != selected_parameter_group_)
        {
            continue;
        }

        // 货架数量的永久默认值始终是 2；新增货架的字段默认值则由
        // createDefaultShelfConfig(index) 提供，右侧仍能显示真实代码默认值。
        const WarehouseConfig &definition_defaults =
            definition.id == "shelf_count" ? defaults : extended_defaults;
        const bool differs_from_default =
            definition.rawValue(working_config_) !=
            definition.rawValue(definition_defaults);

        addParameterRow(
            definition.id,
            definition.name,
            definition.displayValue(working_config_),
            differs_from_default);
    }

    if (preserve_scroll_position)
    {
        parameter_list_->verticalScrollBar()->setValue(previous_scroll_value);
    }
}

void ParameterConfigDialog::addParameterRow(
    const QString &parameter_id,
    const QString &display_name,
    const QString &display_value,
    bool differs_from_default)
{
    // QListWidgetItem 保存参数身份，row_widget 只负责显示名称、数值和分割线。
    // 点击时通过 UserRole 取回 ID，再去统一参数表查询完整定义。
    auto *item = new QListWidgetItem(parameter_list_);
    item->setData(Qt::UserRole, parameter_id);
    item->setData(Qt::UserRole + 1, display_name);
    item->setFlags(
        Qt::ItemIsEnabled |
        Qt::ItemIsSelectable);
    item->setSizeHint(QSize(0, 58));

    auto *row_widget = new QWidget(parameter_list_);
    row_widget->setAttribute(
        Qt::WA_TransparentForMouseEvents);

    auto *row_layout = new QVBoxLayout(row_widget);
    row_layout->setContentsMargins(16, 0, 16, 0);
    row_layout->setSpacing(0);

    auto *value_layout = new QHBoxLayout;
    value_layout->setContentsMargins(0, 0, 0, 0);

    auto *name_label =
        new QLabel(display_name, row_widget);
    auto *value_label =
        new QLabel(display_value, row_widget);
    if (differs_from_default)
    {
        // 只把右侧数值标红，参数名称保持原颜色，方便快速定位已修改项。
        value_label->setObjectName("modifiedParameterValue");
    }

    value_label->setAlignment(
        Qt::AlignRight | Qt::AlignVCenter);

    value_layout->addWidget(name_label);
    value_layout->addStretch();
    value_layout->addWidget(value_label);

    auto *separator = new QFrame(row_widget);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Plain);

    row_layout->addLayout(value_layout, 1);
    row_layout->addWidget(separator);

    parameter_list_->setItemWidget(item, row_widget);
}

void ParameterConfigDialog::showParameterEditor(QListWidgetItem *item)
{
    if (!item)
    {
        return;
    }

    // 第一步：从列表行中取出参数 ID，并找到唯一的参数定义。
    editing_parameter_id_ = item->data(Qt::UserRole).toString();
    ParameterDefinition definition;
    if (!findParameterDefinition(
            editing_parameter_id_, working_config_, &definition))
    {
        editing_parameter_id_.clear();
        parameter_editor_->hide();
        camera_slider_active_ = false;
        setNumericKeypadEnabled(false);
        return;
    }

    const bool numeric_editor =
        definition.editor_type == ParameterEditorType::Number ||
        definition.editor_type == ParameterEditorType::Integer;
    camera_slider_active_ = false;
    // 第二步：分别读取临时配置中的当前值和程序默认值。
    // 这里使用 rawValue，因此不会把 m、s 等显示单位写进输入框。
    const QString current_value = definition.rawValue(working_config_);
    WarehouseConfig defaults = createDefaultWarehouseConfig();
    if (definition.id != "shelf_count")
    {
        while (defaults.shelves.size() < working_config_.shelves.size())
        {
            defaults.shelves.push_back(
                createDefaultShelfConfig(defaults.shelves.size()));
        }
    }
    const QString default_value = definition.displayValue(defaults);

    editor_title_label_->setText(definition.name);
    editor_description_label_->setText(definition.description);
    // 默认值使用与参数列表相同的格式，包含单位和中文枚举名称。
    editor_default_label_->setText(
        "默认值：" + (default_value.isEmpty() ? "空" : default_value));
    editor_confirm_button_->setEnabled(true);
    editor_line_edit_->setEnabled(true);
    editor_combo_box_->setEnabled(true);
    editor_camera_slider_->setEnabled(true);

    // 第三步：Choice 使用下拉框，其余类型使用文本输入框。
    if (definition.editor_type == ParameterEditorType::Choice)
    {
        editor_combo_box_->clear();
        for (const ParameterChoice &choice : definition.choices)
        {
            editor_combo_box_->addItem(choice.text, choice.value);
        }

        const int current_index =
            editor_combo_box_->findData(current_value);
        editor_combo_box_->setCurrentIndex(
            current_index >= 0 ? current_index : 0);
        editor_input_stack_->setCurrentWidget(editor_combo_box_);
    }
    else
    {
        editor_line_edit_->setText(current_value);
        editor_line_edit_->setPlaceholderText(definition.placeholder);
        const bool use_slider = definition.use_integer_slider;
        editor_camera_slider_->setVisible(use_slider);
        if (use_slider)
        {
            editor_camera_slider_->setRange(
                definition.slider_minimum,
                definition.slider_maximum);
            editor_camera_slider_->setValue(current_value.toInt());
            editor_line_edit_->setMinimumWidth(100);
            editor_line_edit_->setMaximumWidth(120);
        }
        else
        {
            editor_line_edit_->setMinimumWidth(0);
            editor_line_edit_->setMaximumWidth(QWIDGETSIZE_MAX);
        }
        camera_slider_active_ = use_slider;
        editor_input_stack_->setCurrentWidget(editor_line_container_);
        if (numeric_editor) {
            // Preserve the existing decimal point and sign so the requested
            // digit-only keypad can edit floating-point and negative values.
            editor_line_edit_->deselect();
            editor_line_edit_->setCursorPosition(current_value.size());
        } else {
            editor_line_edit_->selectAll();
        }
        editor_line_edit_->setFocus();
    }

    // 第四步：检查当前状态是否允许修改。
    // 例如自动曝光开启时，手动曝光值仍会保留，但编辑控件会被禁用。
    const bool editable =
        !definition.editable || definition.editable(working_config_);
    if (!editable)
    {
        editor_line_edit_->setEnabled(false);
        editor_combo_box_->setEnabled(false);
        editor_camera_slider_->setEnabled(false);
        editor_confirm_button_->setEnabled(false);
        editor_description_label_->setText(
            definition.description + " " + definition.disabled_reason);
    }

    setNumericKeypadEnabled(numeric_editor && editable);
    parameter_editor_->show();
}

bool ParameterConfigDialog::updateParameterFromEditor(QString *error_message)
{
    if (editing_parameter_id_.isEmpty())
    {
        setError(error_message, "没有选中参数");
        return false;
    }

    // 仍然通过统一参数表查找，避免在保存函数里再次维护参数 ID 列表。
    ParameterDefinition definition;
    if (!findParameterDefinition(
            editing_parameter_id_, working_config_, &definition))
    {
        setError(error_message, "未知参数：" + editing_parameter_id_);
        return false;
    }

    // 从当前可见的编辑控件取值。下拉框读取 itemData，输入框读取用户文字。
    const QString raw_value =
        definition.editor_type == ParameterEditorType::Choice
            ? editor_combo_box_->currentData().toString()
            : editor_line_edit_->text().trimmed();

    // 先复制候选配置：解析或校验失败时，working_config_ 保持不变。
    WarehouseConfig candidate = working_config_;
    if (!definition.write ||
        !definition.write(candidate, raw_value, error_message))
    {
        return false;
    }

    // 字符串转数字由参数定义负责；跨参数关系和数值范围由总校验函数负责。
    const QString validation_error = validateWarehouseConfig(candidate);
    if (!validation_error.isEmpty())
    {
        setError(error_message, validation_error);
        return false;
    }

    // 只有解析和总校验都成功，才提交这一项到临时配置。
    working_config_ = candidate;
    return true;
}

void ParameterConfigDialog::commitParameterEdit()
{
    // 单项确定按钮只提交到 working_config_，此时还没有写入 warehouse_config.json。
    QString error_message;
    if (!updateParameterFromEditor(&error_message))
    {
        QMessageBox::warning(this, "参数错误", error_message);
        return;
    }

    // 提交成功后关闭右侧编辑器，并重建列表显示新值。
    editing_parameter_id_.clear();
    parameter_editor_->hide();
    camera_slider_active_ = false;
    setNumericKeypadEnabled(false);
    rebuildParameterList(true);
}

void ParameterConfigDialog::handleApply()
{
    // 页面顶部启用按钮是最终提交点：先校验整份配置，再原子写入 JSON。
    const QString validation_error =
        validateWarehouseConfig(working_config_);

    if (!validation_error.isEmpty())
    {
        QMessageBox::warning(
            this, "参数错误", validation_error);
        return;
    }

    QString error_message;
    if (!saveWarehouseConfig(
            working_config_, &error_message))
    {
        QMessageBox::warning(
            this, "保存失败", error_message);
        return;
    }

    // 保存成功后更新基准配置并关闭窗口，调用方可通过 savedConfig() 取得结果。
    // 当前流程不再关闭窗口，而是立即通知主窗口应用配置。
    original_config_ = working_config_;
    emit configApplied(working_config_);
}

