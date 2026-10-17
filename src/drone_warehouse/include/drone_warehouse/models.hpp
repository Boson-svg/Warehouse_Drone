#pragma once
#include "drone_warehouse/color_palette.hpp"

#include <QByteArray>
#include <QMetaType>
#include <QRectF>
#include <QString>
#include <QVector>

//表示三维位置和姿态
struct Pose3D
{
    double x = 0.0;//三维空间中的 x 坐标
    double y = 0.0;//三维空间中的 y 坐标
    double z = 0.0;//三维空间中的 z 坐标

    double roll = 0.0;//绕 x 轴的翻滚角
    double pitch = 0.0;//绕 y 轴的俯仰角
    double yaw = 0.0;//绕 z 轴的偏航角，也就是朝向角
};

//表示无人机当前状态
struct DroneState
{
    Pose3D pose;//无人机当前的位置和姿态信息
    double speed = 0.0;//无人机当前速度，单位通常按米每秒理解
    double battery = 100.0;//无人机当前电量百分比
    QString flight_mode;//无人机当前飞行模式，例如 OFFBOARD
    bool connected = false;//无人机当前是否已连接
};

//表示单个轨迹点
struct TrajectoryPoint
{
    double x = 0.0;//轨迹点在场景中的 x 坐标
    double y = 0.0;//轨迹点在场景中的 y 坐标
    double z = 0.0;//轨迹点在场景中的 z 坐标
};

//表示一个货架在场景中的几何信息
struct ShelfBlock
{
    QRectF base_rect;//货架底面矩形，包含左上角位置以及底面的宽和长
    double height = 0.0;//货架高度，用来决定伪3D绘制时的立体高度
    QString name;//货架名称或编号，例如 A01
    QColor color;//货架绘制颜色
};

//表示单个货物或包裹在场景中的位置
struct CargoItem
{
    QString category_id;//商品类别编号，用来区分货物属于哪一类商品
    QString package_id;//包裹唯一编号，用来唯一标识某一个具体包裹
    QString shelf_name;//所属货架名称，例如 A01，用来表示它放在哪个货架上
    double x = 0.0;//货物在场景中的 x 坐标
    double y = 0.0;//货物在场景中的 y 坐标
    double z = 0.0;//货物在场景中的 z 坐标，通常表示所在层高位置
};

/*********************ros移植部分***********************/
// 原 drone_qt 版本把 WorldCoord 放在 position_view_widget.hpp 里，
// 但当前仓储项目并没有移植 position_view_widget 这个文件。
// 为了让 RosManager 仍然能继续使用上传路线和接收返回路线的同一套数据结构，
// 这里把最小可用的 WorldCoord 独立提到公共 models.hpp 中，避免继续依赖不存在的界面类头文件。
struct WorldCoord
{
    double x = 0.0;//世界坐标系下的 x 坐标
    double y = 0.0;//世界坐标系下的 y 坐标
    double z = 0.0;//世界坐标系下的 z 坐标
    double yaw = 0.0;//世界坐标系下的 yaw 航向
};
Q_DECLARE_METATYPE(WorldCoord)
Q_DECLARE_METATYPE(QVector<WorldCoord>)
/******************************************************/

/***********************AI相关*************************/
//差异状态枚举
enum class SlotDiffStatus
{
    Empty,                 // 台账空，巡检空
    Matched,               // 台账与巡检完整匹配
    ManualOnly,            // 只有台账，没有巡检结果
    ObservedOnly,          // 只有巡检，没有台账
    PartialObserved,       // 巡检结果不完整
    Mismatch,              // 台账与巡检冲突
    PositionOnly,          // 只有位置码，没有类别/包裹
    ObservedWithoutImage   // 有巡检结果，但本次无图
};

//单槽位分析输入
struct SlotAnalysisInput
{
    QString shelf_name;            // 例如 货架1
    int shelf_index = -1;
    QString side;                  // front/back
    int row = -1;
    int col = -1;

    QString manual_category_id;//地面站手动维护的类别编号
    QString manual_package_id;//地面站手动维护的包裹编号

    QString observed_category_id;//无人机巡检实际识别到的类别编号
    QString observed_package_id;//无人机巡检实际识别到的包裹编号
    QString observed_slot_code;//无人机巡检实际识别到仓库位置
    QString observed_time_text;//无人机巡检结果时间

    bool has_image = false;//当前点位是否已经绑定图片
};

//单槽位规则分析结果
struct SlotRuleAnalysis
{
    SlotAnalysisInput input;
    SlotDiffStatus status = SlotDiffStatus::Empty;

    bool has_manual_data = false;//是否有台账数据
    bool has_observed_data = false;//是否有巡检数据
    bool has_partial_observed_data = false;//是否有部分巡检数据
    bool has_position_only = false;//是否只有位置码，没有类别或包裹

    bool package_match = false;//包裹信息是否匹配
    bool category_match = false;//类别信息是否匹配
    bool exact_match = false;//；两者是否都匹配

    QString summary;               // 本地规则生成的简述
    QString reason;                // 本地规则原因
    int priority = 0;              // 0=最低，100=最高
    bool should_revisit = false;   // 是否建议复飞
};

//AI 输出结果
struct SlotAiAnalysis
{
    SlotRuleAnalysis rule;

    QString severity;              // low / medium / high
    QString ai_summary;            // AI 人话摘要
    QString ai_reason;             // AI 补充理由
    QString ai_action;             // 建议动作
    bool ai_should_revisit = false;//ai是否建议复飞
};
/******************************************************/

struct SlotImageData
{
    QByteArray image_data;//图片原始字节
    QString image_format;//图片格式，例如 jpg/png
    QString barcode;//这张图对应的条码文本
    QString time_text;//这张图对应的识别时间
};

struct SlotLocation
{
    int shelf_index = -1;//第几个货架
    QString side;//front 或 back
    int row = -1;//从 0 开始，最大值由 ShelfConfig::rows 决定
    int col = -1;//从 0 开始，最大值由 ShelfConfig::columns 决定
    bool valid = false;//是否成功定位
};

struct ShelfSlotItem
{
    QString category_id;//地面站手动维护的类别编号
    QString package_id;//地面站手动维护的包裹编号

    QString observed_category_id;//无人机巡检实际识别到的类别编号
    QString observed_package_id;//无人机巡检实际识别到的包裹编号
    QString position_package_id;//无人机巡检实际识别到仓库位置
    QString observed_time_text;//无人机巡检结果时间

    bool has_image = false;//当前点位是否已经绑定图片
    SlotImageData latest_image;//当前点位最近一张图片
};

struct ShelfPanelData
{
    QString display_name;//弹窗顶部显示名称，例如“货架1”
    QString button_status_color;//顶部按钮前面的状态灯颜色，直接存颜色字符串，例如 #00d48a
    QVector<ShelfSlotItem> front_slots;//正面槽位，数量由 SlotGridConfig 决定
    QVector<ShelfSlotItem> back_slots;//背面槽位，数量由 SlotGridConfig 决定
};


//统一保存整个主场景需要用到的数据
struct WarehouseSceneData
{
    DroneState drone_state;//无人机当前状态数据
    QVector<TrajectoryPoint> trajectory;//无人机历史轨迹点集合
    QVector<ShelfBlock> shelves;//场景中的所有货架数据
    QVector<CargoItem> cargos;//场景中的所有货物或包裹数据
};