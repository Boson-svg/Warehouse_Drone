# K230 路线点扫描门控方案

## 1. 目标

让 `k230_animals_uart_ros2_node` 只在无人机经过地面站规划路线点时发布 K230 识别结果。

期望行为：

```text
第一次经过路线点：允许扫描并发布 /k230/animals/center
第二次经过同一路线点：不再发布
不在路线点附近：丢弃 K230 识别结果
误入禁飞区或路线外区域：不发布 K230 识别结果
```

本方案按控制端要求，直接使用 `/mavros/local_position/pose`，不依赖 `/current_world_body_pos`。

## 2. 当前代码依据

### 2.1 地面站生成路线点

`src/drone_qt/src/position_view_widget.cpp:594-611`

```cpp
QVector<WorldCoord> PositionViewWidget::plannedWorldPoints() const
{
    const double cell_size_x = 0.5;
    const double cell_size_y = 0.5;

    for (const auto &cell : display_path_) {
        WorldCoord point;
        point.x = origin_x + cell.row * cell_size_x;
        point.y = origin_y + cell.col * cell_size_y;
        result.push_back(point);
    }
}
```

含义：地面站把 UI 路径格子转换成路线坐标点。

### 2.2 地面站上传路线点

`src/drone_qt/src/mainwindow.cpp:347-374`

```cpp
const QVector<WorldCoord> path_points = position_view_->plannedWorldPoints();
ros_manager_->uploadMissionSummary(path_points, summary);
```

`src/drone_qt/src/ros_manager.cpp:360-365`

```cpp
for (const auto &point : path_points) {
    drone_msgs::msg::WorldPoint world_point;
    world_point.x = point.x;
    world_point.y = point.y;
    request->points.push_back(world_point);
}
```

含义：地面站把路线点作为 `WorldPoint[]` 上传。

### 2.3 机载端回传路线点

`src/drone_control/src/route_comm_node.cpp:35-37`

```cpp
return_route_pub_ =
    create_publisher<drone_msgs::msg::WorldGroup>(
        "/return/drone/world_group", qos);
```

`src/drone_control/src/route_comm_node.cpp:183-187`

```cpp
drone_msgs::msg::WorldPoint point;
point.x = pos[0].as<double>();
point.y = pos[1].as<double>();
cached_route_.points.push_back(point);
```

`src/drone_control/src/route_comm_node.cpp:260-264`

```cpp
return_route_pub_->publish(cached_route_);
RCLCPP_INFO(get_logger(), "已回传处理后的路线到 /return/drone/world_group。");
```

含义：控制端把处理后的路线用 `drone_msgs/msg/WorldGroup` 发布出来。

### 2.4 当前位置来源

`src/drone_control/src/tf_bridge_node.cpp:30-32`

```cpp
px4_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&BodyToEnuBridge::px4Pose_callback, this, std::placeholders::_1));
```

含义：当前系统已经使用 `/mavros/local_position/pose` 作为飞控原始位姿来源。本方案中，K230 ROS2 节点也直接订阅这个 topic。

### 2.5 K230 当前发布逻辑

`src/drone_perception/src/k230_animals_uart_ros2_node.cpp:304-358`

当前逻辑是收到 K230 UART 检测包后，解析 JSON 并发布：

```cpp
center_pub_->publish(center_msg);
```

本方案需要在这句发布前增加门控判断。

## 3. 总体流程

```text
地面站规划路线
        ↓
生成路线点 WorldGroup
        ↓
K230 ROS2 节点订阅路线点 topic
        ↓
K230 ROS2 节点缓存路线点 scan_points_
        ↓
K230 ROS2 节点订阅 /mavros/local_position/pose
        ↓
实时获得无人机当前位置 current_x/current_y
        ↓
K230 板子持续识别并通过 UART 发结果
        ↓
ROS2 节点收到识别结果
        ↓
判断 current_x/current_y 是否接近某个未扫描路线点
        ↓
是：发布 /k230/animals/center，并标记该点已扫描
否：丢弃这次识别结果
```

## 4. Topic 与消息关系

### 4.1 路线点 topic

推荐做成参数：

```text
route_topic = "/return/drone/world_group"
```

消息类型：

```text
drone_msgs/msg/WorldGroup
```

`WorldGroup` 不是 topic 名，它只是消息类型。真正订阅的是 `/return/drone/world_group` 这个 topic。

### 4.2 当前位姿 topic

推荐做成参数：

```text
pose_topic = "/mavros/local_position/pose"
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

使用字段：

```text
msg->pose.position.x
msg->pose.position.y
```

### 4.3 K230 输出 topic

现有结构化目标中心话题：

```text
/k230/animals/center
drone_msgs/msg/K230AnimalCenter
```

本方案中，该话题只在门控通过时发布。

## 5. 路线点缓存的含义

“缓存路线点”不是保存文件，也不是重新规划航线。

含义是：K230 ROS2 节点收到 `WorldGroup` 后，在内存里保存一份路线点白名单。

建议内部结构：

```cpp
struct ScanPoint {
    double x_m {};
    double y_m {};
    bool visited {};
};
```

示例：

```text
scan_points_:
  0: x=0.0, y=0.0, visited=false
  1: x=0.5, y=0.0, visited=false
  2: x=1.0, y=0.0, visited=false
```

`visited=false` 表示该路线点还没有完成扫描。

## 6. 门控判断

每次收到 K230 检测结果时，先找当前无人机是否接近某个未扫描路线点。

距离计算：

```text
dx = current_x - target_x
dy = current_y - target_y
distance = sqrt(dx * dx + dy * dy)
```

判断条件：

```text
distance <= scan_radius
point.visited == false
```

推荐初始参数：

```text
scan_radius = 0.25 m
scan_dwell_ms = 500 ms
```

`scan_radius` 不能太小，因为 `/mavros/local_position/pose` 是连续小数，不会刚好等于 `(0,0)`。

## 7. PX4-Inspired 状态设计

本项目不是 PX4 工程，因此不引入 `ModuleBase`、`uORB`、`PX4_INFO` 等 PX4 API。

但状态和命名可以参考 PX4 风格：单位写进变量名、状态语义明确、早返回减少嵌套。

建议私有成员：

```cpp
static constexpr double kDefaultScanRadiusM = 0.25;
static constexpr int64_t kDefaultScanDwellMs = 500;

std::vector<ScanPoint> _scan_points {};

double _current_x_m {};
double _current_y_m {};
bool _pose_valid {false};
bool _route_valid {false};

int _active_scan_index {-1};
rclcpp::Time _scan_started_at {};
```

建议核心函数：

```cpp
void updateRoutePoints(const drone_msgs::msg::WorldGroup::SharedPtr msg);
void updateLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
int findActiveScanPoint() const;
bool shouldPublishK230Result();
void markActivePointVisited();
```

## 8. 伪代码

```cpp
bool K230AnimalsUartRos2Node::shouldPublishK230Result()
{
    if (!_pose_valid) {
        return false;
    }

    if (!_route_valid) {
        return false;
    }

    const int scan_index = findActiveScanPoint();

    if (scan_index < 0) {
        return false;
    }

    _active_scan_index = scan_index;
    return true;
}
```

```cpp
int K230AnimalsUartRos2Node::findActiveScanPoint() const
{
    for (int i = 0; i < static_cast<int>(_scan_points.size()); ++i) {
        const ScanPoint &point = _scan_points[i];

        if (point.visited) {
            continue;
        }

        const double dx_m = _current_x_m - point.x_m;
        const double dy_m = _current_y_m - point.y_m;
        const double distance_m = std::sqrt(dx_m * dx_m + dy_m * dy_m);

        if (distance_m <= _scan_radius_m) {
            return i;
        }
    }

    return -1;
}
```

发布前门控：

```cpp
if (!shouldPublishK230Result()) {
    return;
}

center_pub_->publish(center_msg);
markActivePointVisited();
```

## 9. 点位完成策略

推荐先使用简单策略：

```text
第一次在未扫描路线点附近发布有效 K230 结果后，标记该点 visited=true。
```

如果测试发现“某些点没有动物，导致一直不算扫描完成”，再升级为：

```text
进入点位半径后开始计时；
scan_dwell_ms 时间到后，即使没有有效动物，也标记该点扫描完成。
```

## 10. 禁飞区处理

本方案不单独订阅禁飞区。

规则：

```text
只允许 WorldGroup 路线点附近发布 K230 结果。
不在 WorldGroup 中的坐标，一律视为非扫描区域。
```

因此无人机误入禁飞区或路线外区域时，因为当前位置匹配不到未扫描路线点，K230 结果不会发布。

后续如需显式禁飞区，可新增禁飞区 topic 或把禁飞区点一起从地面站发出。

## 11. 测试顺序

1. 确认路线点 topic 存在：

```bash
ros2 topic list | grep world_group
ros2 topic info /return/drone/world_group
```

2. 确认 MAVROS 位姿存在：

```bash
ros2 topic echo /mavros/local_position/pose
```

3. 规划一条短路线：

```text
(0,0) -> (0.5,0) -> (1.0,0)
```

4. 让无人机或仿真位置接近每个路线点。

5. 观察：

```bash
ros2 topic echo /k230/animals/center
```

预期：

```text
第一次到点：有输出
第二次经过同一点：不再输出
路线外：不输出
```

## 12. 结论

本方案的核心不是让 K230 板子理解路线，而是让 ROS2 接收节点做门控：

```text
K230 板子：持续识别并通过 UART 发送结果
K230 ROS2 节点：结合路线点和 MAVROS 当前坐标，决定是否发布结果
控制端：只接收已经通过门控的 /k230/animals/center
```

这样实现改动集中、测试路径清晰，也符合 PX4-inspired 的状态管理思路：输入清楚、状态明确、早返回拒绝非法发布。
