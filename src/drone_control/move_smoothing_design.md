# Move 平滑过渡逐行对照源码讲义

这份文档是给主人专门准备的“读源码版讲义”喵。目标不是只告诉你结论，而是带你顺着真实代码去看懂：

- `move` 平滑过渡功能到底改在了哪里
- 每个函数什么时候被调用
- 每个变量在什么时候被读写
- 一次 `move` 从开始到结束，控制流是怎么流动的

本文重点对照这几个文件：

- [drone_action.hpp](/home/bosen/drone_ws/src/drone_control/include/drone_control/drone_action.hpp)
- [action_executor.hpp](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp)
- [mission_controller_node.cpp](/home/bosen/drone_ws/src/drone_control/src/mission_controller_node.cpp)

---

## 1. 先别急着看 `buildNextMoveSetpoint()`，先看整体入口

如果一上来就盯着 `buildNextMoveSetpoint()`，很容易只看到“怎么算下一步”，但看不懂：

- 目标点从哪来
- 参数从哪来
- 什么时候初始化
- 为什么这个函数能知道“上一次发了什么命令”

所以正确阅读顺序应该是：

1. YAML `move` 配置
2. `parseMoveAction()`
3. `DroneAction::createMoveToAction()`
4. `MoveRuntimeState`
5. `executeAction()`
6. `executeMoveToPosition()`
7. `initializeMoveRuntime()`
8. `resolveMoveTargetPoseOnce()`
9. `buildNextMoveSetpoint()`
10. `stepTowardPosition()` / `stepTowardYawRad()`
11. `isMoveGoalReached()`

下面就按这个顺序来喵。

---

## 2. 第 1 站：YAML 里的 `move` 参数是起点

比如你现在 mission 里写的是：

```yaml
- type: "move"
  frame: "world_body"
  position: [0.75, 0.00, 0.40]
  yaw: 1.57
  tolerance: 0.12
  yaw_tolerance_deg: 4.0
  max_xy_speed_mps: 0.30
  max_z_speed_mps: 0.15
  max_yaw_rate_deg_s: 18.0
```

这些字段不会直接被飞控使用，它们首先会被任务控制器解析。

---

## 3. 第 2 站：`parseMoveAction()` 把 YAML 变成 `DroneAction`

源码位置：

- [mission_controller_node.cpp:216](/home/bosen/drone_ws/src/drone_control/src/mission_controller_node.cpp:216)

这个函数是：

```cpp
std::shared_ptr<DroneAction> parseMoveAction(const YAML::Node &item,
                                             std::size_t index)
```

你可以把它理解成：

**把一条 YAML 里的 move 配置，翻译成程序内部的动作对象。**

### 3.1 逐段看

#### 第一段：创建目标位姿对象

```cpp
geometry_msgs::msg::PoseStamped target;
```

作用：

- 创建一个 ROS 位姿消息对象
- 后面会把位置和 yaw 都塞进去

#### 第二段：定义角度换算常量

```cpp
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 57.29577951308232;
```

作用：

- 因为 YAML 里有的字段用度，有的字段用弧度
- 程序内部又希望统一用弧度做计算

#### 第三段：读取 `frame`

```cpp
const std::string frame =
    item["frame"] ? item["frame"].as<std::string>() : "world_body";
```

作用：

- 读取目标坐标系
- 如果用户没写，就默认 `world_body`

#### 第四段：读取 `position`

```cpp
const YAML::Node pos = item["position"];
if (!pos || pos.size() != 3) { ... }
```

作用：

- 检查位置数组是否存在，是否是 `[x, y, z]`

后面真正写入：

```cpp
target.pose.position.x = pos[0].as<double>();
target.pose.position.y = pos[1].as<double>();
target.pose.position.z = pos[2].as<double>();
```

#### 第五段：读取 `yaw`

```cpp
const double yaw = item["yaw"] ? item["yaw"].as<double>() : 0.0;
tf2::Quaternion q;
q.setRPY(0.0, 0.0, yaw);
target.pose.orientation = tf2::toMsg(q);
```

这几行一定要看懂：

1. YAML 里的 `yaw` 先读出来
2. 它当前按弧度解释
3. 再把 yaw 变成四元数
4. 写进 `target.pose.orientation`

所以：

**目标 yaw 没有单独存成一个变量，而是直接放进 `PoseStamped.orientation` 里。**

#### 第六段：读取平滑参数

```cpp
const double tolerance = ...
const double yaw_tolerance_deg = ...
const double max_xy_speed_mps = ...
const double max_z_speed_mps = ...
const double max_yaw_rate_deg_s = ...
```

这些值就是平滑控制的“动作参数来源”。

#### 第七段：创建 `DroneAction`

```cpp
return DroneAction::createMoveToAction(
    target,
    use_frame,
    tolerance,
    yaw_tolerance_deg * kDegToRad,
    max_xy_speed_mps,
    max_z_speed_mps,
    max_yaw_rate_deg_s * kDegToRad);
```

这几行是整个参数链路最关键的一步：

- 位置容差直接传
- `yaw_tolerance_deg` 先转弧度
- `max_yaw_rate_deg_s` 也先转弧度

也就是说：

**从这一刻开始，后面的执行层几乎都在用弧度。**

---

## 4. 第 3 站：`createMoveToAction()` 把参数存进动作对象

源码位置：

- [drone_action.hpp:84](/home/bosen/drone_ws/src/drone_control/include/drone_control/drone_action.hpp:84)

函数定义：

```cpp
static std::shared_ptr<DroneAction> createMoveToAction(...)
```

### 4.1 逐段看

#### 创建动作对象

```cpp
auto action = std::make_shared<DroneAction>(PrivateTag{});
```

作用：

- 创建一个新的 `DroneAction`

#### 写动作类型

```cpp
action->type_ = ActionType::MOVE_TO_POSITION;
```

作用：

- 告诉执行器：这是一个 move 动作

#### 写目标位姿

```cpp
action->target_pose_ = target_pose;
```

作用：

- 保存最终目标位置和目标 yaw

#### 写参考系

```cpp
action->frame_ = frame;
```

作用：

- 保存这个目标应该如何解释

#### 写位置容差

```cpp
action->position_tolerance_ = position_tolerance;
```

作用：

- 后面判断“位置到没到”要用

#### 写 yaw 容差

```cpp
action->yaw_tolerance_rad_ = yaw_tolerance_rad;
```

作用：

- 后面判断“机头有没有转到位”要用

#### 写平滑速度限制

```cpp
action->move_max_xy_speed_mps_ = move_max_xy_speed_mps;
action->move_max_z_speed_mps_ = move_max_z_speed_mps;
action->move_max_yaw_rate_radps_ = move_max_yaw_rate_radps;
```

作用：

- 后面每个控制周期计算“本周期最多能走多少”

### 4.2 这一步的本质

这一层还没有开始执行控制，只是在做：

**动作参数归档**

---

## 5. 第 4 站：你必须先看懂 `MoveRuntimeState`

源码位置：

- [action_executor.hpp:164](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:164)

```cpp
struct MoveRuntimeState
```

这不是配置，而是“执行中的状态”。

这是很多新手最容易混淆的点：

- `DroneAction` 保存的是“这个动作想怎么执行”
- `MoveRuntimeState` 保存的是“这个动作现在执行到哪里了”

### 5.1 逐个变量解释

#### `initialized`

```cpp
bool initialized = false;
```

作用：

- 表示当前 move 是否已经初始化过运行时状态

什么时候变化：

- 初始是 `false`
- 第一次进入 `move` 后会被设成 `true`
- 动作结束后在 `resetMoveRuntimeState()` 里恢复成 `false`

#### `start_pose`

```cpp
geometry_msgs::msg::PoseStamped start_pose;
```

作用：

- 保存动作开始时的当前位置

用途：

- 记录动作起点
- 后续调试时可以知道从哪开始飞

#### `resolved_target_pose`

```cpp
geometry_msgs::msg::PoseStamped resolved_target_pose;
```

作用：

- 保存最终固定的世界系终点

这是最重要的变量之一。

因为原始目标不一定天然是固定 `world_enu` 点，所以必须先“冻结”。

#### `last_command_pose`

```cpp
geometry_msgs::msg::PoseStamped last_command_pose;
```

作用：

- 保存上一帧真正发出去的命令位姿

这是平滑过渡成立的核心。

为什么？

因为下一帧不是从“当前位置”继续推，而是从“上一帧命令位置”继续推。

#### `last_update_time`

```cpp
rclcpp::Time last_update_time;
```

作用：

- 保存上一帧更新时间

用途：

- 计算 `dt`

#### `start_yaw_rad`

```cpp
double start_yaw_rad = 0.0;
```

作用：

- 保存动作开始时的 yaw

#### `target_yaw_rad`

```cpp
double target_yaw_rad = 0.0;
```

作用：

- 保存目标 yaw

#### `last_command_yaw_rad`

```cpp
double last_command_yaw_rad = 0.0;
```

作用：

- 保存上一帧发出去的命令 yaw

和 `last_command_pose` 一样，是 yaw 平滑过渡的核心状态。

#### `total_distance_m`

```cpp
double total_distance_m = 0.0;
```

作用：

- 保存动作开始时总距离

当前更多是为了调试、状态分析。

#### `stable_count`

```cpp
int stable_count = 0;
```

作用：

- 连续满足到达条件的帧数

意义：

- 防止抖动导致误判完成

---

## 6. 第 5 站：`executeAction()` 把 move 送进执行器

源码位置：

- [action_executor.hpp:788](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:788)

```cpp
void executeAction(const std::shared_ptr<DroneAction> &action)
```

这里是动作分发器。

重点看：

```cpp
case ActionType::MOVE_TO_POSITION:
    executeMoveToPosition(action);
    break;
```

也就是说：

**所有 move 的平滑控制，最终都会进入 `executeMoveToPosition()`。**

---

## 7. 第 6 站：`executeMoveToPosition()` 是主控制流程

源码位置：

- [action_executor.hpp:814](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:814)

这个函数建议你分成 6 段看。

### 第 1 段：初始化检查

```cpp
if (!move_runtime_.initialized)
{
    if (!initializeMoveRuntime(action))
    {
        sendPositionSetpoint(current_pose_);
        return;
    }
}
```

意思：

- 如果是第一次进入这个 move
- 就先初始化
- 如果初始化失败，先保持当前位置不动

### 第 2 段：计算 `dt`

```cpp
const rclcpp::Time now = node_->now();
double dt = (now - move_runtime_.last_update_time).seconds();
move_runtime_.last_update_time = now;
```

作用：

- 计算本周期时长

紧接着：

```cpp
if (dt <= 0.0)
{
    dt = 0.02;
}
```

作用：

- 防止首帧或时间异常导致本周期步长为 0

### 第 3 段：生成下一帧 setpoint

```cpp
const geometry_msgs::msg::PoseStamped next_setpoint =
    buildNextMoveSetpoint(action, dt);
```

这一句是：

**整条平滑参考轨迹生成的核心调用点**

### 第 4 段：保存并发布

```cpp
move_runtime_.last_command_pose = next_setpoint;
sendPositionSetpoint(next_setpoint);
```

注意顺序：

1. 先把这次发出去的命令保存为“上一帧命令”
2. 再发送给飞控

### 第 5 段：判断是否完成

```cpp
if (isMoveGoalReached(action))
{
    move_runtime_.stable_count++;
    if (move_runtime_.stable_count > 20)
    {
        completeCurrentAction(...);
        return;
    }
}
else
{
    move_runtime_.stable_count = 0;
}
```

意思：

- 只要某一帧没到位，计数清零
- 必须连续很多帧都到位，动作才算完成

### 第 6 段：状态播报

后面这段：

```cpp
const double distance_to_target = ...
const double yaw_error_deg = ...
...
broadcastStatusThrottled(...)
```

作用：

- 把当前状态、目标状态、剩余距离、剩余 yaw 输出给日志和状态话题

这段不控制飞机，但对调试非常有帮助。

---

## 8. 第 7 站：`initializeMoveRuntime()` 只在第一帧做准备

源码位置：

- [action_executor.hpp:620](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:620)

### 8.1 先看整体

这个函数做的事可以概括成一句话：

**给这次 move 建立一套完整的平滑执行上下文。**

### 8.2 逐行拆

#### 先创建局部目标变量

```cpp
geometry_msgs::msg::PoseStamped resolved_target_pose;
```

#### 调用目标冻结函数

```cpp
if (!resolveMoveTargetPoseOnce(action, resolved_target_pose))
{
    return false;
}
```

作用：

- 把目标解析成固定 `world_enu` 终点

#### 标记初始化完成

```cpp
move_runtime_.initialized = true;
```

#### 记录起点

```cpp
move_runtime_.start_pose = current_pose_;
```

#### 保存最终终点

```cpp
move_runtime_.resolved_target_pose = resolved_target_pose;
```

#### 把当前位姿作为第一帧命令位姿

```cpp
move_runtime_.last_command_pose = current_pose_;
move_runtime_.last_command_pose.header.frame_id = "world_enu";
```

这一步非常关键：

- 第一帧平滑推进一定要从当前实际位置起步

#### 记录时间

```cpp
move_runtime_.last_update_time = node_->now();
```

#### 记录 yaw 状态

```cpp
move_runtime_.start_yaw_rad = getPoseYawRad(current_pose_);
move_runtime_.target_yaw_rad = getPoseYawRad(resolved_target_pose);
move_runtime_.last_command_yaw_rad = move_runtime_.start_yaw_rad;
```

意义：

- 当前 yaw 是平滑起点
- 目标 yaw 是平滑终点

#### 记录总距离

```cpp
move_runtime_.total_distance_m =
    SpatialPoint(current_pose_).distance(SpatialPoint(resolved_target_pose));
```

#### 清稳定计数

```cpp
move_runtime_.stable_count = 0;
```

---

## 9. 第 8 站：`resolveMoveTargetPoseOnce()` 是“目标冻结器”

源码位置：

- [action_executor.hpp:574](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:574)

这个函数是整套方案的基础。

如果这一步没理解，后面就会搞不懂为什么目标不会漂移。

### 9.1 第一行

```cpp
resolved_target_pose = action->getTargetPose();
```

作用：

- 先把 `DroneAction` 里保存的原始目标取出来

### 9.2 `WORLD_BODY` 分支

```cpp
if (action->getFrame() == DroneAction::Frame::WORLD_BODY)
{
    resolved_target_pose = tf_buffer_.transform(resolved_target_pose, "world_enu");
}
```

意思：

- 原始目标在 `world_body`
- 现在把它变换到 `world_enu`

注意重点：

- 这里只做一次
- 不是每个周期都做

### 9.3 `BODY` 分支

```cpp
const Eigen::Vector3d delta = bodyVectorToEnu(...);
const double target_yaw_delta = getPoseYawRad(resolved_target_pose);

resolved_target_pose = last_finish_pose_;
resolved_target_pose.pose.position.x += delta.x();
resolved_target_pose.pose.position.y += delta.y();
resolved_target_pose.pose.position.z += delta.z();
resolved_target_pose.pose.orientation =
    makeQuaternionFromYaw(
        normalizeAngleRad(getPoseYawRad(last_finish_pose_) + target_yaw_delta));
```

这段是很多人第一次看会晕的地方，慢慢拆：

#### 第一步：读取 BODY 目标里的位移增量

```cpp
resolved_target_pose.pose.position.x
resolved_target_pose.pose.position.y
resolved_target_pose.pose.position.z
```

这些不是世界坐标，而是：

- 机体系增量

#### 第二步：转成 ENU 增量

```cpp
const Eigen::Vector3d delta = bodyVectorToEnu(...);
```

#### 第三步：读取 BODY yaw 增量

```cpp
const double target_yaw_delta = getPoseYawRad(resolved_target_pose);
```

#### 第四步：以 `last_finish_pose_` 为锚点

```cpp
resolved_target_pose = last_finish_pose_;
```

意思：

- 当前 BODY move 是相对上一动作结束位姿来定义的

#### 第五步：把位置增量叠上去

```cpp
resolved_target_pose.pose.position.x += delta.x();
...
```

#### 第六步：把 yaw 增量叠上去

```cpp
makeQuaternionFromYaw(
    normalizeAngleRad(getPoseYawRad(last_finish_pose_) + target_yaw_delta));
```

这一句的意思是：

- 最终 yaw = 上一动作结束 yaw + 本次 BODY 给定 yaw 增量

### 9.4 为什么叫“冻结”

因为这个函数执行完以后：

- 你就得到了一个固定的 `resolved_target_pose`
- 后面控制过程不会再改这个终点

---

## 10. 第 9 站：`buildNextMoveSetpoint()` 是平滑参考轨迹生成器

源码位置：

- [action_executor.hpp:645](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:645)

这就是你选中的函数，主人要重点吃透它喵。

### 10.1 函数输入

```cpp
const std::shared_ptr<DroneAction> &action,
double dt
```

含义：

- `action` 提供平滑参数
- `dt` 告诉你“这一个控制周期过了多久”

### 10.2 第一行

```cpp
geometry_msgs::msg::PoseStamped next_setpoint = move_runtime_.last_command_pose;
```

意义：

- 先拿上一帧命令位姿做模板
- 因为下一帧命令应该在它的基础上继续往前走

### 10.3 提取当前位置和目标位置

```cpp
const Eigen::Vector3d from(... last_command_pose ...);
const Eigen::Vector3d to(... resolved_target_pose ...);
```

注意：

- `from` 不是 `current_pose_`
- `from` 是 `last_command_pose`

这是这套方案的核心设计点。

### 10.4 计算下一步位置

```cpp
const Eigen::Vector3d next_position = stepTowardPosition(
    from,
    to,
    action->getMoveMaxXYSpeed() * dt,
    action->getMoveMaxZSpeed() * dt);
```

这里可以这样理解：

- `max_xy_speed * dt` = 本周期最大允许水平位移
- `max_z_speed * dt` = 本周期最大允许竖直位移

然后调用 `stepTowardPosition()`，算出下一帧应该发到哪里。

### 10.5 把位置写回 setpoint

```cpp
next_setpoint.pose.position.x = next_position.x();
next_setpoint.pose.position.y = next_position.y();
next_setpoint.pose.position.z = next_position.z();
```

### 10.6 计算下一步 yaw

```cpp
move_runtime_.last_command_yaw_rad = stepTowardYawRad(
    move_runtime_.last_command_yaw_rad,
    move_runtime_.target_yaw_rad,
    action->getMoveMaxYawRateRadps() * dt);
```

含义：

- 从“上一帧命令 yaw”出发
- 朝“目标 yaw”转过去
- 但是每帧最多只转 `max_yaw_rate * dt`

### 10.7 把 yaw 写回 setpoint

```cpp
next_setpoint.pose.orientation =
    makeQuaternionFromYaw(move_runtime_.last_command_yaw_rad);
```

这一步的意义是：

- ROS 位置 setpoint 还是要发四元数
- 所以最终要把平滑后的 yaw 再包回四元数

### 10.8 返回结果

```cpp
return next_setpoint;
```

这意味着：

**每次调用 `buildNextMoveSetpoint()`，都会生成一帧新的平滑中间目标。**

---

## 11. 第 10 站：`stepTowardPosition()` 逐行看

源码位置：

- [action_executor.hpp:522](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:522)

### 11.1 初始化结果

```cpp
Eigen::Vector3d result = from;
```

意思：

- 先假设结果就是当前命令位置
- 后面再往目标推进

### 11.2 计算平面增量

```cpp
const Eigen::Vector2d xy_delta = to.head<2>() - from.head<2>();
const double xy_distance = xy_delta.norm();
```

作用：

- 算水平面还差多少

### 11.3 判断水平是否一步可到

```cpp
if (xy_distance <= max_xy_step || max_xy_step <= 0.0)
{
    result.x() = to.x();
    result.y() = to.y();
}
```

意思：

- 如果这一帧允许走的距离已经足够到目标
- 那就直接贴到目标

### 11.4 否则只推进一小步

```cpp
const Eigen::Vector2d xy_step = xy_delta.normalized() * max_xy_step;
result.x() += xy_step.x();
result.y() += xy_step.y();
```

意思：

- 沿着当前方向走 `max_xy_step`

### 11.5 z 方向同样处理

```cpp
const double z_delta = to.z() - from.z();
if (std::abs(z_delta) <= max_z_step || max_z_step <= 0.0)
{
    result.z() = to.z();
}
else
{
    result.z() += std::copysign(max_z_step, z_delta);
}
```

意思：

- 竖直方向单独做限幅

### 11.6 为什么这样设计

好处是：

- 水平和竖直可分开调
- 室内更稳

---

## 12. 第 11 站：`stepTowardYawRad()` 逐行看

源码位置：

- [action_executor.hpp:506](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:506)

### 12.1 先算最短角误差

```cpp
const double yaw_error = normalizeAngleRad(target_yaw_rad - current_yaw_rad);
```

作用：

- 求当前朝向离目标朝向还差多少
- 并且保证差值在 `[-pi, pi]`

### 12.2 判断一步是否能到

```cpp
if (std::abs(yaw_error) <= max_step_rad)
{
    return target_yaw_rad;
}
```

意思：

- 如果本周期允许的最大转角已经足够到目标
- 那就直接到目标

### 12.3 否则只转一步

```cpp
return normalizeAngleRad(
    current_yaw_rad + std::copysign(max_step_rad, yaw_error));
```

意思：

- 沿着 yaw 误差的正方向或负方向
- 只走一个最大允许角步长

---

## 13. 第 12 站：`isMoveGoalReached()` 为什么能判断完成

源码位置：

- [action_executor.hpp:684](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:684)

### 13.1 位置误差

```cpp
const SpatialPoint current(current_pose_);
const SpatialPoint target(move_runtime_.resolved_target_pose);
const double position_error = current.distance(target);
```

这里用的是：

- 当前实际位置
- 最终固定目标位置

### 13.2 yaw 误差

```cpp
const double yaw_error = std::abs(normalizeAngleRad(
    move_runtime_.target_yaw_rad - getPoseYawRad(current_pose_)));
```

这里用的是：

- 当前实际 yaw
- 最终目标 yaw

### 13.3 双条件判断

```cpp
return position_error < action->getPositionTolerance() &&
       yaw_error < action->getYawToleranceRad();
```

意思：

- 位置到了还不够
- yaw 也得到了才行

---

## 14. 一次完整 `move` 的真实时序图

下面用文字流程图把一次完整动作串起来：

```text
mission.yaml
  -> parseMoveAction()
      读取 position / yaw / tolerance / speed limits
  -> DroneAction::createMoveToAction()
      保存 move 参数
  -> ActionExecutor::executeAction()
      发现这是 MOVE_TO_POSITION
  -> executeMoveToPosition()
      第一次进入:
        -> initializeMoveRuntime()
            -> resolveMoveTargetPoseOnce()
            -> 保存 start_pose / resolved_target_pose / last_command_pose
      每个控制周期:
        -> 计算 dt
        -> buildNextMoveSetpoint()
            -> stepTowardPosition()
            -> stepTowardYawRad()
        -> sendPositionSetpoint()
        -> isMoveGoalReached()
      连续多帧满足阈值:
        -> completeCurrentAction()
```

---

## 15. 为什么这套方案能实现“平滑式过渡”

现在把最关键的一句话说透喵：

这套方案之所以平滑，不是因为它用了什么复杂规划器，而是因为它同时满足了这 4 点：

1. 最终目标先被冻结，不会执行中漂移
2. 每帧都从上一帧命令继续推进，而不是直接跳终点
3. 位置每帧推进距离受 `max_xy_speed_mps` / `max_z_speed_mps` 限制
4. yaw 每帧推进角度受 `max_yaw_rate_radps` 限制

因此发给飞控的 setpoint 序列本身就是连续变化的。

飞控看到的不是：

- “一下子跳到远方”

而是：

- “下一帧再往前一点”
- “下一帧再转一点”

这就是平滑过渡的本质。

---

## 16. 新手读这段源码时最容易卡住的 5 个点

### 卡点 1：为什么不是直接用 `current_pose_`

答案：

- 因为 `current_pose_` 会受瞬时估计误差影响
- `last_command_pose` 更像连续参考轨迹

### 卡点 2：为什么要先冻结目标

答案：

- 避免 `BODY` / `WORLD_BODY` 在执行过程中因为姿态变化而漂移

### 卡点 3：为什么 yaw 用弧度

答案：

- `tf2`、三角函数、角误差处理都天然使用弧度

### 卡点 4：为什么完成条件要连续多帧

答案：

- 防止抖动导致误判

### 卡点 5：为什么位置分 `xy` 和 `z`

答案：

- 室内一般希望升降更慢、更稳

---

## 17. 你下一步怎么继续读代码最有效

如果主人准备继续自己啃源码，浮浮酱建议：

1. 先打开 [action_executor.hpp](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:814)
   只盯 `executeMoveToPosition()`

2. 遇到不会的函数，再跳去看：
   - `initializeMoveRuntime()`
   - `resolveMoveTargetPoseOnce()`
   - `buildNextMoveSetpoint()`
   - `stepTowardPosition()`
   - `stepTowardYawRad()`
   - `isMoveGoalReached()`

3. 再回来看 `executeMoveToPosition()`，就会顺很多

这比从头把整个 `action_executor.hpp` 一把梭读完更有效。

---

## 18. 总结

把整套实现压缩成一句话就是：

**先把 move 的目标位姿解析成固定世界系终点，再在每个控制周期里，基于上一帧命令位姿和速度/角速度上限，生成下一帧中间 setpoint，并在位置和 yaw 连续多帧都进入容差后结束动作。**

如果主人愿意，浮浮酱下一步可以继续做两种增强版之一：

1. 给这份文档补“逐行摘代码块 + 行间解释”
2. 直接按 [action_executor.hpp](/home/bosen/drone_ws/src/drone_control/include/drone_control/action_executor.hpp:814) 带你一段一段口头精读 `executeMoveToPosition()` 喵
