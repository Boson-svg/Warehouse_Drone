# K230 动物识别与控制记录闭环说明

## 1. 目标

本文描述固定巡检路线下，K230 动物识别、路线点门控、控制端到位通知、视觉端拍照记录和后续跳过的完整闭环。

核心目标是：

```text
无人机按固定路线巡检
-> 视觉侧只在未扫描路线点附近发布稳定目标
-> 控制端只处理未扫描点的目标
-> 控制端到达拍照条件后发布 capture_ready
-> 视觉侧拍照并返回单个目标处理结果
-> 当前路线点全部目标处理完成后，记录该路线点已扫描
-> 后续再次经过同一路线点时不再发布目标、不再触发巡检
```

这里的“视觉侧”主要指 Linux 侧视觉 ROS2 桥接节点。K230 板端 Python 负责检测、稳定确认、拍照和 UART 通信；它不能直接订阅 ROS2 话题，ROS2 话题由 Linux 侧节点订阅和发布。

---

## 2. 两个关键输入

### 2.1 路线点列表

```text
话题名：/return/drone/world_group
消息类型：drone_msgs/msg/WorldGroup
```

该话题发布的是本次任务的路线点列表，不是无人机当前位置。

消息含义：

```text
points:
- x: 路线点 1 的世界坐标 x
  y: 路线点 1 的世界坐标 y
- x: 路线点 2 的世界坐标 x
  y: 路线点 2 的世界坐标 y
...
```

例如：

```text
points:
- x: 0.0
  y: 0.0
- x: 0.5
  y: 0.0
- x: 1.0
  y: 0.0
```

表示无人机本次任务计划经过这些巡检路线点。

### 2.2 当前无人机实时位置

```text
话题名：/mavros/local_position/pose
消息类型：geometry_msgs/msg/PoseStamped
```

该话题发布的是无人机当前实时位姿。

视觉侧主要使用：

```text
pose.position.x
pose.position.y
pose.position.z
```

其中 `x/y` 用于判断无人机当前是否进入某个路线点半径范围内。

---

## 3. 职责划分

### 3.1 K230 板端 Python

K230 板端负责图像侧工作：

1. 持续进行动物检测。
2. 对检测框做多帧稳定确认。
3. 过滤短暂误识别、类别跳变、位置跳变和低置信度目标。
4. 生成稳定目标列表。
5. 为同一帧内同一 `label` 的目标分配连续 `label_instance_id`。
6. 通过 UART 向 Linux 视觉节点发送稳定目标列表。
7. 接收 Linux 视觉节点通过 UART 转发的拍照指令。
8. 执行拍照、保存或发送图片。
9. 通过 UART 返回拍照结果。

### 3.2 Linux 视觉 ROS2 节点

Linux 视觉节点负责路线点门控和 ROS2/UART 桥接：

1. 订阅 `/return/drone/world_group`，缓存路线点列表。
2. 订阅 `/mavros/local_position/pose`，保存无人机当前实时位置。
3. 为每个路线点维护 `scanned` 状态。
4. 判断当前无人机是否进入某个未扫描路线点半径 `R`。
5. 只有在未扫描路线点附近，才发布稳定目标列表给控制端。
6. 如果当前路线点已经扫描过，则不发布目标，或发布 skipped 状态用于调试。
7. 订阅控制端的 `capture_ready`。
8. 收到 `capture_ready=true` 后，二次确认当前路线点仍未扫描，再转发拍照指令给 K230。
9. 收到 K230 拍照成功结果后，发布单个目标拍照结果给控制端。
10. 收到控制端确认当前路线点全部目标处理完成后，将该路线点标记为 `scanned=true`。

### 3.3 控制端

控制端负责飞行控制和到位判断：

1. 订阅视觉侧发布的稳定目标列表。
2. 检查同一 `label` 下 `label_instance_id` 是否连续。
3. 对收到的目标执行靠近、对准或到达拍照位置的控制逻辑。
4. 判断目标已经满足拍照条件。
5. 到达拍照条件后发布 `capture_ready=true`。
6. 订阅视觉侧拍照结果。
7. 维护当前路线点内的目标处理状态。
8. 当前路线点内所有目标都进入终态后，通知视觉侧该路线点扫描完成，并恢复航线去下一个路线点。

控制端不负责维护“路线点是否已经扫描过”。该状态由 Linux 视觉节点维护，因为视觉节点决定是否发布目标、是否触发 K230 拍照。

---

## 4. 路线点门控原理

### 4.1 路线点缓存

视觉侧收到 `/return/drone/world_group` 后，在内存中缓存路线点。

内部状态语义：

```text
scan_points:
  index: 路线点编号
  x: 路线点世界坐标 x
  y: 路线点世界坐标 y
  scanned: 是否已经完成扫描
```

初始状态：

```text
scan_points:
  0: x=0.0, y=0.0, scanned=false
  1: x=0.5, y=0.0, scanned=false
  2: x=1.0, y=0.0, scanned=false
```

### 4.2 实时位置更新

视觉侧持续订阅：

```text
/mavros/local_position/pose
```

每次收到位姿后，更新：

```text
current_x = pose.position.x
current_y = pose.position.y
current_z = pose.position.z
pose_valid = true
```

### 4.3 进入路线点半径 R 的判断

对每一个未扫描路线点，计算无人机当前位置到路线点的平面距离：

```text
dx = current_x - point.x
dy = current_y - point.y
distance = sqrt(dx * dx + dy * dy)
```

判断条件：

```text
distance <= scan_radius
且 point.scanned == false
```

满足条件时，认为无人机当前进入了一个未扫描路线点。

推荐初始参数：

```text
scan_radius = 0.25 m
```

`scan_radius` 不能太小，因为 `/mavros/local_position/pose` 是连续小数，无人机不会刚好等于路线点坐标。

### 4.4 门控结果

如果当前位置接近某个未扫描路线点：

```text
允许发布该点附近的稳定目标列表
active_scan_point_index = 该路线点编号
```

如果当前位置不在任何未扫描路线点附近：

```text
不发布目标给控制端
```

如果当前位置接近的路线点已经 `scanned=true`：

```text
不发布目标给控制端
或发布 skipped/already_scanned 状态用于调试
```

---

## 5. 推荐 ROS2 话题

### 5.1 视觉侧订阅

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/return/drone/world_group` | `drone_msgs/msg/WorldGroup` | 获取本次任务路线点列表 |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | 获取无人机当前实时位置 |
| `/k230/animals/capture_ready` | 自定义消息 | 控制端通知已到达拍照条件 |
| `/k230/animals/scan_point_done` | 自定义消息 | 控制端通知当前路线点全部目标处理完成 |

### 5.2 视觉侧发布

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/k230/animals/targets` | 自定义消息 | 发布未扫描路线点附近的稳定目标列表 |
| `/k230/animals/record_result` | 自定义消息 | 发布拍照成功、失败或跳过结果 |
| `/k230/animals/scan_status` | 可选自定义消息 | 发布路线点 scanned/skipped 状态，便于调试 |

### 5.3 控制端订阅

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/k230/animals/targets` | 自定义消息 | 获取需要处理的稳定目标列表 |
| `/k230/animals/record_result` | 自定义消息 | 获取拍照结果或跳过结果 |

### 5.4 控制端发布

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/k230/animals/capture_ready` | 自定义消息 | 通知视觉侧当前已经满足拍照条件 |
| `/k230/animals/scan_point_done` | 自定义消息 | 通知视觉侧当前路线点扫描完成 |

---

## 6. 目标列表语义

视觉侧发布 `/k230/animals/targets` 时，建议把消息分成两层：

```text
整帧公共信息
单个目标列表 targets[]
```

整帧公共信息描述：

```text
这一帧来自哪里
属于哪个路线点
发布时间是什么
这一帧包含多少个稳定目标
```

单个目标信息描述：

```text
这一帧里每一个动物目标的类别、编号、坐标、置信度和稳定状态
```

### 6.1 整帧公共字段

建议整帧消息包含：

```text
frame_seq
stamp
scan_point_index
scan_point_x
scan_point_y
target_count
targets[]
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `frame_seq` | 视觉帧序号，用于和控制端回传对应 |
| `stamp` | Linux 视觉节点发布该帧目标列表的 ROS2 时间戳 |
| `scan_point_index` | 当前目标列表属于哪个未扫描路线点 |
| `scan_point_x` | 当前路线点世界坐标 x |
| `scan_point_y` | 当前路线点世界坐标 y |
| `target_count` | 当前帧稳定目标数量 |
| `targets[]` | 当前帧稳定目标数组 |

这些字段属于“整帧目标列表”，不是某一个动物独有的字段。

例如：

```text
frame_seq = 100
scan_point_index = 3
scan_point_x = 1.5
scan_point_y = 0.5
target_count = 3
```

表示：

```text
第 100 帧稳定目标列表来自第 3 个路线点附近，
这个路线点坐标是 (1.5, 0.5)，
这一帧共有 3 个稳定动物目标。
```

### 6.2 单个目标字段

单个目标建议尽量保留当前检测链路里已经有用的字段。

建议目标字段包含：

```text
label
label_instance_id
score
confirmed
stable_frames

cx
cy
err_x
err_y
norm_x
norm_y

x1
y1
x2
y2
bbox_w
bbox_h
bbox_area
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `label` | 动物类别，例如 `大象`、`狼` |
| `label_instance_id` | 当前帧内同一 `label` 下的连续编号 |
| `score` | 检测置信度 |
| `confirmed` | 是否已经通过视觉端稳定确认 |
| `stable_frames` | 该目标连续稳定帧数 |
| `cx/cy` | 目标中心点像素坐标 |
| `err_x/err_y` | 目标中心相对图像中心的像素偏差 |
| `norm_x/norm_y` | 归一化偏差 |
| `x1/y1/x2/y2` | 目标框左上角和右下角像素坐标 |
| `bbox_w/bbox_h` | 目标框宽高 |
| `bbox_area` | 目标框面积 |

### 6.3 label_instance_id 命名

原始讨论里的 `label_2` 建议在正式消息中改名为：

```text
label_instance_id
```

原因是 `label_2` 容易被理解成“二级类别”或“另一个标签”，而这里真正含义是：

```text
当前帧内，同一 label 下的第几个稳定目标。
```

示例：

```text
大象 label_instance_id = 1
大象 label_instance_id = 2
狼   label_instance_id = 1
```

`label_instance_id` 不是永久 ID，也不是跨帧跟踪 ID。它只表示当前这一帧内同类目标的顺序编号。

如果早期实现或调试 JSON 中已经使用 `label_2`，可以在过渡期保留兼容字段；但 ROS2 正式消息建议统一使用 `label_instance_id`。

### 6.4 字段层级关系

`frame_seq`、`stamp`、`scan_point_index`、`scan_point_x`、`scan_point_y` 是整帧字段。

`label`、`label_instance_id`、`cx/cy`、`score` 是单个目标字段。

层级关系示例：

```text
K230AnimalTargets
  frame_seq = 100
  stamp = ...
  scan_point_index = 3
  scan_point_x = 1.5
  scan_point_y = 0.5

  targets[0]
    label = 大象
    label_instance_id = 1
    cx = 320
    cy = 210
    score = 0.91

  targets[1]
    label = 大象
    label_instance_id = 2
    cx = 500
    cy = 230
    score = 0.88

  targets[2]
    label = 狼
    label_instance_id = 1
    cx = 260
    cy = 200
    score = 0.86
```

也就是：

```text
frame_seq/stamp/scan_point_index 描述“这一帧发生在哪里”
targets[] 描述“这一帧里有哪些动物”
label/label_instance_id 描述“某一个动物是什么类别，以及它是该类别中的第几个”
```

### 6.5 建议的 drone_msgs 拆分

为了避免一个消息过大、字段语义混在一起，建议按“单目标”和“整帧数组”拆分消息。

#### K230AnimalTarget

表示单个稳定动物目标。

建议字段：

```text
string label
uint32 label_instance_id

float64 score
bool confirmed
int32 stable_frames

int32 cx
int32 cy
int32 err_x
int32 err_y
float64 norm_x
float64 norm_y

int32 x1
int32 y1
int32 x2
int32 y2
int32 bbox_w
int32 bbox_h
int32 bbox_area
```

该消息主要继承当前 `K230AnimalCenter` 中对控制和调试有价值的字段，并补充目标框信息。

#### K230AnimalTargets

表示某一帧、某个路线点附近的稳定目标列表。

建议字段：

```text
builtin_interfaces/Time stamp
uint32 frame_seq

int32 scan_point_index
float64 scan_point_x
float64 scan_point_y

uint32 target_count
K230AnimalTarget[] targets
```

该消息发布到：

```text
/k230/animals/targets
```

控制端只应处理该消息中的目标。视觉侧如果判断当前路线点已扫描，则不发布该消息，或发布空列表/调试状态。

#### K230CaptureReady

表示控制端已经把无人机带到某个目标的拍照条件。

建议字段：

```text
builtin_interfaces/Time stamp
uint32 frame_seq

int32 scan_point_index
string label
uint32 label_instance_id

bool capture_ready
```

该消息发布到：

```text
/k230/animals/capture_ready
```

`capture_ready=true` 只表示可以尝试拍照，不表示该路线点已经扫描完成。

#### K230RecordResult

表示单个目标的拍照或保存结果。

建议字段：

```text
builtin_interfaces/Time stamp
uint32 frame_seq

int32 scan_point_index
string label
uint32 label_instance_id

bool record_success
string result_state
string image_name
```

`result_state` 建议使用以下语义：

```text
captured：拍照成功
failed：拍照失败
skipped：视觉侧二次确认失败或目标已不适合拍照
```

该消息发布到：

```text
/k230/animals/record_result
```

控制端收到该消息后，只更新当前目标状态，不应直接认为整个路线点扫描完成。

#### K230ScanPointDone

表示控制端确认当前路线点内所有目标已经处理完。

建议字段：

```text
builtin_interfaces/Time stamp
int32 scan_point_index
bool scan_point_done
```

该消息发布到：

```text
/k230/animals/scan_point_done
```

视觉侧收到 `scan_point_done=true` 后，才将对应路线点标记为 `scanned=true`。

### 6.6 当前 K230AnimalCenter 字段迁移

当前单目标中心消息中的字段不建议直接丢弃。

建议迁移关系：

| 当前字段 | 新位置 | 说明 |
| --- | --- | --- |
| `stamp` | `K230AnimalTargets.stamp` 或结果类消息 `stamp` | 外层消息时间戳 |
| `seq` | `K230AnimalTargets.frame_seq` | 建议统一命名为帧序号 |
| `valid` | 可由是否发布目标、`confirmed`、`target_count` 表达 | 不一定需要单独保留 |
| `confirmed` | `K230AnimalTarget.confirmed` | 单目标稳定状态 |
| `stable_frames` | `K230AnimalTarget.stable_frames` | 单目标稳定帧数 |
| `count` | `K230AnimalTargets.target_count` | 当前帧稳定目标数量 |
| `label` | `K230AnimalTarget.label` | 单目标类别 |
| `score` | `K230AnimalTarget.score` | 单目标置信度 |
| `cx/cy` | `K230AnimalTarget.cx/cy` | 单目标中心坐标 |
| `err_x/err_y` | `K230AnimalTarget.err_x/err_y` | 单目标中心偏差 |
| `norm_x/norm_y` | `K230AnimalTarget.norm_x/norm_y` | 单目标归一化偏差 |

新增字段建议：

```text
label_instance_id
scan_point_index
scan_point_x
scan_point_y
x1/y1/x2/y2
bbox_w/bbox_h/bbox_area
```

这样既能保留当前控制端已经熟悉的中心偏差字段，又能支持多目标、路线点门控和逐目标拍照闭环。

---

## 7. label_instance_id 连续性检查

控制端收到目标列表后，需要按 `label` 分组检查 `label_instance_id` 是否连续。

正确示例：

```text
大象：1, 2, 3
狼：1
```

异常示例：

```text
大象：1, 3
```

异常含义可能是：

1. 漏检。
2. 多检。
3. 编号异常。
4. 视觉侧过滤后没有重新整理编号。

如果编号异常，控制端不应靠近、不应对准、不应发布 `capture_ready=true`。建议结束本帧处理，等待下一帧稳定目标。

---

## 8. capture_ready 语义

控制端发布：

```text
/k230/animals/capture_ready
```

建议字段：

```text
frame_seq
scan_point_index
label
label_instance_id
capture_ready
```

字段含义：

| 字段 | 说明 |
| --- | --- |
| `frame_seq` | 对应视觉侧发布的目标帧 |
| `scan_point_index` | 对应路线点编号 |
| `label` | 对应目标类别 |
| `label_instance_id` | 对应当前帧内同类目标编号 |
| `capture_ready` | 是否已经满足拍照条件 |

`capture_ready=true` 的含义是：

```text
控制端已经完成靠近、对准或到达拍照位置，
视觉侧现在可以尝试触发拍照。
```

它不表示：

```text
该路线点已经扫描完成
照片已经保存成功
应该立即把路线点标记 scanned=true
```

---

## 9. 单目标结果与路线点完成条件

视觉侧收到 `capture_ready=true` 后，不应直接把路线点标记为已扫描，而应执行二次确认并只处理对应的单个目标。

### 9.1 二次确认

视觉侧检查：

```text
scan_point_index 有效
scan_points[scan_point_index].scanned == false
当前 pose 仍然在该路线点 scan_radius 内
目标仍可匹配 frame_seq + label + label_instance_id
```

如果二次确认失败：

```text
不触发拍照
发布 record_result = skipped 或 failed
```

### 9.2 触发 K230 拍照

二次确认通过后，Linux 视觉节点通过 UART 向 K230 发送拍照指令。

K230 收到指令后：

```text
对对应目标拍照
保存或发送图片
返回拍照结果
```

### 9.3 单个目标处理结果

K230 返回拍照结果后，Linux 视觉节点发布：

```text
/k230/animals/record_result
```

成功结果语义：

```text
scan_point_index
label
label_instance_id
record_success = true
image_name
```

失败或跳过结果语义：

```text
scan_point_index
label
label_instance_id
record_success = false
result_state = failed 或 skipped
```

单个目标拍照成功，只表示这个目标处理完成，不表示整个路线点已经扫描完成。

### 9.4 当前路线点全部目标完成

控制端在收到 `/k230/animals/targets` 后，应为当前 `scan_point_index` 建立目标处理列表。

例如：

```text
scan_point_index = 3
大象 label_instance_id = 1 -> unprocessed
大象 label_instance_id = 2 -> unprocessed
狼   label_instance_id = 1 -> unprocessed
```

控制端逐个处理这些目标。

目标状态建议为：

```text
unprocessed：尚未处理
tracking：正在靠近或对准
captured：已经拍照成功
skipped：跳过
failed：处理失败
```

其中终态为：

```text
captured
skipped
failed
```

只有当前路线点内所有目标都进入终态后，控制端才认为该路线点扫描完成。

### 9.5 scan_point_done

当当前路线点内所有目标都进入终态后，控制端发布：

```text
/k230/animals/scan_point_done
```

建议字段：

```text
scan_point_index
scan_point_done
```

视觉侧收到：

```text
scan_point_done = true
```

之后，才标记：

```text
scan_points[scan_point_index].scanned = true
```

这样可以避免以下错误：

```text
路线点内有 3 个动物，
拍完第 1 个动物后就把整个路线点标记 scanned=true，
导致后面 2 个动物永远不会被处理。
```

---

## 10. 完整运行流程

### 10.1 初始化

```text
1. 地面站或控制端生成路线点
2. route_comm_node 发布 /return/drone/world_group
3. Linux 视觉节点订阅并缓存路线点
4. 所有路线点 scanned=false
5. Linux 视觉节点订阅 /mavros/local_position/pose
```

### 10.2 无人机第一次进入某路线点

```text
1. 当前 pose 接近 scan_point[3]
2. scan_point[3].scanned == false
3. K230 检测到稳定动物目标
4. Linux 视觉节点允许发布 /k230/animals/targets
5. 目标消息携带 scan_point_index = 3
```

### 10.3 控制端处理目标

```text
1. 控制端订阅 /k230/animals/targets
2. 检查 label_instance_id 连续性
3. 为当前 scan_point_index 建立目标处理列表
4. 选择一个未处理目标
5. 对该目标执行靠近、对准或到达拍照位置
6. 到达拍照条件后发布 /k230/animals/capture_ready
```

### 10.4 视觉侧拍照

```text
1. Linux 视觉节点收到 capture_ready=true
2. 二次确认 scan_point[3].scanned == false
3. 二次确认当前 pose 仍在 scan_point[3] 半径内
4. Linux 视觉节点向 K230 转发拍照指令
5. K230 拍照成功
6. Linux 视觉节点发布 /k230/animals/record_result
7. 控制端把该目标标记为 captured
```

### 10.5 当前路线点扫描完成

```text
1. 控制端继续处理当前路线点内剩余目标
2. 每个目标都通过 record_result 进入 captured/skipped/failed
3. 控制端判断当前路线点内所有目标均为终态
4. 控制端发布 /k230/animals/scan_point_done
5. Linux 视觉节点收到 scan_point_done=true
6. Linux 视觉节点标记 scan_point[3].scanned = true
7. 控制端恢复航线，飞向下一个路线点
```

### 10.6 无人机第二次经过同一路线点

```text
1. 当前 pose 再次接近 scan_point[3]
2. scan_point[3].scanned == true
3. 即使 K230 再次识别到动物
4. Linux 视觉节点也不发布 /k230/animals/targets
5. 控制端不会收到需要处理的目标
6. 无人机不会再次靠近、对准或拍照
```

---

## 11. 后续重复动物问题的边界

该方案解决的是：

```text
同一个路线点重复经过时，不再重复发布目标和触发拍照。
```

它不能完全解决：

```text
无人机在另一个未扫描路线点，斜向看到之前已经拍过的同一只动物。
```

因为本方案的判重依据是：

```text
路线点 index + 当前无人机 pose
```

不是：

```text
动物的世界坐标
```

如果后续必须解决“不同路线点看到同一只动物”的问题，需要增加动物位置估计，例如：

```text
animal_world_x
animal_world_y
label
```

然后按：

```text
label 相同
且 distance(current_animal_world, recorded_animal_world) < R_target
```

做目标级判重。

当前阶段建议先实现路线点级门控，因为它改动集中、验证路径清晰，能先解决固定路线下同一点重复扫描的问题。

---

## 12. 配套修改范围

### 12.1 K230 端 Python 脚本

K230 端需要支持：

1. 发布稳定多目标列表，而不是只发布单个最佳目标。
2. 为同一帧同一 `label` 的目标分配连续 `label_instance_id`。
3. 接收 Linux 视觉节点通过 UART 下发的拍照指令。
4. 只在收到拍照指令后拍照。
5. 拍照完成后通过 UART 返回结果。

### 12.2 Linux UART ROS2 桥接节点

Linux 视觉节点需要支持：

1. 订阅 `/return/drone/world_group`。
2. 订阅 `/mavros/local_position/pose`。
3. 缓存路线点并维护 `scanned` 状态。
4. 解析 K230 发来的稳定目标列表。
5. 只在未扫描路线点附近发布 `/k230/animals/targets`。
6. 订阅 `/k230/animals/capture_ready`。
7. 订阅 `/k230/animals/scan_point_done`。
8. 将拍照指令通过 UART 发给 K230。
9. 解析 K230 单个目标的拍照结果。
10. 发布 `/k230/animals/record_result`。
11. 收到 `scan_point_done=true` 后标记路线点 `scanned=true`。

### 12.3 drone_msgs 消息定义

需要新增或调整消息，用于表达：

1. `K230AnimalTarget`：单个稳定目标。
2. `K230AnimalTargets`：一帧稳定多目标列表。
3. `K230CaptureReady`：控制端拍照就绪通知。
4. `K230RecordResult`：视觉端单目标拍照结果。
5. `K230ScanPointDone`：控制端通知当前路线点全部目标处理完成。
6. 可选 `K230ScanStatus`：路线点 scanned/skipped 调试状态。

### 12.4 控制端

控制端需要支持：

1. 订阅 `/k230/animals/targets`。
2. 检查 `label_instance_id` 连续性。
3. 控制无人机靠近、对准或到达拍照位置。
4. 到位后发布 `/k230/animals/capture_ready`。
5. 订阅 `/k230/animals/record_result`。
6. 根据 `record_result` 更新当前目标状态。
7. 当前路线点内所有目标都进入终态后发布 `/k230/animals/scan_point_done`。
8. 发布 `scan_point_done` 后恢复航线，前往下一个路线点。

### 12.5 推荐实现顺序

建议顺序：

```text
1. 先确定 drone_msgs 消息接口
2. 修改 Linux UART ROS2 桥接节点，加入路线点门控和 scanned 状态
3. 修改 K230 Python 端，改为接收拍照指令后再拍照
4. 修改控制端，订阅 targets 并发布 capture_ready
5. 最后补充 scan_status 调试话题和日志
```

消息接口先稳定后，K230、Linux 视觉节点和控制端才能按同一套语义实现。
