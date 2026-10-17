# K230 视觉端当前状态与控制端开发交接

## 1. 文档目的

本文只说明当前阶段需要交接给控制端的内容：

```text
视觉端当前已经能发布什么
控制端下一步要怎么处理这些数据
控制端需要交付哪些话题和字段给视觉端
控制端提交后，视觉端再继续补什么
```

本文不展开 K230 板端细节，也不重复完整闭环文档。

---

## 2. 当前视觉端已经完成的任务

### 2.1 新增多目标列表发布

视觉端当前已经可以发布稳定多目标列表：

```text
话题名：/k230/animals/targets
消息类型：drone_msgs/msg/K230AnimalTargets
发布方：Linux 视觉节点
订阅方：控制端
```

视觉端发布该话题前，已经做了路线点门控：

```text
K230 发来稳定目标列表
  ↓
Linux 视觉节点检查当前无人机是否进入未扫描路线点半径
  ↓
不在路线点半径内：不发布 /k230/animals/targets
在路线点半径内：填入 scan_point_index/x/y 并发布 /k230/animals/targets
```

当前路线点半径：

```text
scan_radius_m = 0.2
```

含义：

```text
无人机距离某个路线点中心 0.2 m 以内，才认为进入该 scan point。
```

控制端只需要处理 `/k230/animals/targets`，不需要再自己判断当前目标是否属于路线点。视觉端已经把路线点编号填进消息里。

---

## 3. 视觉端发布的消息内容

### 3.1 K230AnimalTarget.msg

该消息表示一个动物目标。

```msg
string label  # 动物类别，例如“大象”“狼”“老虎”。
uint32 label_instance_id  # 当前帧内同一 label 下的连续编号，从 1 开始；不是永久 ID。

float64 score  # 检测置信度，通常为 0.0-1.0。
bool confirmed  # 是否已经通过 K230 视觉端多帧稳定确认。
int32 stable_frames  # 当前目标连续稳定帧数。

int32 cx  # 目标中心点像素 x 坐标。
int32 cy  # 目标中心点像素 y 坐标。
int32 err_x  # 目标中心相对图像中心的水平像素偏差。
int32 err_y  # 目标中心相对图像中心的垂直像素偏差。

float64 norm_x  # 归一化水平中心偏差，便于控制端使用。
float64 norm_y  # 归一化垂直中心偏差，便于控制端使用。

int32 x1  # 目标框左上角 x 坐标。
int32 y1  # 目标框左上角 y 坐标。
int32 x2  # 目标框右下角 x 坐标。
int32 y2  # 目标框右下角 y 坐标。
int32 bbox_w  # 目标框宽度。
int32 bbox_h  # 目标框高度。
int32 bbox_area  # 目标框面积。
```

控制端重点理解以下字段：

```text
label：动物类别。
label_instance_id：该帧内同类动物编号。例如“大象(label_instance_id=1)”表示当前帧第 1 个大象。
err_x/err_y：目标中心相对图像中心的像素偏差，可用于对准。
norm_x/norm_y：归一化偏差，可用于比例控制。
bbox_area：目标框面积，只能粗略反映目标在画面中变大或变小，不能当作精确距离。
```

### 3.2 K230AnimalTargets.msg

该消息表示某一帧稳定目标列表。

```msg
builtin_interfaces/Time stamp  # Linux 视觉节点发布该帧目标列表的时间戳。
uint32 frame_seq  # K230/视觉端发布的目标列表帧序号，用于和控制端回传命令对应。

int32 scan_point_index  # 当前目标列表对应的路线点编号。
float64 scan_point_x  # 当前路线点世界 x 坐标。
float64 scan_point_y  # 当前路线点世界 y 坐标。

uint32 target_count  # 当前帧稳定目标数量，应与 targets 数组长度一致。
K230AnimalTarget[] targets  # 当前帧稳定动物目标列表。
```

控制端重点理解以下字段：

```text
stamp：Linux 视觉节点发布时间。
frame_seq：目标列表帧序号，控制端发布 capture_ready 时需要带回。
scan_point_index：目标列表属于哪个路线点，控制端以它为单位建立目标队列。
scan_point_x/scan_point_y：路线点世界坐标，不是动物世界坐标。
target_count：当前帧目标数量，必须等于 targets.size()。
targets：该路线点当前帧内的稳定动物目标数组。
```

---

## 4. 控制端下一步要做什么

### 4.1 控制端第一步：订阅目标列表

控制端需要订阅：

```text
/k230/animals/targets
drone_msgs/msg/K230AnimalTargets
```

收到消息后，先不要立刻控制无人机，先做校验和入队。

### 4.2 控制端第二步：校验当前帧

控制端收到一帧目标列表后，按以下顺序检查。

第一，检查路线点是否有效：

```text
scan_point_index >= 0
```

第二，检查数量是否一致：

```text
target_count == targets.size()
```

第三，检查同一个 `label` 下的 `label_instance_id` 是否连续。

正确示例：

```text
大象：1, 2, 3
狼：1
```

异常示例：

```text
大象：1, 3
```

如果编号不连续，可能说明当前帧有：

```text
漏检
多检
编号异常
```

控制端此时不要把该帧整组目标入库，应等待后续帧继续确认。

### 4.3 控制端第三步：建立目标队列

控制端以 `scan_point_index` 为单位维护目标队列。

例如收到：

```text
frame_seq = 12
scan_point_index = 3
target_count = 3

targets:
  label = 大象, label_instance_id = 1
  label = 大象, label_instance_id = 2
  label = 狼,   label_instance_id = 1
```

控制端内部应建立：

```text
active_scan_point_index = 3

target_queue:
  大象(label_instance_id=1) pending
  大象(label_instance_id=2) pending
  狼(label_instance_id=1) pending
```

建议控制端内部目标 key 使用：

```text
scan_point_index + frame_seq + label + label_instance_id
```

字段含义：

```text
scan_point_index：该目标属于哪个路线点
frame_seq：该目标来自哪一帧目标列表
label：动物类别
label_instance_id：该帧内同类目标编号
```

### 4.4 控制端第四步：选择当前要处理的目标

控制端从 `target_queue` 中选择一个 `pending` 目标作为当前目标。

目标状态建议至少包含：

```text
pending：已发现，尚未处理
tracking：正在靠近或对准
capture_requested：已经发布 capture_ready，等待视觉端拍照结果
captured：视觉端返回拍照成功
failed：视觉端返回拍照失败，或控制端超时
skipped：控制端主动跳过
```

### 4.5 控制端第五步：控制无人机靠近或对准

控制端可使用目标字段：

```text
err_x
err_y
norm_x
norm_y
cx
cy
bbox_area
```

常用判断方式：

```text
abs(err_x) 小于阈值：水平基本对准
abs(err_y) 小于阈值：垂直基本对准
bbox_area 达到阈值或无人机到达预定位置：认为距离条件满足
```

控制端应根据自己的飞控策略决定何时满足拍照条件。

注意：

```text
控制端不要和已有 mission_controller_node / ActionExecutor 同时抢发 /mavros/setpoint_position/local。
```

如果需要真正控制无人机运动，应接入现有控制执行体系，避免两个控制器同时发 setpoint。

### 4.6 控制端第六步：发布 capture_ready

当控制端判断当前目标已经满足拍照条件后，发布：

```text
/k230/animals/capture_ready
```

建议消息字段：

```text
uint32 frame_seq
int32 scan_point_index
string label
uint32 label_instance_id
bool capture_ready
```

字段含义：

```text
frame_seq：目标来自哪一帧
scan_point_index：目标属于哪个路线点
label：动物类别
label_instance_id：该帧内同类目标编号
capture_ready：true 表示控制端已经到达拍照条件
```

示例：

```text
frame_seq = 12
scan_point_index = 3
label = 大象
label_instance_id = 1
capture_ready = true
```

这条消息是控制端当前阶段最重要的交付结果。

---

## 5. 控制端需要交付给视觉端的内容

控制端代码提交时，需要交付清楚以下内容。

### 5.1 capture_ready 话题

必须明确：

```text
话题名
消息类型
字段含义
什么时候发布
是否只发布一次
是否会重复发布
```

建议：

```text
话题名：/k230/animals/capture_ready
发布时机：当前目标达到拍照条件时
发布频率：对同一个目标只发布一次
```

### 5.2 控制端目标状态机

控制端需要说明目标状态如何变化。

推荐流程：

```text
pending
  -> tracking
  -> capture_requested
  -> captured / failed / skipped
```

### 5.3 scan_point_done 话题

该话题用于通知视觉端：

```text
当前 scan point 内所有目标都已经处理完。
```

建议话题名：

```text
/k230/animals/scan_point_done
```

临时简单版本可以用：

```text
std_msgs/msg/Int32
```

含义：

```text
data = scan_point_index
```

正式版本建议用自定义消息：

```text
int32 scan_point_index
bool scan_point_done
```

控制端可以先完成 `capture_ready`，然后再继续补 `scan_point_done`。

---

## 6. 控制端流程完整示例

假设视觉端发布：

```text
frame_seq = 12
scan_point_index = 3
scan_point_x = 1.5
scan_point_y = 0.5
target_count = 2

targets:
  大象(label_instance_id=1)
  狼(label_instance_id=1)
```

控制端处理流程：

```text
1. 收到 /k230/animals/targets
2. 检查 scan_point_index = 3 有效
3. 检查 target_count = 2，targets.size() = 2
4. 检查 label_instance_id 连续
5. 建立 scan_point 3 的目标队列
6. 选择 大象(label_instance_id=1)
7. 控制无人机靠近或对准 大象(label_instance_id=1)
8. 满足拍照条件
9. 发布 /k230/animals/capture_ready
10. 等待视觉端后续返回 /k230/animals/record_result
11. 大象(label_instance_id=1) 进入终态
12. 继续处理 狼(label_instance_id=1)
13. 所有目标终态后，发布 /k230/animals/scan_point_done
```

---

## 7. 控制端提交后视觉端要继续做什么

控制端交付 `capture_ready` 后，视觉端继续补以下内容。

### 7.1 视觉端订阅 capture_ready

视觉端收到：

```text
/k230/animals/capture_ready
```

后执行：

```text
1. 检查 capture_ready == true
2. 检查 scan_point_index 是否有效
3. 记录当前 /mavros/local_position/pose
4. 通过 UART type 4 通知 K230 拍照
```

### 7.2 视觉端发布 record_result

K230 拍照完成后，Linux 视觉节点解析 UART type 5，并发布：

```text
/k230/animals/record_result
```

建议字段：

```text
uint32 frame_seq
int32 scan_point_index
string label
uint32 label_instance_id
bool record_success
string result_state
string image_name
```

### 7.3 视觉端接收 scan_point_done

控制端完成当前路线点所有目标后，发布：

```text
/k230/animals/scan_point_done
```

视觉端收到后：

```text
scan_points[scan_point_index].scanned = true
```

之后无人机再次进入该路线点半径时，视觉端不再发布该点的目标列表。

---

## 8. 当前阶段结论

视觉端当前已经完成：

```text
/k230/animals/targets 发布
K230AnimalTarget / K230AnimalTargets 消息转换
scan_point_index/x/y 填充
路线点半径门控
```

控制端下一步重点是：

```text
订阅 /k230/animals/targets
校验 target_count
校验 label_instance_id 连续性
建立目标队列
控制无人机靠近或对准目标
达到拍照条件后发布 /k230/animals/capture_ready
```

控制端交付 `capture_ready` 后，视觉端再继续实现：

```text
capture_ready 接收
当前位置记录
K230 拍照命令转发
record_result 发布
scan_point_done 接收和 scanned=true 标记
```
