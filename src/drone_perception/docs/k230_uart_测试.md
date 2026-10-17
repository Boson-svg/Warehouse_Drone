# K230 UART 动物识别测试说明

## 1. 目的

验证 K230 端动物识别通过 UART 向 ROS2 发送检测结果、目标中心数据和抓拍图片，并确认 ROS2 侧可以正常订阅图片数据。

## 2. 相关话题

- `/k230/animals/detect`
  - 类型：`std_msgs/msg/String`
  - 内容：原始检测 JSON

- `/k230/animals/center`
  - 类型：`drone_msgs/msg/K230AnimalCenter`
  - 内容：时间戳、序号、类别、置信度、像素坐标、误差值、归一化坐标等

- `/drone/image`
  - 类型：`drone_msgs/msg/BarcodeCapture`
  - 内容：JPEG 图片二进制数据

## 3. 新增消息说明

### 3.1 `drone_msgs/msg/K230AnimalCenter`

该消息用于 `/k230/animals/center`，负责发布动物目标的结构化中心坐标数据。

```text
builtin_interfaces/Time stamp

uint32 seq
bool valid
bool confirmed
int32 stable_frames
uint32 count

string label
float64 score

int32 cx
int32 cy
int32 err_x
int32 err_y

float64 norm_x
float64 norm_y
```

字段说明：

- `stamp`：ROS2 接收到有效检测时的时间戳
- `seq`：K230 UART 数据包序号
- `valid`：当前检测是否有效
- `confirmed`：是否已经通过稳定检测确认
- `stable_frames`：连续稳定检测帧数
- `count`：当前检测目标数量
- `label`：动物类别
- `score`：识别置信度
- `cx/cy`：识别框中心像素坐标
- `err_x/err_y`：目标中心相对图像中心的像素误差
- `norm_x/norm_y`：归一化误差，适合控制节点使用

### 3.2 `drone_msgs/msg/BarcodeCapture`

该消息用于 `/drone/image`，负责发布 K230 抓拍后的 JPEG 图片数据。

```text
builtin_interfaces/Time stamp

string barcode
string image_format
uint8[] image_data
```

字段说明：

- `stamp`：ROS2 发布图片消息时的时间戳
- `barcode`：图片名称，例如 `k230_peacock_0003`
- `image_format`：图片格式，当前为 `jpeg`
- `image_data`：JPEG 原始二进制数据

## 4. 编译

```bash
cd ~/drone_ws
colcon build --packages-select drone_msgs drone_perception
source install/setup.bash
```

## 5. 启动 ROS2 UART 节点

```bash
ros2 run drone_perception k230_animals_uart_ros2_node
```

默认参数：

- 串口：`/dev/ttyUSB0`
- 波特率：`921600`
- 图片话题：`/drone/image`
- 检测话题：`/k230/animals/detect`
- 中心话题：`/k230/animals/center`

## 6. 测试步骤

### 6.1 检查消息类型

```bash
ros2 interface show drone_msgs/msg/K230AnimalCenter
```

### 6.2 查看检测结果

```bash
ros2 topic echo /k230/animals/detect
```

预期：

- 能看到 K230 发来的检测 JSON
- JSON 内包含 `valid`、`label`、`score`、`cx`、`cy`、`err_x`、`err_y`、`norm_x`、`norm_y`

### 6.3 查看中心数据

```bash
ros2 topic echo /k230/animals/center
```

预期：

- `valid=true` 时才会发布
- 能看到 `stamp`
- 能看到 `cx/cy`
- 能看到 `err_x/err_y`
- 能看到 `norm_x/norm_y`
- 能看到 `label`、`score`、`confirmed`、`stable_frames`

### 6.4 查看图片消息

```bash
ros2 topic echo /drone/image --field barcode
```

预期：

- 只有从无效识别切换到有效识别时才会触发抓拍
- 不会在持续有效时每帧都发图片

## 7. 已知行为

- 多目标情况下，当前逻辑只抓拍最佳稳定目标。
- `/k230/animals/center` 是结构化消息，不需要再解析字符串。
- `/drone/image` 发布的是 JPEG 原始二进制数据，是否保存图片由订阅端自行处理。

## 8. 常见问题

### 8.1 话题没有数据

- 检查 K230 是否已经接上串口
- 检查 `/dev/ttyUSB0` 是否正确
- 检查是否已经 `source install/setup.bash`

### 8.2 话题类型不对

- 先关闭旧节点
- 重新编译并 `source install/setup.bash`
- 再重新启动 ROS2 节点
