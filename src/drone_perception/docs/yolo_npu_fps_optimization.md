# YOLO RKNN 推理 FPS 优化笔记

## 背景现象

当前 YOLO/RKNN 节点运行后，监控里能看到 NPU 总体负载接近或达到 `100%`，但画面 FPS 仍然只有十几帧。这个现象不能直接判断为异常：

- `NPU load=100` 只说明 NPU devfreq 设备整体很忙，不等价于 `core0/core1/core2` 三个 NPU 核心都各自满载。
- 没有检测框不代表模型没跑。只要每帧进入 `rknn_run()`，即使最终没有目标框，NPU 仍然会被占用。
- INT8 模型也可能显示 `NPU load=100`。INT8 的意义是降低单帧推理耗时，不是让 NPU load 变低。
- 如果相机输入 topic 只有十几 Hz，YOLO 输出 FPS 会被输入流限制。

因此优化前要同时看：

```text
输入 topic Hz
端到端 FPS
preprocess 耗时
rknn_run 耗时
postprocess 耗时
NPU 总体 load
NPU 三核心 load
YOLO 进程 CPU/RSS
温度和频率
```

## 当前实测结论

### 2026-06-10 分段耗时

测试条件：开启视频流，打开 `debug_view`，节点内部已加入 detector 分段计时。

| 阶段 | 耗时 |
| --- | ---: |
| `preprocess_ms` | `0.97 ms` |
| `input_set_ms` | `0.34 ms` |
| `rknn_run_ms` | `28.52 ms` |
| `output_get_ms` | `0.16 ms` |
| `postprocess_ms` | `0.20 ms` |
| `detector_total_ms` | `30.20 ms` |

换算：

```text
detector 理论上限 FPS = 1000 / 30.20 ≈ 33.1 FPS
rknn_run 占 detector_total 比例 = 28.52 / 30.20 ≈ 94.4%
```

结论：

```text
当前主要瓶颈是 NPU 推理本身，即 rknn_run。
preprocess / input_set / output_get / postprocess 都不是当前主要瓶颈。
```

因此当前优先级应调整为：

| 优化项 | 当前优先级 | 原因 |
| --- | --- | --- |
| NPU 频率 / governor 检查 | 高 | `rknn_run_ms` 占比最高，先确认频率是否稳定 |
| `rknn_set_core_mask` | 高 | 直接影响 RKNN context 如何使用 NPU core |
| `RKNN_FLAG_PRIOR_HIGH` | 中 | 可能降低 NPU 调度等待和抖动 |
| FP16 / INT8 / hybrid 模型对比 | 高 | 判断 bbox FP16 是否显著增加 `rknn_run_ms` |
| 降低输入尺寸 / 更小模型 | 高 | 若要明显超过当前 FPS，上限主要由模型计算量决定 |
| RGA 预处理 | 低 | 当前 `preprocess_ms≈0.97ms`，收益空间小 |
| zero-copy 输入 | 低 | 当前 `input_set_ms≈0.34ms`，收益空间小 |
| NMS / 后处理优化 | 低 | 当前 `postprocess_ms≈0.20ms`，收益空间小 |
| `want_float=0` raw output 后处理 | 低 | 当前 `output_get_ms + postprocess_ms` 很小，收益有限 |

## GitHub 可借鉴点

### leafqycc/rknn-cpp-Multithreading

仓库地址：

```text
https://github.com/leafqycc/rknn-cpp-Multithreading
```

这个仓库是 C++ OpenCV 视频推理 demo，不是 ROS2 节点。它的核心目标是提高视频推理吞吐，不是控制场景下的低延迟结果发布。

可借鉴点：

| 机制 | 仓库做法 | 对当前项目的适用性 |
| --- | --- | --- |
| 多 RKNN context | `rknnPool` 中为多个 worker 创建多个模型对象 | 可借鉴，但要改成 ROS2 最新帧队列 |
| 权重复用 | 使用 `rknn_dup_context(ctx_in, &ctx)` | 可借鉴，适合多 context 降低模型内存开销 |
| NPU core 绑定 | 每个 context 调用 `rknn_set_core_mask` 绑定不同 core | 可借鉴，适合测试三核吞吐 |
| 线程池异步 | `ThreadPool` + `std::future` FIFO 队列 | 不能直接照搬，FIFO 可能导致控制拿到旧帧 |
| CPU/NPU 定频 | `performance.sh` 锁 CPU/NPU 频率 | 适合基准测试和对比模型 |
| RGA | 用 RGA 做 resize 预处理 | 只有 `preprocess_ms` 明显偏高时才值得做 |
| `want_float=0` | 后处理直接使用 int8 输出和 scale/zp | 后处理瓶颈明显时再考虑 |
| ReLU 模型 | 使用 RKNN 友好的 YOLOv5s ReLU 模型 | 属于模型训练/导出侧改造，不能直接套到当前 YOLO 模型 |

不建议直接照搬的点：

```text
1. FIFO future 队列会积压旧帧，不适合无人机实时控制。
2. demo FPS 代表视频吞吐，不代表 ROS2 + D435i + depth 同步端到端 FPS。
3. ReLU 优化需要模型结构和训练/导出配合，不能直接修改当前 RKNN 文件得到。
4. 仓库里的 RGA 是 resize 预处理，不是 RGA 加速画框。
```

如果后续要做多 context，应采用本文“多 RKNN context / 多 worker”中的实时策略：

```text
输入队列只保留最新帧
结果带 timestamp/frame_id
旧结果直接丢弃
严禁无限队列积压
```

### hannahrepo/rk3588-npu

这个仓库偏 RK3588 NPU 压测和运行时优化，不能直接替换 ROS2 YOLO 节点，但可以借鉴机制。

| 文件 | 可借鉴点 | 适合程度 |
| --- | --- | --- |
| `main.cc` | `RKNN_FLAG_PRIOR_HIGH`、`rknn_set_core_mask`、`rknn_create_mem`、`rknn_set_io_mem`、`pass_through=1`、NPU/DDR 频率读取 | 适合借机制，不适合直接照搬 |
| `board_stress_test_6p.py` | 6 进程压测，`CORES = [1, 2, 4]`，每个 NPU core 两个进程 | 只适合压测上限，不适合实时控制 |
| `board_stress_test_16t.py` | 单 NPU core 多线程压测 | 只适合观察单核压力 |
| `pc_remote_test_v1.py` | `perf_debug=True`、`eval_perf()`、`eval_memory()` | 适合单模型/单图性能剖析 |
| `fix_freq_rk3588.sh` | 锁 CPU/NPU/GPU/DDR 频率 | 适合性能测试前稳定频率 |

### VaporTang/yolo-vision-pipeline-rknn

这个仓库偏 YOLO 训练、ONNX 导出、RKNN 转换，主要可借鉴模型侧和转换侧配置。

| 配置/流程 | 可借鉴点 |
| --- | --- |
| `export_config.yaml` | RKNN 导出推荐 `opset_version: 12`，`simplify: false` |
| `rknn_config.yaml` | `do_quantization: true`、`target_platform: rk3588`、`optimization_level: 3` |
| `prepare_calibration` | INT8 量化需要代表性校准图像 |
| `train_config.yaml` | 通过 `yolov8n/s/m`、`imgsz` 控制模型计算量 |

### Vanishi/BXC_VideoAnalyzer_v3 和 BXC_AutoML

`BXC_VideoAnalyzer_v3` 的说明里提到 C++ RKNPU YOLO 推理、RGA 预处理加速、支持 `rk3588/rk3576`、异常行为检测和报警视频合成。它更像完整视频分析系统，可借鉴运行时管线，但公开仓库不等于完整可直接移植源码。

`BXC_AutoML` 更偏训练和 RKNN 转换流程，定位类似自动化训练/导出/转换框架。它不是 Rockchip 官方项目，但如果内部调用 RKNN-Toolkit2，那么实际转换工具链仍然是 Rockchip 官方 RKNN-Toolkit2。

| 项目 | 可借鉴点 | 适合程度 |
| --- | --- | --- |
| `BXC_VideoAnalyzer_v3` | C++ RKNPU 推理、RGA 预处理、视频分析管线、报警视频合成 | 适合借架构和优化方向 |
| `BXC_AutoML` | YOLO 训练、ONNX/RKNN 转换、INT8 量化流程 | 适合借模型转换流程 |

## 可测试参数

新增脚本：

```bash
./src/drone_perception/scripts/monitor_yolo_npu_perf.sh
```

先启动 YOLO 推理节点，再另开终端运行：

```bash
./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --interval 0.5 --show-raw
```

需要读取 debugfs 三核心信息时：

```bash
sudo ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --interval 0.5 --show-raw
```

对比 FP16 和 INT8 时，建议各跑 30 秒：

```bash
./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --duration 30 --interval 0.5 --no-clear --show-raw
```

脚本能采样：

- YOLO 进程 `pid`
- YOLO 进程 CPU
- YOLO 进程 RSS
- YOLO 进程线程数
- NPU devfreq 总体 load
- NPU 当前频率
- NPU governor
- debugfs 三核心占用，前提是 `/sys/kernel/debug/rknpu/load` 可读
- 温度
- rknpu 设备占用者

脚本不能从外部直接测：

- RKNN 每层耗时
- 单次 `rknn_run()` 精确耗时
- YOLO 后处理耗时

这些需要在 C++ 节点内部打点，或使用 RKNN Toolkit2 的 `perf_debug/eval_perf` 单独剖析模型。

## 完整操作和测试流程

优化必须按“测量 -> 判断 -> 修改 -> 复测”闭环执行。不要只看一个 `NPU load=100` 就直接改架构。

### 步骤 1：建立基线

先固定一个测试场景，保证 FP16、INT8、小模型、小输入尺寸都使用同一段视频流或同一组现场画面。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

同时运行：

```bash
./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --duration 30 --interval 0.5 --no-clear --show-raw
```

记录：

```text
输入 topic Hz
端到端 FPS
YOLO 进程 CPU/RSS
NPU load/freq
NPU 三核心占用
温度
```

### 步骤 2：加节点内分段计时

在 C++ 节点里记录：

```text
preprocess_ms
rknn_run_ms
postprocess_ms
total_callback_ms
```

判断分支：

```text
preprocess_ms 高 -> 优先考虑 RGA / 输入尺寸 / letterbox 优化
rknn_run_ms 高 -> 优先考虑 INT8 / 小模型 / core mask / high priority
postprocess_ms 高 -> 优先优化候选框遍历 / NMS / 阈值 / 输出格式
输入 Hz 低 -> 优先处理相机流和同步
```

### 步骤 3：按瓶颈选择优化

```text
NPU 推理瓶颈:
  检查 NPU 频率/governor
  测试 RKNN_NPU_CORE_0_1_2
  测试 RKNN_FLAG_PRIOR_HIGH
  对比 FP16 / INT8 / hybrid 的 rknn_run_ms
  INT8
  更小模型
  更小输入尺寸
  多 RKNN context

预处理瓶颈:
  减少输入尺寸
  简化 letterbox
  RGA resize/cvtColor/fill
  后续再考虑 RGA buffer -> RKNN input memory

后处理瓶颈:
  提高阈值减少候选框
  优化 NMS
  避免每帧画框和 clone
  减少 want_float 反量化开销
```

结合当前实测数据，当前分支属于：

```text
NPU 推理瓶颈
rknn_run_ms = 28.52ms
rknn_run 占 detector_total 约 94.4%
```

因此当前不应优先投入 RGA、zero-copy 或 NMS 优化，除非后续新计时显示这些阶段重新变成瓶颈。

### 步骤 4：单项改动后复测

每次只改一个变量，例如只改 `debug_view`、只改 INT8、只改输入尺寸、只改 core mask。每次都跑同样的 30 秒采样，避免混合多个改动后无法判断收益来源。

## 优化机制

### 1. 分段计时

最先要加的是推理链路分段耗时：

```text
preprocess: resize / letterbox / cvtColor
rknn_run:   真正 RKNN/NPU 推理
postprocess: 输出解析 / 阈值过滤 / NMS / 画框
```

判断逻辑：

```text
rknn_run 占大头 -> 优先优化模型/NPU runtime
preprocess 占大头 -> 优化 resize、letterbox、颜色转换、输入尺寸
postprocess 占大头 -> 优化候选框解析、阈值、NMS、画框
```

### 2. 输入流检查

如果输入流只有约 `15 Hz`，YOLO 输出 `14 FPS` 可能已经接近上限。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

若输入 topic 不稳定，优先处理相机帧率、depth 对齐和 message_filters 同步等待。

### 3. 关闭调试显示

性能测试时先关闭窗口显示和画框：

```bash
ros2 run drone_perception qr_vision_node --ros-args -p debug_view:=false
```

可视化会引入图像 clone、绘制、窗口刷新等 CPU/内存带宽开销。

### 4. INT8 量化

INT8 是 RK3588 上最重要的模型侧优化之一。判断 INT8 是否有效，应比较：

```text
FP16 rknn_run 单帧耗时
INT8 rknn_run 单帧耗时
FP16 端到端 FPS
INT8 端到端 FPS
```

转换侧建议：

```yaml
opset_version: 12
simplify: false
do_quantization: true
target_platform: rk3588
mean_values: [[0, 0, 0]]
std_values: [[255, 255, 255]]
optimization_level: 3
```

校准集要使用有代表性的真实场景图像。校准图像不代表实际输入分布时，INT8 可能速度提升但精度下降明显。

### 5. 降低模型计算量

如果 `rknn_run` 是主要瓶颈，最直接的优化是降低每帧计算量：

- `640 -> 416` 或 `320`
- `yolov8s -> yolov8n`
- 减少类别数
- ROI 裁剪，只对重点区域推理
- 隔帧推理，例如两帧推理一次

对 NPU 满载场景，降低输入尺寸通常比多线程更直接。

### 5.1 ReLU 激活函数优化

如果模型结构仍然是 `SiLU/Swish` 这类较复杂激活函数，可以考虑在训练阶段改成更适合 RKNN/NPU 的 `ReLU` 或 `ReLU6`。它的目标不是改代码，而是改模型结构，让编译器更容易做算子融合，降低 `rknn_run_ms`。

大致流程：

```text
原始数据集
-> 训练得到 SiLU/Swish 版本 best.pt
-> 替换激活函数为 ReLU/ReLU6
-> 用原数据集继续微调
-> 导出 ReLU 版本 ONNX
-> 再转 RKNN
```

适用前提：

```text
1. 当前模型确实存在激活函数瓶颈。
2. 能接受重新训练或至少微调。
3. 愿意重新比较精度和速度。
```

不建议直接在已有 ONNX/RKNN 上硬改激活函数，因为权重分布是按原激活函数训练出来的，直接替换可能让检测精度明显下降。

### 6. RGA 预处理加速

RGA 是 Rockchip SoC 上的 2D 图像硬件加速模块，适合加速 YOLO 推理前的图像预处理。它不是 NPU，也不是 GPU，主要处理：

```text
resize
crop
color convert
rotate / flip
copy
fill
blend
```

YOLO 预处理通常包含：

```text
相机图像
-> resize / letterbox 到模型输入尺寸
-> BGR/RGB/YUV 颜色格式转换
-> NHWC uint8
-> 送入 RKNN
```

如果这些步骤都由 OpenCV CPU 完成，会消耗 CPU 和内存带宽。RGA 的目标是把 `resize`、`cvtColor`、`fill`、`crop` 等操作交给硬件 2D 单元。

#### RGA letterbox 流程

以原图 `640x480`、模型输入 `640x640` 为例：

```text
scale = min(640 / 640, 640 / 480) = 1
new_w = 640
new_h = 480
pad_x = 0
pad_y = 80
```

操作流程：

```text
1. 准备源图 buffer
2. 准备目标 640x640 buffer
3. 用 RGA fill 把目标 buffer 填成 114
4. 用 RGA resize 把原图缩放到 640x480
5. 把 resize 后的图贴到目标 buffer 的 y=80 位置
6. 必要时用 RGA 做 BGR/RGB/YUV 颜色转换
7. 记录 scale/pad_x/pad_y，供后处理坐标还原
```

坐标还原仍然使用：

```text
x = (x_model - pad_x) / scale
y = (y_model - pad_y) / scale
```

#### RGA 与 zero-copy 的关系

RGA 优化的是图像预处理计算：

```text
resize / letterbox / color convert
```

RKNN zero-copy 优化的是输入 buffer 交给 RKNN 时的拷贝：

```text
rknn_inputs_set() 可能带来的内部 copy
```

理想链路是：

```text
相机 dma-buf
-> RGA 处理到目标 dma-buf
-> RKNN 直接使用目标 buffer
```

实际工程建议分阶段做：

```text
阶段 1: OpenCV Mat -> RGA 预处理 -> CPU buffer -> RKNN
阶段 2: RGA 输出 buffer -> RKNN input tensor memory
阶段 3: 相机 dma-buf -> RGA -> RKNN 全链路减少拷贝
```

#### RGA 适合场景

- `preprocess_ms` 明显偏高，例如超过 `3ms~5ms`。
- OpenCV `resize/cvtColor/letterbox` 占用了较多 CPU。
- 输入尺寸固定。
- 需要释放 CPU 给后处理、ROS 同步或控制逻辑。

#### RGA 常见坑

- OpenCV 常见是 BGR，但 RKNN/YOLO 可能需要 RGB。
- 相机输入可能是 YUYV/NV12，不能和 BGR/RGB 混淆。
- RGA 对 stride、宽高、buffer 对齐有要求。
- padding、scale、pad_x、pad_y 必须和后处理完全一致。
- 如果为了使用 RGA 增加多次 CPU buffer 拷贝，可能抵消收益。
- 小图或轻量预处理场景下，RGA 调度开销可能大于收益。
- RGA 只优化预处理，不会让 YOLO 模型本身的 `rknn_run` 更快。

当前实测 `preprocess_ms≈0.97ms`，RGA 不是第一优先级。即使把预处理完全优化为 `0ms`，理论提升也只有约 `0.97ms / 30.20ms ≈ 3.2%`。只有当后续输入格式变化、分辨率升高或 `preprocess_ms` 超过 `3ms~5ms` 时，再进入 RGA 实现。

### 7. 高优先级运行

`main.cc` 使用：

```cpp
rknn_init(&ctx, model_data, model_size, RKNN_FLAG_PRIOR_HIGH, nullptr);
```

可借鉴为：

```cpp
rknn_init(&context_, model_data_.data(), model_size, RKNN_FLAG_PRIOR_HIGH, nullptr);
```

作用：

- 提高当前 RKNN context 的调度优先级。
- 当系统存在多个 NPU 使用者时，可能降低等待和抖动。

注意：

- 如果系统只有一个 YOLO 节点，收益可能有限。
- 需要实测 FPS 和 `rknn_run` 耗时。

### 8. Core Mask

`board_stress_test_6p.py` 使用：

```python
CORES = [1, 2, 4]
```

分别对应 RK3588 三个 NPU core。`main.cc` 使用：

```cpp
rknn_set_core_mask(ctx, RKNN_NPU_CORE_0);
rknn_set_core_mask(ctx, RKNN_NPU_CORE_1);
rknn_set_core_mask(ctx, RKNN_NPU_CORE_2);
```

当前单 context 可先试：

```cpp
rknn_set_core_mask(context_, RKNN_NPU_CORE_0_1_2);
```

验证重点：

- `rknn_run` 耗时是否下降
- FPS 是否提高
- debugfs 三核心占用是否变化
- 温度是否升高

注意：

- 单 context 绑定三核不一定比默认快。
- 有些模型天然不容易被一个 context 均匀分到三核。

### 9. Zero-Copy 输入

`main.cc` 里的 zero-copy 思路是：

```cpp
rknn_tensor_mem *input_mem = rknn_create_mem(ctx, input_attr.size);
memcpy(input_mem->virt_addr, img.data, input_attr.size);
input_attr.pass_through = 1;
rknn_set_io_mem(ctx, input_mem, &input_attr);
```

准确地说，这不是 Linux 多进程共享内存，也不是 `shm_open/mmap/System V shared memory`。它是 RKNN Runtime 提供的 IO tensor memory：CPU 可以通过 `input_mem->virt_addr` 写入图像数据，NPU 直接把同一块 buffer 当作模型输入。

它的目标是绕开每帧 `rknn_inputs_set()` 可能带来的 Runtime 内部拷贝。

适合场景：

- `rknn_run` 前的输入设置耗时明显。
- 输入尺寸固定。
- 每帧都要重复推理。

注意：

- 它不是完全不拷贝。OpenCV 图像到 RKNN 输入 buffer 仍可能需要 `memcpy`。
- 需要管理 `rknn_tensor_mem` 生命周期。
- 需要保证输入格式、布局、尺寸和模型 input attr 完全匹配。
- 这属于后期优化，应在确认 preprocess/input-set 成本明显后再做。

当前实测 `input_set_ms≈0.34ms`，zero-copy 不是第一优先级。它属于后期优化，适合在输入拷贝或 `rknn_inputs_set()` 明显偏高时再做。

### 10. 多 RKNN context / 多 worker

不要直接照搬 `board_stress_test_6p.py` 的 6 进程方式。那是压测模型吞吐，不适合实时无人机视觉。

更适合的结构是：

```text
一个 ROS2 节点进程
2 或 3 个 RKNN context
每个 worker 绑定一个 NPU core
输入队列只保留最新帧
推理结果带 timestamp/frame_id
旧结果直接丢弃
```

多 worker 会乱序完成：

```text
frame 101 -> worker0
frame 102 -> worker1
frame 103 -> worker2

可能 frame 102 先完成，frame 101 后完成。
```

实时控制中应使用“只发布最新结果”的策略：

```text
结果比当前最新已发布结果旧 -> 丢弃
结果更新 -> 发布
```

严格按顺序发布会增加延迟，不适合控制。

这个方向适合在以下条件满足时再做：

```text
1. 单 context 的 rknn_run_ms 已经通过频率/core mask/high priority 测过。
2. 输入 topic Hz 足够高，单 context 处理不过来。
3. debugfs 显示 NPU 三核没有被单 context 充分利用。
4. 系统允许“提高吞吐但不保证每帧都按顺序返回”。
```

不要照搬视频 demo 的 FIFO future 队列。无人机实时视觉应使用最新帧策略：

```text
推理输入队列长度 1
新帧到来时覆盖旧待处理帧
输出结果比最新已发布结果旧则丢弃
发布端只使用最新 timestamp/frame_id
```

### 11. 频率锁定

`fix_freq_rk3588.sh` 的作用是把 NPU、CPU、GPU、DDR 锁到高频，降低性能测试抖动。例如：

```bash
echo userspace > /sys/class/devfreq/fdab0000.npu/governor
echo 1000000000 > /sys/class/devfreq/fdab0000.npu/userspace/set_freq
cat /sys/class/devfreq/fdab0000.npu/cur_freq
```

测试用途：

- 对比 FP16/INT8 时保证频率一致。
- 避免 governor 自动调频导致结果抖动。

注意：

- 路径和可用频率要以当前板子为准。
- 长时间锁高频会升温，可能触发降频或影响稳定性。

### 12. RKNN Toolkit2 单模型剖析

`pc_remote_test_v1.py` 使用：

```python
rknn.init_runtime(target="rk3588", device_id=DEVICE_ID, perf_debug=True, eval_mem=True)
outputs = rknn.inference(inputs=[img])
rknn.eval_perf(is_print=True)
rknn.eval_memory()
```

适合用来：

- 看每层耗时。
- 看模型内存占用。
- 判断某些层是否特别慢。
- 单独比较 FP16/INT8 模型。

限制：

- 它是单图/单模型剖析，不是 ROS2 节点端到端测试。
- 不包含相机、ROS 同步、预处理、后处理和显示开销。

## 推荐实施路线

### 阶段 0：当前基线结论

当前已测得：

```text
preprocess_ms:      0.97
input_set_ms:       0.34
rknn_run_ms:       28.52
output_get_ms:      0.16
postprocess_ms:     0.20
detector_total_ms: 30.20
```

当前阶段判断：

```text
主要瓶颈 = rknn_run
理论 detector FPS 上限 ≈ 33.1
RGA / zero-copy / NMS 暂不优先
```

### 阶段 1：补齐测量闭环

```text
1. 保留 preprocess / input_set / rknn_run / output_get / postprocess 分段计时
2. 用 ros2 topic hz 测 color/depth 输入频率
3. 用 monitor_yolo_npu_perf.sh 看 NPU load/freq/三核/温度
4. 关闭 debug_view 做纯性能测试
5. 记录 total_callback_ms，区分 detector 耗时和 ROS/debug view 耗时
```

### 阶段 2：当前优先优化

```text
1. 检查 NPU cur_freq/governor/load，必要时测试锁频
2. 测试 RKNN_NPU_CORE_0_1_2
3. 测试 RKNN_FLAG_PRIOR_HIGH
4. 同场景对比 FP16 / 纯 INT8 / hybrid bbox FP16 的 rknn_run_ms
5. 如果 hybrid bbox FP16 明显变慢，评估是否改回纯 INT8 或减少保 FP16 的层
6. 如果想明显超过当前上限，测试 640 -> 416/320 或更小模型
```

### 阶段 3：结构优化，按条件进入

```text
进入条件：
  单 context 已测完 core mask/high priority
  输入 topic Hz 足够高
  三核心没有被单 context 充分利用
  需要提高吞吐而不是严格逐帧低延迟

实现要求：
  2 或 3 个 RKNN context
  每个 worker 绑定一个 NPU core
  输入队列只保留最新帧
  输出按 timestamp/frame_id 丢弃旧结果
  严禁无限队列积压
```

### 阶段 4：后期优化，按瓶颈触发

```text
preprocess_ms 高:
  RGA resize/cvtColor/letterbox
  RGA 输出 buffer 与 RKNN input memory 打通

input_set_ms 高:
  zero-copy 输入

output_get/postprocess 高:
  输出保持量化格式，减少 want_float 反量化开销
  优化 NMS 和候选框遍历

任务侧允许:
  ROI / 隔帧推理

长期测试:
  频率锁定和散热优化
```

## 常见坑

- `NPU load=100` 不等于三核心都满载。
- 没有检测框不等于模型没跑。
- INT8 也显示 `NPU load=100` 不一定异常。
- 输入 topic 只有十几 Hz 时，YOLO 输出 FPS 会被输入流限制。
- debug view、画框、窗口显示会影响性能测试结果。
- 多进程压测脚本的 FPS 不能代表 ROS2 节点端到端 FPS。
- 多 worker 会导致结果乱序，必须用 timestamp/frame_id 过滤旧结果。
- 队列不能无限积压，否则 FPS 看似高，控制拿到的却是旧画面。
- RGA 只优化预处理，不会让 `rknn_run` 本身变快。
- RGA 的 RGB/BGR/YUV 格式、stride、padding 很容易出错。
- zero-copy 不是第一步，它需要确认输入拷贝确实是瓶颈。
- 锁频只能稳定测试条件，不能替代模型和代码优化。

## 调试命令

### 查看 NPU 总体负载

```bash
cat /sys/class/devfreq/fdab0000.npu/load
cat /sys/class/devfreq/fdab0000.npu/cur_freq
cat /sys/class/devfreq/fdab0000.npu/governor
```

如果输出类似：

```text
100@1000000000Hz
```

表示 NPU devfreq 总体负载接近 `100%`，频率或采样频率约为 `1 GHz`。

### 查看 NPU 三核心负载

```bash
sudo mount -t debugfs debugfs /sys/kernel/debug
sudo cat /sys/kernel/debug/rknpu/load
```

如果系统不暴露该文件，就只能看到总体 NPU load，无法确认三核心分别占用。

### 查找 NPU 占用进程

```bash
sudo fuser -v /dev/rknpu
```

如果设备名不同，先查看：

```bash
ls /dev | grep -Ei 'rknpu|npu'
```

### 对比 FP16 和 INT8

```bash
./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --duration 30 --interval 0.5 --no-clear --show-raw
```

比较：

```text
平均 FPS
rknn_run 平均耗时
NPU load
NPU freq
三核心占用
YOLO 进程 CPU
温度
```

## 复盘结论

当前最值得优先做的是：

```text
保留分段计时
确认输入 topic 频率和 total_callback_ms
检查 NPU 频率/governor/load
对比 FP16 / INT8 / hybrid 的 rknn_run_ms
测试 RKNN_NPU_CORE_0_1_2
测试 RKNN_FLAG_PRIOR_HIGH
需要明显提速时降低输入尺寸或换更小模型
```

RGA、多 worker 和 zero-copy 都是有效方向，但进入时机不同：RGA 只在 `preprocess_ms` 明显偏高时优先做；multi-worker 要在单 context 无法充分利用三核心且输入帧率足够高时再做；zero-copy 要在输入拷贝或 input-set 成本明确偏高时再做。

当前实测下，`rknn_run_ms` 占比约 `94.4%`，所以任何不直接降低 `rknn_run_ms` 的优化都只能带来很小收益。
