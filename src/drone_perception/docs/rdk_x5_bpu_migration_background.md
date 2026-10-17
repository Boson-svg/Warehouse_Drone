# RDK X5 BPU 模型部署背景文档（对接 AI 用）

> 本文档面向后续接手开发的 AI/C++ 开发者，总结全链路背景知识，使其无需重走排查过程。
> 目标：将现有 `drone_perception`（原 RK3588 NPU 架构）迁移至 **RDK X5 BPU**。

---

## 一、全链路路径总览

### 1.1 已完成的路径

```
训练权重 (.pt)                               ─── Ultralytics YOLO11n 训练，nc=2（二维码 + 条形码）
    │
    ▼  ─── export_monkey_patch.py           ─── yolo conda 环境
ONNX 模型 (.onnx)                            ─── 6 输出，NHWC，猴子补丁截断 DFL/Sigmoid
    │
    ▼  ─── mapper.py + hb_mapper            ─── rdk_env conda 环境，校准图片 46 张
BPU 量化模型 (.bin)                          ─── rdk_best_bayese_640x640_nv12.bin
    │
    ▼  ─── hb_perf（PC 端理论性能估算）
性能报告                                     ─── 207.8 FPS，4.8 ms 延迟
    │
    ▼  ─── ONNX Runtime 验证（float32 推理）
验证通过                                     ─── cls sigmoid max = 0.9646（stride 32 尺度）
```

### 1.2 待完成的路径（需在 RDK X5 板子上执行）

```
.bin 模型
    │
    ▼  ─── C++ 推理代码（hb_dnn API，适配 drone_perception）
板端实时推理
    │
    ▼  ─── hrt_model_exec perf（板端实测 FPS）
真实性能数据
```

---

## 二、所有涉及的文件路径及说明

### 2.1 训练与导出阶段

| 路径 | 说明 |
|---|---|
| `/media/gjl/新加卷/OpenCVVVVVVVVVVVVVVVVVVV/ultralytics-8.3.163/runs/detect/qr_predict/weights/best.pt` | 训练好的 YOLO11n 权重（nc=2：二维码 + 条形码，640×640） |
| `/media/gjl/新加卷/OpenCVVVVVVVVVVVVVVVVVVV/ultralytics-8.3.163/runs/detect/qr_predict/weights/rdk_best.onnx` | 导出后的 ONNX 模型（猴子补丁方式，6 输出，NHWC） |
| `/home/gjl/rdk_model_zoo/samples/vision/ultralytics_yolo/conversion/export_monkey_patch.py` | ONNX 导出脚本（含猴子补丁，截断 DFL/Sigmoid，只导出卷积输出） |
| `/media/gjl/新加卷/OpenCVVVVVVVVVVVVVVVVVVV/ultralytics-8.3.163/runs/detect/qr_predict/weights/rdk_best_bayese_640x640_nv12.bin` | 编译后的 BPU 量化模型（最终产物） |
| `/media/gjl/新加卷/OpenCVVVVVVVVVVVVVVVVVVV/ultralytics-8.3.163/runs/detect/qr_predict/weights/hb_mapper_makertbin.log` | hb_mapper 编译日志 |

### 2.2 校准数据

| 路径 | 说明 |
|---|---|
| `/home/gjl/qr_image/qr_image 20-50/` | 46 张校准图片（二维码 + 条形码混编，全部用于量化校准） |

### 2.3 官方参考代码

| 路径 | 说明 |
|---|---|
| `/home/gjl/rdk_model_zoo/samples/vision/ultralytics_yolo/conversion/mapper.py` | 官方模型编译脚本（自动生成校准数据 + 配置文件 + 调用 hb_mapper） |
| `/home/gjl/rdk_model_zoo/samples/vision/ultralytics_yolo/conversion/config.yaml` | 量化配置模板 |
| `/home/gjl/rdk_model_zoo/samples/vision/ultralytics_yolo/runtime/cpp/detect/main.cc` | **官方 C++ 推理参考代码（6 输出 + DFL 后处理）** |
| `/home/gjl/rdk_model_zoo/samples/vision/ultralytics_yolo/evaluator/README_cn.md` | 精度与性能评测说明 |
| `/home/gjl/rdk_model_zoo/docs/source_reference/bpu_sample_docs_html/` | 解压后的 BPU 文档站点 |

### 2.4 第三方参考文章

| 路径 | 说明 |
|---|---|
| `/home/gjl/Desktop/万字长文，学弟一看就会的RDKX5模型转换及部署，你确定不学？ - 开发与问题 _ Model Zoo - 地瓜机器人论坛.html` | SkyXZ 撰写的社区教程（含完整 C++ BPU_Detect 类封装、内存管理、从内存加载模型等） |

### 2.5 目标工程

| 路径 | 说明 |
|---|---|
| `/home/gjl/drone_ws/src/drone_perception/` | ROS2 目标工程（原 RK3588 NPU 架构，现需迁移至 RDK X5 BPU） |
| `/home/gjl/drone_ws/src/drone_perception/include/drone_perception/detection.hpp` | 检测结果数据结构（struct Detection） |
| `/home/gjl/drone_ws/src/drone_perception/include/drone_perception/rknn_yolo_detector.hpp` | RKNN 检测器类接口 |
| `/home/gjl/drone_ws/src/drone_perception/include/drone_perception/yolo_postprocessor.hpp` | YOLO 后处理器接口 |
| `/home/gjl/drone_ws/src/drone_perception/src/rknn_yolo_detector.cpp` | RKNN 检测器实现 |
| `/home/gjl/drone_ws/src/drone_perception/src/yolo_postprocessor.cpp` | YOLO 后处理实现（8400 候选框，解析单输出/分离输出） |
| `/home/gjl/drone_ws/src/drone_perception/src/main.cpp` | ROS2 节点入口 |
| `/home/gjl/drone_ws/src/drone_perception/src/qr_vision_node.cpp` | 主 ROS2 节点（含图像订阅、推理调用、深度关联、结果发布） |
| `/home/gjl/drone_ws/src/drone_perception/CMakeLists.txt` | 构建文件（条件编译 RKNN） |
| `/home/gjl/drone_ws/src/drone_perception/docs/yolo_npu_fps_optimization.md` | 现有 FPS 优化分析文档 |

---

## 三、环境说明

### 3.1 当前可用环境

| 环境名称 | 类型 | 用途 | 关键依赖 |
|---|---|---|---|
| `yolo` | Conda | ONNX 导出（pt → onnx） | ultralytics 8.3.163, torch 2.5.0 |
| `rdk_env` | Conda | BPU 量化编译（onnx → bin） | rdkx5-yolo-mapper（hb_mapper 1.24.3）, onnxruntime, opencv-python |
| 板端（RDK X5） | 出厂系统 | C++ 推理运行 | 出厂自带：hb_dnn.h, libhbrt.so, OpenCV |

### 3.2 RDK X5 板端 SDK（出厂自带，无需安装）

```
头文件:
  /usr/include/dnn/hb_dnn.h
  /usr/include/dnn/hb_dnn_ext.h
  /usr/include/dnn/hb_sys.h
  /usr/include/dnn/plugin/hb_dnn_layer.h
  /usr/include/dnn/plugin/hb_dnn_plugin.h

动态库:
  /usr/lib/libhbrt*.so
  /usr/lib/libdnn.so
```

### 3.3 编译方式对比

**RDK X5 上直接编译（推荐）：**
```bash
# 在板子上执行
g++ main.cc -o detect -lhbrt -lopencv_core -lopencv_imgproc -lopencv_highgui
```

**PC 上交叉编译（仅当需要在 PC 上编译时）：**
```bash
# 需要 Docker 镜像（含 ARM64 交叉工具链）
docker pull openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8
```

---

## 四、模型输出格式关键说明（适配核心）

### 4.1 当前 RK3588 模型输出格式

| 参数 | 值 |
|---|---|
| 输出形式 | 单输出 `[1, 6, 8400]` 或 分离输出 `[1,4,8400]` + `[1,2,8400]` |
| 数据排布 | NCHW |
| 框编码 | center format（x_center, y_center, width, height） |
| 后处理 | 遍历 8400 候选框，Sigmoid + NMS |

### 4.2 RDK X5 BPU 模型输出格式（当前编译产物）

| 参数 | 值 |
|---|---|
| 输出形式 | **6 个独立输出**：cls × 3 尺度 + bbox × 3 尺度 |
| 各输出 shape | `(1, 80, 80, 2)`、`(1, 80, 80, 64)`、`(1, 40, 40, 2)`、`(1, 40, 40, 64)`、`(1, 20, 20, 2)`、`(1, 20, 20, 64)` |
| 数据排布 | **NHWC**（注意：不是 NCHW！） |
| 模型类别数 | **nc=2**（类 0：二维码，类 1：条形码） |
| 框编码 | **DFL（Distribution Focal Loss）**：每个方向 16 个分布值，需 Softmax + 加权求和解码 |
| cls 通道数 | 2（对应 2 个类别） |
| bbox 通道数 | 64（= 4 个方向 × 16 分布值，REG=16） |
| 后处理 | DFL 解码 + Sigmoid + NMS（需完全重写，与 RKNN 后处理不兼容） |

### 4.3 关键适配项

> ⚠️ **后处理代码必须完全重写**，因为：
> 1. 候选框数量不同：8400（RKNN）→ 6400（BPU，80×80 + 40×40 + 20×20）
> 2. 框解码方式不同：center format（RKNN）→ DFL 分布解码（BPU）
> 3. 输出排布不同：NCHW（RKNN）→ NHWC（BPU）
> 4. 输出数量不同：1~2 个 tensor → 6 个 tensor
> 5. 数据读取索引计算方式完全不同

### 4.4 输出顺序映射

BPU 输出索引顺序（由 `export_monkey_patch.py` 的 permute 决定，按 NHWC 排列）：

```
output[0]: cls_8     (1, 80, 80, 2)   小尺度（stride 8） cls
output[1]: bbox_8    (1, 80, 80, 64)  小尺度（stride 8） bbox
output[2]: cls_16    (1, 40, 40, 2)   中尺度（stride 16）cls
output[3]: bbox_16   (1, 40, 40, 64)  中尺度（stride 16）bbox
output[4]: cls_32    (1, 20, 20, 2)   大尺度（stride 32）cls
output[5]: bbox_32   (1, 20, 20, 64)  大尺度（stride 32）bbox
```

### 4.5 与 main.cc 后处理对照

官方 `main.cc` 中 DFL 解码的核心逻辑（第 574~657 行）：

```cpp
// 遍历 3 个尺度（stride 8, 16, 32）
for (int scale_idx = 0; scale_idx < 3; scale_idx++) {
    int cls_idx = order[scale_idx * 2];
    int bbox_idx = order[scale_idx * 2 + 1];

    float* cls_raw = (float*)output[cls_idx].sysMem[0].virAddr;
    float* bbox_raw = (float*)output[bbox_idx].sysMem[0].virAddr;

    // 遍历每个网格
    for (int gh = 0; gh < h; gh++)
        for (int gw = 0; gw < w; gw++) {
            // 找 cls 最大值（遍历 2 个类）
            // DFL 解码（4 组 × 16 个分布值 → Softmax → 加权求和）
            // Sigmoid → 置信度
            // NMS
        }
}
```

**适配要点：**
- 输出读取顺序：先 cls 后 bbox（与 RKNN 的 bbox 在前不同）
- 数据索引方式：NHWC（`data[row * W * C + col * C + channel]`），不是 NCHW
- bbox 解码：没有 center/width/height，改用 4 方向距离 + DFL 分布
- cls 最大值搜索范围：2 个类（不是 80，也不是 1）

---

## 五、量化配置关键参数备忘

当前编译 `rdk_best_bayese_640x640_nv12.bin` 使用的参数：

```yaml
model_parameters:
  march: "bayes-e"                           # RDK X5 BPU 架构
  input_type_rt: 'nv12'                      # 板端输入格式
  input_type_train: 'rgb'                    # 训练时格式
  input_layout_train: 'NCHW'
  norm_type: 'data_scale'
  scale_value: 0.003921568627451             # = 1/255（BPU 自动归一化）
  calibration_type: 'default'                # = KL 散度
  compile_mode: 'latency'                    # 延迟优先
  optimize_level: 'O3'                       # 最高优化
```

> ⚠️ **校准数据范围必须为 0~255（原始像素值），不能做 /255 归一化。**
>
> 因为 `scale_value: 1/255` 会在 BPU 端自动做归一化，如果在校准脚本中再次除以 255，会导致双重归一化（有效输入变成 pixel/65025），模型输出完全崩塌（cls 全部为强负数）。

---

## 六、C++ 优化建议（结合官方 main.cc 与社区文章）

### 6.1 多线程流水线（最大 FPS 提升）

官方 `main.cc` 是单线程同步推理。社区文章也未实现多线程。

**建议方案：** 生产者-消费者 3 线程流水线

```
线程 1（Camera + Preprocess）:
  读帧 → LetterBox → BGR→NV12 → 写入共享内存 → 通知线程 2

线程 2（Inference）:
  等待通知 → hbDNNInfer（BPU 推理）→ 通知线程 3

线程 3（Postprocess + Publish）:
  等待通知 → DFL 解码 → NMS → 坐标变换 → ROS2 发布
```

**预期效果：** 小模型（YOLO11n）单线程约 200 FPS，多线程流水线可达 300+ FPS。

**核心 API 用法：** `hbDNNInfer` 是异步的，返回 `task_handle`。可在 BPU 计算时让 CPU 做预处理或后处理：

```cpp
hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param);
// CPU 可以在这里做一些非阻塞工作
hbDNNWaitTaskDone(task_handle, 0);  // 等待 BPU 完成
```

### 6.2 从内存加载模型

官方 `main.cc` 使用 `hbDNNInitializeFromFiles`（从文件加载）。
社区文章提供 `hbDNNInitializeFromDDR`（从内存加载）替代方案：

```cpp
// 从文件加载（简单，调试用）
hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1);

// 从内存加载（更快，生产环境用）
FILE* fp = fopen(model_path, "rb");
fseek(fp, 0, SEEK_END);
size_t model_size = ftell(fp);
fseek(fp, 0, SEEK_SET);
void* model_data = malloc(model_size);
fread(model_data, 1, model_size, fp);
fclose(fp);

const void* data_array[] = {model_data};
int32_t len_array[] = {(int32_t)model_size};
hbDNNInitializeFromDDR(&packed_dnn_handle, data_array, len_array, 1);
```

**建议：** 初始化时从文件读到内存，然后用 `FromDDR` 初始化。减少约 10~20ms 模型加载时间。

### 6.3 内存缓存管理（正确性要求）

每次推理必须做以下操作，否则可能读到脏数据：

```cpp
// 推理前：清理 CPU 缓存，确保 BPU 看到最新输入数据
hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

// 推理后：刷新缓存，确保 CPU 读到 BPU 写好的输出
for (int i = 0; i < output_count; i++)
    hbSysFlushMem(&output[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
```

### 6.4 重复使用内存池

社区文章每次推理都重新 `hbSysAllocCachedMem` + `hbSysFreeMem`，增加不必要的开销。

**建议：** 在初始化时一次性分配好输入/输出内存，推理循环中重复使用，仅在模型或分辨率变化时重新分配。

```cpp
// 初始化时分配一次
hbSysAllocCachedMem(&input.sysMem[0], input_size);
for (int i = 0; i < output_count; i++)
    hbSysAllocCachedMem(&output[i].sysMem[0], output_size);

// 推理循环中只做 memcpy + Flush + 推理 + Invalidate
while (running) {
    memcpy(input.sysMem[0].virAddr, nv12_data, input_size);
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    hbDNNInfer(...);
    hbDNNWaitTaskDone(...);
    hbSysFlushMem(&output[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    // 后处理...
}
```

### 6.5 NV12 输入硬件化

官方代码中的 `bgr2nv12()` 函数用 CPU（OpenCV）做 BGR→NV12 转换（耗时约 0.5~2ms）。

RDK X5 的 ISP/VPU 硬件可以直接输出 NV12 格式：
- **摄像头输入**：从 ISP 直接获取 NV12 数据，跳过 BGR→NV12 转换
- **图片/视频输入**：使用 VPS（Video Processing Subsystem）硬件加速

### 6.6 后处理优化

| 优化项 | 建议 | 说明 |
|---|---|---|
| `CLASSES_NUM` | **保持 2**（原值不变） | 模型有二维码和条形码 2 个类，不能合并 |
| `SCORE_THRESHOLD` | 可酌情从 0.25 提高到 0.4~0.5 | 减少 NMS 候选框数量 |
| NMS 算法 | 单类别检测可简化 | 此处 2 类，标准 NMS |
| 后处理向量化 | NEON 指令 / 批量矩阵运算 | 替代逐元素 for 循环 |

> **关于类别数的说明：** 当前模型 nc=2（二维码 + 条形码），后处理时 cls 分支需遍历 2 个类找最大值。相比 COCO 的 80 类（官方 main.cc 默认值）已经少了很多，此部分无需优化。

### 6.7 优先级调度

多模型同时推理时，可通过 `hbDNNInferCtrlParam` 设置优先级：

```cpp
hbDNNInferCtrlParam infer_ctrl_param;
HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
infer_ctrl_param.priority = HB_DNN_PRIORITY_HIGHEST;
```

### 6.8 CPU / BPU 频率锁定

板端执行（不涉及代码修改，但影响 FPS 稳定性）：

```bash
# CPU 性能模式
sudo cpufreq-set -g performance

# 检查 BPU 频率
cat /sys/class/devfreq/.../cur_freq
```

---

## 七、验证清单

### 7.1 ONNX 导出验证

| 检查项 | 期望值 | 说明 |
|---|---|---|
| 输出数量 | 6 | bbox×3 + cls×3 |
| 输出格式 | NHWC（`1, H, W, C`） | 由 export_monkey_patch.py 的 permute 决定 |
| bbox shape | `(1, H, W, 64)` | 64 = 4 × reg_max(16) |
| cls shape | `(1, H, W, 2)` | 2 = 二维码 + 条形码 |
| Softmax 节点 | 仅 1 个 | 来自 attention block，不含 DFL |

### 7.2 量化编译验证

| 指标 | 好 | 差 |
|---|---|---|
| Cosine Similarity | ≥ 0.99 | < 0.95 |
| Chebyshev Distance | < 0.5 | > 2.0 |
| 子图数 | 1~2 | > 5 |

### 7.3 板端推理验证

```bash
# 查看模型信息
hrt_model_exec model_info --model_file rdk_best_bayese_640x640_nv12.bin

# 单线程测速
hrt_model_exec perf --model_file rdk_best_bayese_640x640_nv12.bin --thread_num 1

# 多线程吞吐测速
hrt_model_exec perf --model_file rdk_best_bayese_640x640_nv12.bin --thread_num 2
```

---

## 八、关键结论与后续动作

### 8.1 已确认结论

1. ✅ ONNX 导出正确（float32 验证通过，cls sigmoid max = 0.9646）
2. ✅ 校准数据范围正确（0~255，无双重归一化）
3. ✅ 量化编译成功（rdk_best_bayese_640x640_nv12.bin，207.8 FPS 理论值）
4. ✅ 模型在 stride 32 尺度能正常检测大目标（二维码/条形码）
5. ✅ 官方工具链（Conda + mapper.py）链路完整，无需 Docker
6. ✅ 模型类别 nc=2（二维码 + 条形码），ONNX / .bin / 验证全部一致

### 8.2 后续必须做的适配

| 序号 | 工作 | 优先级 | 说明 | 参考代码 |
|---|---|---|---|---|
| 1 | **后处理代码重写** | P0 | 8400 候选框+center format → 3 尺度网格+DFL 解码，cls 遍历 2 类 | `main.cc:574-657` |
| 2 | **推理 API 替换** | P0 | RKNN API → hb_dnn API（`hbDNNInfer` / `hbDNNWaitTaskDone`） | `main.cc:556-561` |
| 3 | **输入格式适配** | P0 | RGB float32 NCHW → NV12；增加 BGR→NV12 转换 | `main.cc:165-200` |
| 4 | **输出排布适配** | P0 | NCHW 读取 → NHWC 读取，索引公式改变 | `main.cc:473-503` |
| 5 | **CMakeLists 更新** | P1 | 添加 `-ldnn`，移除 RKNN 条件编译 | — |
| 6 | **ROS2 节点接口保持** | P1 | `QrVisionNode` 对上层发布的 Detection 结构不变 | — |

### 8.3 可选优化（按优先级排列）

| 优化 | 预期收益 | 工作量 | 类型 |
|---|---|---|---|
| 多线程推理流水线 | FPS 翻倍 | 中 | 架构 |
| 重复使用内存池 | 减少 malloc jitter | 小 | 代码 |
| NV12 硬件直通 | 省掉格式转换时间 | 中（需硬件支持） | 硬件 |
| CPU/BPU 频率锁定 | FPS 更稳定 | ✅ 极小（板端命令） | 系统 |
| 提高 score_thres | 减少后处理候选框 | ✅ 极小 | 调参 |

---

## 九、附录：关键概念解释

### 猴子补丁（Monkey Patch）

运行时替换对象的方法，不修改源码文件。在 `export_monkey_patch.py` 中用于临时替换 `Detect.forward`，使 ONNX 只导出卷积输出（原始特征），不包含 DFL 解码和 Sigmoid。

### DFL 解码（Distribution Focal Loss Decoding）

YOLO 的 bbox 输出不是直接的坐标值，而是每个方向（左/上/右/下）的 16 个概率分布值。解码过程：

```
原始输出: [d0, d1, ..., d15] 共 16 个值（每个方向）
    → Softmax 变成概率分布
    → 加权求和：Σ(pi × i) → 最终偏移量
```

### NHWC 与 NCHW

| 格式 | 内存排列 | 索引公式 |
|---|---|---|
| NCHW | 先通道，再高宽 | `data[C×H×W + H×W + ...]` |
| NHWC | 先高宽，再通道 | `data[H×W×C + W×C + C]` |

BPU 编译产物使用 NHWC，与 `export_monkey_patch.py` 中 `permute(0, 2, 3, 1)` 的输出一致。

### 校准数据范围

量化校准图片必须保持 0~255 原始像素范围（float32），BPU 通过 `scale_value: 1/255` 在硬件层面自动归一化。如果在脚本中手动 `/255`，会导致双重归一化，模型输出完全崩塌。
