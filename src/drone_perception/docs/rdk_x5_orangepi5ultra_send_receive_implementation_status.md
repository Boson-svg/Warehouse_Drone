# RDK X5 → OrangePi 5 Ultra 收发与硬件解码 实施状态

> 依据：`docs/rdk_x5_orangepi5ultra_send_receive_implementation_plan.md`
> 测试环境：RDK X5(`ubuntu`@192.168.3.114) / OrangePi 5 Ultra(`orangepi5ultra`@192.168.3.168)
> 网络：Xiaomi 12 手机热点 2.4GHz（两端均 Wi-Fi，OrangePi 信号 ~-78dBm）
> 更新：2026-08-06

## 1. 总体结论

发送→RTSP→MPP 硬件解码→appsink 拉帧的代码路径**已全部打通并验证**。
当前测试网络（手机热点 2.4GHz，iperf3 实测 RDK→OP 仅 ~2Mbit/s、UDP 3M 丢包 6.5%）无法承载计划默认的 **3000kbit/s**；
**1500kbit/s 下稳定 ~29.9fps，作为本网络验收码率**。3000kbit/s 记为“需更好网络（5GHz/有线）复测”。

## 2. 测试结果矩阵

| 项 | 结果 | 证据 |
|---|---|---|
| T0 版本能力 | ✅ | RDK: hb_media_codec/libmultimedia；OP: mppvideodec 1.14、GStreamer 1.20 |
| T1 相机 profile | ✅ | YUYV 640x480@30, bytesperline 1280, size 614400 |
| T2 独立硬编码 | ✅ | 30s 编码 878 帧，ffprobe: h264(Main) 640x480 30fps，无 hb_mm_mc 错误 |
| T3 MediaMTX/RTSP | ✅ | :8554 监听，发布 640x480@30 CBR 3000kbit/s，ffprobe: h264/Main/640x480/30 |
| T4 合成码流 MPP | ✅ | 合成 150/150、真实 D435i 878/878 帧解码到 NV12（multifilesink 计数验证） |
| T5 UDP RTSP MPP | ✅(1500kbps) | 60s 稳定 29.9fps；3000kbps 仅 ~10fps（网络受限） |
| T5 TCP RTSP MPP | 🟡 | 能解码但 ~15-20fps（WiFi 重传反压）；UDP 可用时按计划以 UDP 为主 |
| T6 appsink 真实拉帧 | ✅ | caps=NV12 640x480@30 memory=system；dump 帧 460800B，Y 标准差 46+（真实内容） |
| T7 重连 3 轮 | ✅ | 三轮停 RDK 10s→重启，探针保持存活并重新获得 IDR（seq 446→1055→1548） |
| T8 长时间基线 | ✅(3min) | 1500kbps UDP；发送端 ~30% CPU、温度 63-65°C；接收端探针 ~2% CPU、MPP 硬解、温度 43-44°C；0 drop、0 reconnect，FPS 前 ~60s 波动后稳定 ~28-30 |
| T9 正式 service 集成 | ✅ | D435I_start.service→start_d435.sh→MediaMTX+qr_vision_node；检测与图传消费同一 D435ColorFrame；仅 qr_vision_node 持相机；停服全部退出/8554/相机释放；重启 RTSP 自动恢复 30fps |

## 3. 网络发现（重要）

- 两端同连 Xiaomi 12 手机热点（2.4GHz，信道 11）；OrangePi 信号 -78dBm，链路 19.5Mbit/s。
- iperf3 RDK→OP：TCP ~1.9Mbit/s；UDP 1M/2M 0 丢包，**3M 丢包 6.5%**。
- 3000kbit/s 流超出链路能力：MediaMTX 日志持续 `reader is too slow, discarding N frames`。
- 1500kbit/s 在 UDP 下稳定 29.9fps。TCP 在此 WiFi 反而更差（重传反压）。
- 结论：本网络验收用 **1500kbit/s**；3000kbit/s 需 5GHz 或有线链路复测。

## 4. 待办

- [x] **T8**：3 分钟基线（用户确认缩短），日志存 `test_logs/t8_baseline/`（gitignore）
- [x] **T9**：D435I_start.service 集成完成。qr_vision_node 内图传 worker 消费同一 D435ColorFrame，
  ffmpeg 用 fork+exec 且子进程关闭继承 fd（相机不被 ffmpeg 持有）；start_d435.sh 管理 MediaMTX 生命周期
- [ ] 3000kbit/s 在更好网络下复测
- [ ] feature 分支合并到 main（测试后板端已回 main 前先保持 feature）
- [ ] Qt 地面站阶段（另写专项方案，本阶段不涉及）

## 5. 复现命令

发送端（RDK，需先停 service）：
```bash
sudo systemctl stop D435I_start.service
~/warehouse_ws/src/drone_perception/scripts/rdk_rtsp_link_sender_test.sh 1500 120 /tmp/rtsp_link_test
sudo systemctl start D435I_start.service   # 测试后恢复
```

接收端（OrangePi，独立 g++ 构建）：
```bash
g++ -std=c++17 $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0) \
  src/orangepi_rtsp_mpp_probe.cpp -o /tmp/orangepi_rtsp_mpp_probe
~/warehouse_ws/src/drone_perception/scripts/orangepi_rtsp_mpp_probe_run.sh udp 60
```

## 6. 已知的非致命怪癖

- gstreamer-rockchip 1.14 的 `mppvideodec` 在 PLAYING 阶段打印
  `gst_video_decoder_negotiate_default: assertion 'GST_VIDEO_INFO_WIDTH != 0' failed`，
  实测**不影响解码**（878/878 帧全部输出 NV12）。属插件旧版无害警告。
- RDK 发送器在每次启动会丢弃少量“short camera frames”（T2 为 4/878），非错误。
