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
