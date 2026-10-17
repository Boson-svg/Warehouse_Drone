builtin_interfaces/Time stamp  # 视觉端生成该结果消息的时间戳
uint32 frame_seq  # 对应的目标列表帧序号，用于和 capture_ready 对应
int32 scan_point_index  # 目标所属扫描点编号
string label  # 动物类别
uint32 label_instance_id  # 该帧内同类目标编号
bool record_success  # 本次拍照/保存是否成功
string result_state  # 结果状态：captured / failed / skipped
string image_name  # 成功或失败时对应的图像文件名，可为空
