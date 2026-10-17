builtin_interfaces/Time stamp  # Linux 视觉节点发布该帧目标列表的时间戳。
uint32 frame_seq  # K230/视觉端发布的目标列表帧序号，用于和控制端回传命令对应。

int32 scan_point_index  # 当前目标列表对应的路线点编号。
float64 scan_point_x  # 当前路线点世界 x 坐标。
float64 scan_point_y  # 当前路线点世界 y 坐标。

uint32 target_count  # 当前帧稳定目标数量，应与 targets 数组长度一致。
K230AnimalTarget[] targets  # 当前帧稳定动物目标列表。
