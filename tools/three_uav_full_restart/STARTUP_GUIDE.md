# 三机完整启动说明

## 整体启动
```bash
~/consensus_ws/tools/three_uav_full_restart/restart_three_uav_stack.sh
```

该脚本依次启动：
1. QGroundControl
2. Micro XRCE-DDS Agent
3. UAV1（创建 Student Center world 与 Gazebo GUI）
4. UAV2（接入既有 world）
5. UAV3（接入既有 world）

实例关系：

| UAV | PX4实例 | DDS命名空间 | 初始位置 |
|---|---:|---|---|
| UAV1 | 1 | `/px4_1` | `(-6,0,0)` |
| UAV2 | 2 | `/px4_2` | `(6,0,0)` |
| UAV3 | 3 | `/px4_3` | `(0,6,0)` |

## 查看运行终端
```bash
tmux attach -t map_consensus_3uav
```

## QGC显示三机均已连接后启动起飞
```bash
~/consensus_ws/tools/three_uav_full_restart/start_three_uav_takeoff.sh
```

## 全部停止
```bash
~/consensus_ws/tools/three_uav_full_restart/stop_three_uav_stack.sh
```
