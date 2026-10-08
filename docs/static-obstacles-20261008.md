# 2026-10-08 按原始 PGM 恢复静态障碍物（坐标修正版）

当前场景：scenarios/short_routes_20261007。

## 原问题

首次直接使用原 SDF 的 Untitled 模型坐标，虽然复制了 13 个方块，
但原 PGM 的地图坐标与 SDF 不一致，造成静态方块位置错误，两个甚至位于外墙外。
该版已纠正，旧的 (0.4,-7.06) 轨道终点也不再使用。

## 当前实现

- 使用 tools/align_static_obstacles.py 将原 PGM 墙线与本地 SDF 墙体配准，
  局部 SDF 到原 PGM 的平移为 (1.63773,0.03356)m，旋转为 0.006156rad。
  保留85%墙线样本的距离均方根为0.01196m（用于配准，不代表模型全部几何误差）。
- 从原 PGM 的 13 个独立方块内部提取位置和朝向，转换到本地坐标。
  检测结果保存在 static_obstacles_from_map.json，原图分辨率为0.05m。
  尺寸保留原 SDF 的 1×1×1m，不能宣称扫描图和实体边界逐像素完全一致。
- 对照图 static_alignment_comparison.png 左侧为原 PGM，右侧为修正后墙体/方块，
  均在原 PGM 参考坐标中展示。layout.png 为本地实际世界坐标布局。
- 方块模型 imported_static_obstacles 固定 static=1，所有13个中心都在外墙内。
  13个可视材质均设为橙色（RGBA 1,0.45,0.05,1）。
  同步渲染碰撞几何到本地 mapn3.pgm；墙、货物、区域保持。
- 用户选择保留静态方块位置、缩短相交移动轨道；正确配准后两条轨道都相交，
  上方从(-3.65,1.0)到(-1.4,1.0)，下方从(-5.6,-7.06)到(-1.45,-7.06)。
  两条移动轨道对固定方块最小表面间距分别0.62546m、0.54661m，速度仍为0.35m/s。
- SDF格式检查通过；每个货物至少一个抓取停车点和三处区域停车点通过0.48m
  静态净空连通性检查；区域未与静态物体相交，货物间距保持。
- U盘文件与本地导入原件未修改；生成器检查配准所用原PGM的SHA256防止误用旧结果。
- 尚未对本次新增、配准后的静态场景执行五件搬运回归，旧110秒结果不适用于本布局。

重新生成：

```bash
PYTHONNOUSERSITE=1 python3 tools/align_static_obstacles.py
PYTHONNOUSERSITE=1 python3 tools/design_short_map.py
PYTHONNOUSERSITE=1 python3 tools/plot_map_layout.py scenarios/short_routes_20261007
```

运行：

```bash
PYTHONNOUSERSITE=1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 bash test_map.sh short_routes_20261007 benchmark
```
