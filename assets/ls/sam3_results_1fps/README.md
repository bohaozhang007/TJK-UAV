# 1 fps SAM3 检测结果

- 抽帧：`../frames_1fps/`，183 帧，时间戳 0～182 秒。
- 每帧：`frame_XXXXXX/` 中有 5 张 `tN_input.jpg`、5 张 `tN_top3.jpg`、1 张 `overview.jpg`，以及检测坐标和置信度 `detections.json`。
- 输入图：上方视频帧，下方参考图；参考图缩放至视频宽度，提示框同步变换。保存时离线画框，模型接收未画框的图和独立 bbox 提示。
- Top3：每组参考独立 NMS（IoU 0.50），置信度降序取最多 3 个框，候选阈值 0.30。
- 框颜色：置信度严格大于 0.50 为红色，其余为绿色。编号、置信度同时显示；不足 3 个时按实际数量保存。
- 汇总图：2×3 排列，5 个子图左上角标注 t1～t5。
- 最后筛选：任一子图的 Top3 中有置信度严格大于 0.50 的框，就将 overview 原样复制至 `../overviews_1fps_gt05/frame_XXXXXX_overview.jpg`。该目录的 `selection.json` 记录命中框及时间戳。
- `progress.json` 记录进度，`complete` 表示检测及筛选结束。

运行或中断后恢复（不要与运行中的实例并行启动）：

```powershell
& 'C:/Users/colab999/anaconda3/envs/sam3/python.exe' -u 'C:/Users/colab999/Desktop/project/TJK-UAV/scripts/detect_ls_video.py' --fps 1
```

使用本地 SAM3 权重和 BF16 推理，关闭无需输出的分割头。原 30 fps 抽帧及部分结果仍保留在原目录，本轮结果独立存放。
