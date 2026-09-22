# 让转台过渡点（moveType=0）的大关节跳变进入 RRT 规划 —— 提示词

> 说明：本提示词最初把修复位置定在 `applyMeasLinePathPredict`，但执行其
> 第一步「先确认」时发现根因是**跨转台角度分组的接缝**，`applyMeasLinePathPredict`
> 逐组调用、永远看不到接缝，故修复位置已更正。下面是更新后的完整提示词。

---

```
项目：D:\test\QtWidgetsApplication3（C++/Qt 机器人测量路径规划）。

【目标】
让「转台过渡点等 moveType=0 的点」周围的大关节跳变也能进入 RRT 规划。现在这些
点直接以裸 MOVJ 输出，机器人被命令做一次性巨大关节重构（已离线实测：这种跳变
用 RRT 是能规划出可行路径的），中间没有任何碰撞安全的过渡点。

【已确认的根因（来自实际数据验证，务必先读懂）】
JBI 运动指令路径的生成流程：
- main.cpp execute()：allSurfacePath（面点，partRotateAngle 全为 0）+
  combinedEdgePath（切边点 + 转台过渡点）按 partRotateAngle 分组进 std::map aaa。
- 对 aaa 逐组调 BallScan::applyMeasLinePathPredict(it.second, cfg)（main.cpp 约 580 行）。
  逐组处理是有意为之：每个转台角度的碰撞环境不同，必须分别在各自环境下规划。
- applyMeasLinePathPredict（BallScan.cpp:1415）：对【组内】每对相邻点调
  ViewpointPlanner::measLinePathPredict 拼成 predicted，再调
  spliceRRTIntoDensePath(predicted, rrtCtx)。
- RRT（planRRTBetween，path_rrt_splice.cpp:120）只处理「连续 moveType==-1 的 run，
  长度 >=2」。-1 由 markAndCleanMinusTwo 把 -2 占位点的左右邻居改写而来；
  -2 由 measLinePathPredict 在「直线插补位姿静态不可达且扰动失败」时产生。

验证结论（combinedEdgePath_debug.json 实测）：
- combinedEdgePath 里有 2 个 moveType=0 的点，确认就是转台过渡点
  （isImportant=false, isPathJoint=true, moveType=0，与 main.cpp 约 556-558 行一致）。
- 过渡点#1：partRotateAngle=0，joints J6=-155.8°；它后面那个点 partRotateAngle=90，
  J6=+32.4° —— J6 单轴跳变约 188°。过渡点#2（partRotateAngle=90）→ 下一点
  （partRotateAngle=180）同样是大跳变。
- 关键：过渡点与它后面那个点的 partRotateAngle 不同，分属 aaa 的两个不同分组。
  applyMeasLinePathPredict 是【逐组】调用的，组与组之间的接缝点对
  （上一组最后一点 = 过渡点，下一组第一点）从来不会被传进 measLinePathPredict
  → 永远不产生 -2 → 永远不进入任何一组的 spliceRRTIntoDensePath / RRT。
- 因此根因是：转台过渡点（moveType=0）正好卡在转台角度分组的接缝上，而逐组处理
  使这段接缝运动对「直线预测 + RRT」机制结构性不可见。
- 推论：修复【不能】放在 applyMeasLinePathPredict 内（它只看单组，永远看不到接缝）。

【具体例子】
JBI（D:/output/JBI/YASKAWA.JBI）里 C00270(MOVL) → C00272(MOVJ)，J 各轴大跳变、
中间无任何 RRT 中间点。C00272 是一个 moveType=0 的过渡点。（C00271 是导出阶段
insertMovlToMovjTransitions 插入的拷贝点，不是规划点。）

【请你做的】
第一步 —— 先确认（验证已基本完成，复核即可，不要盲改）：
- 若 D:/output/predictedAllPath_debug.json 已存在（由 main.cpp 一个调试块导出，
  含 moveType/isImportant/isPathJoint/partRotateAngle/joints），读它复核：
  找出所有 moveType==0 的点，确认它们与相邻分组之间的接缝运动确实没有 RRT
  中间点。若文件不存在，先重新编译运行一次生成。
- 把复核结果先告诉用户。

第二步 —— 设计并实现修复（修复必须作用于【拼接后的整条路径 / 分组之间的接缝】，
不能放在 applyMeasLinePathPredict 内）：
- 在 main.cpp 的 aaa 逐组循环之后，各组已处理完并可拼成完整路径
  （参考已有的 predictedAllPath：遍历 aaa 把每个 it.second 依次拼接）。
- 对【相邻两组的接缝点对】（上一组最后一点、下一组第一点）做段间运动级
  可行性判断（关节空间跳变超阈值，或对两点间关节空间直线采样逐点
  collision/关节限位检查）。
- 对判定为「不可行」的接缝对 (A,B)：用现有 RRT 机制规划这段——可直接调
  planRRTBetween(A, B, rrtCtx, midJoints)，或构造长度为 2 的 -1 run 再走
  spliceRRTIntoDensePath；把 RRT 中间点插入接缝处。
- A、B 必须有合法（非 NaN）关节角。
- 注意碰撞环境问题：接缝跨越两个不同转台角度（如 0° 与 90°），RRT 该用哪个
  角度的碰撞环境需要先想清楚——这正是过渡点设计上要解决的问题。如果不确定，
  先把这个设计问题提出来问用户，不要擅自决定。
- 修复后该接缝在 JBI 里应表现为「一串 RRT 中间 MOVJ 点」，而非单条大跳变 MOVJ。

【项目固有约束（务必遵守）】
- 只能纯新增代码（新增函数、新增调用行都算新增），不修改/删除任何已有行；
  新增逻辑必须写注释。
- 在 ViewPoint / 数据层修复，不要在 JBIExporter 输出层手动拼行。
- 不确定时先排查、先问，不要盲目改。
```
