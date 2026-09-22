## 重编 PathPlan.lib 让 QtWidgetsApplication2 调试 RRT 生效 — 执行提示词

背景项目：D:\test\QtWidgetsApplication3（Qt Widgets，Yaskawa 机器人切边路径规划）

### 已查明的根因（不需要再查证一遍）

QtWidgetsApplication2.vcxproj 通过 `<AdditionalDependencies>` 链接 `..\3part\m_lib\PathPlan.lib`（vcxproj 第 75/84 行），而这份 .lib 由另一个工程 `D:\test\Vision_1\vision_ljs\vision_ljs\vision_cpp.vcxproj` 编译产出（该 vcxproj 第 27 行 `<ProjectName>PathPlan</ProjectName>`，x64 配置为 StaticLibrary）。

实测时间戳：
- `D:\test\3part\m_lib\PathPlan.lib`：**2026-05-14 15:42**
- `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp`：**2026-05-15 12:22**
- `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\rrtExtend.cpp`：**2026-05-15 12:22**

→ 用户对 RRT 源码的修改从未进入运行时。具体表现：
- `process_line_result.cpp:182` 调用 `PlanPathRRTconnect_new` 后 `res.goal_ind = -189000`（旧 lib 内部产生的非预期值）
- 在 `PlanPathRRTconnect.cpp` 内打的断点不命中（lib 的 PDB 没和当前源同步，或根本不在 m_lib 旁边）

`PathPlan.lib` 实际包含的源文件（来自 vision_cpp.vcxproj 解析，共 45 个）：
- `..\..\FTC_TZ\IZ_new_transcode\` 下 25 个：calculate_time, computeXYZBounds, ConfigureCheck, DressCheck, findNearest, getFinalResult, getSample, ijk2tm, ijk2wprnew, intersect, IZ_new, IZ_new_tol, jianzhi230628, JointSampling, multinomial, newnode_check, **PlanPathRRTconnect**, PlanPathRRTconnect_tol, RobotJointConstriant, **rrtExtend**, rrtExtend_tol, setPath, steering, Utils
- `..\C2A\` 下 9 个：CollisionDetectorSingleton, DelaunayTriangulation, func, InterpMotion, model, stopwatch, Triangle, TriDist, Vector3D
- vision_cpp 项目自身目录 11 个：find_perpendicular_vector, IZ, rotate_about_arbitrary_axis, rotation_matrix_to_euler, SamplePoseEllipsoid, Schmidt_orthogonalization, solveijkset, tingyxml2, unique_stable, 源.cpp（中文文件名）

### 目标
重编 `PathPlan.lib`，让 QtWidgetsApplication2 链接到与当前 RRT 源码同步、且带匹配 PDB 的 lib。完成后：
- 在 `PlanPathRRTconnect.cpp` / `rrtExtend.cpp` 内打断点能命中
- `res.goal_ind` 不再是 `-189000` 这种垃圾值
- 后续改 RRT 源码只需重编 vision_cpp 即可生效

### 执行步骤（按序）

#### 第 0 步：确认当前 QtWidgetsApplication2 的活动配置
打开 `D:\test\QtWidgetsApplication3\QtWidgetsApplication2.sln`（或主工程的 .sln），看顶部工具栏：
- Configuration（Debug 还是 Release）
- Platform（应该是 x64）

后面所有步骤都用**这个相同的 Configuration|Platform** 来编译 vision_cpp。两边不一致会出 CRT / 调试信息不匹配，链接报错或断点不能 bind。

#### 第 1 步：把 vision_cpp 加进解决方案（如果还没有）
- 在 VS 主工程的 Solution Explorer 右键 Solution → Add → Existing Project
- 选 `D:\test\Vision_1\vision_ljs\vision_ljs\vision_cpp.vcxproj`
- 加进来后 Solution Explorer 会出现 PathPlan 项目（注意 ProjectName 是 PathPlan，不是 vision_cpp）

#### 第 2 步：在 PathPlan 项目上右键 → Properties → 验证以下设置（与 QtWidgetsApplication2 兼容）
针对你第 0 步看到的 Configuration|Platform：

| 项 | 期望值 | 在哪看 |
|---|---|---|
| Configuration Type | Static library (.lib) | General |
| Platform Toolset | v143（与主工程一致） | General |
| C++ Language Standard | ISO C++20 (/std:c++20) | C/C++ → Language |
| Runtime Library | Multi-threaded Debug DLL (/MDd) for Debug 或 Multi-threaded DLL (/MD) for Release（与主工程一致） | C/C++ → Code Generation |
| Debug Information Format | Program Database (/Zi) | C/C++ → General |
| Generate Debug Information | Yes (/DEBUG) | Linker → Debugging（虽然 lib 不链接，但 .pdb 会随 .obj 生成） |

**特别注意**：vision_cpp.vcxproj 当前 Release|x64 没显式设 RuntimeLibrary，会用默认值 `MultiThreaded`（/MT）。这与 QtWidgetsApplication2 的 `MultiThreadedDLL`（/MD）**不兼容**，链接时会报 LNK4098 / 重定义。Release|x64 配置必须显式改成 `Multi-threaded DLL (/MD)`。

#### 第 3 步：处理 .lib 输出位置（三选一，推荐 C）
默认 vision_cpp 编出来的 PathPlan.lib 会放在 `D:\test\Vision_1\vision_ljs\vision_ljs\x64\<Configuration>\PathPlan.lib`，**不会**自动覆盖 `D:\test\3part\m_lib\PathPlan.lib`。

- **方案 A**：每次编完手动 copy `PathPlan.lib` + `PathPlan.pdb` 到 `D:\test\3part\m_lib\` —— 最不易错，最不优雅
- **方案 B**：在 PathPlan 项目 Properties → General → Output Directory 改成 `$(SolutionDir)..\3part\m_lib\`（路径相对于 sln 位置自行调整）—— 编完直接覆盖，但污染 m_lib（原本是供应商 prebuilt 目录）
- **方案 C（推荐）**：改 `D:\test\QtWidgetsApplication3\QtWidgetsApplication2.vcxproj`：
  1. 把 `<AdditionalDependencies>` 末尾的 `..\3part\m_lib\PathPlan.lib` 删掉
  2. 在 Solution Explorer 中 QtWidgetsApplication2 项目 → 右键 → Add → Reference → 勾选 PathPlan 项目
  3. 这样 MSBuild 会自动把 PathPlan 的输出 .lib 链给 QtWidgetsApplication2，且 .pdb 也会被自动找到

  方案 C 的好处：保持 m_lib 是"供应商不动"的语义；以后改 RRT 源 → 直接 Rebuild Solution，主工程会感知 PathPlan 的依赖并自动重链。

#### 第 4 步：Rebuild PathPlan
在 Solution Explorer → PathPlan 项目右键 → **Rebuild**（不是 Build，必须强制重编以打破 stale .obj）。

如果失败：
- LNK 报符号缺失 → 看是不是少了某个依赖头文件路径，检查 PathPlan 项目 IncludePath 里 D:\test\eigen-3.4.0、D:\test\3part\m_include 等是否存在
- C++ 编译错误 → 大概率是 PlanPathRRTconnect.cpp / rrtExtend.cpp 自身有错（用户最近改过），照报错修
- 文件 `源.cpp` 编译错（中文文件名编码问题）→ 给该文件单独加 `/utf-8` 编译选项，或临时把 `<ClCompile>` 列表里这一项标 ExcludedFromBuild=Yes 看是否仅这个文件的问题

成功后产物路径（默认）：
- `D:\test\Vision_1\vision_ljs\vision_ljs\x64\<Configuration>\PathPlan.lib`
- `D:\test\Vision_1\vision_ljs\vision_ljs\x64\<Configuration>\PathPlan.pdb`

#### 第 5 步：让 QtWidgetsApplication2 用上新 lib
- 方案 A：把 `PathPlan.lib` + `PathPlan.pdb` 一起 copy 到 `D:\test\3part\m_lib\`
- 方案 B：第 3 步已让 lib 直接输出到 m_lib，无需操作
- 方案 C：第 3 步加了 Reference，无需操作

#### 第 6 步：Rebuild QtWidgetsApplication2

#### 第 7 步：验证
1. 在 `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp` 第一行可执行代码处打断点
2. F5 启动调试，触发 RRT 调用路径（即跑到 BallScan 的测量线规划）
3. **预期**：断点命中
4. 在 `D:\test\QtWidgetsApplication3\process_line_result.cpp:182` 之后单步，`res.goal_ind` 不再是 `-189000`
5. 顺便看 VS → Debug → Windows → Modules，找包含 PathPlan 符号的模块，Symbol Status 应该是 "Symbols loaded"

如果断点仍打不进：
- 鼠标悬停在那个红色断点上，VS 会弹气泡说明原因，把气泡原文贴出来
- 检查 PathPlan.pdb 修改时间是否与 .lib 一致
- 检查主工程 .exe 旁边有没有 PathPlan.pdb（链接器有时把静态库的 pdb 合并到主 pdb 里）

### 约束（必须遵守）
- 不要把 IZ_new_transcode 下的 .cpp 加进 QtWidgetsApplication2.vcxproj 直接编译（之前对话讨论过的"方案 A"，已弃—— 因为 PathPlan.lib 还包了 C2A 9 个文件 + vision_cpp 自身 11 个文件，搬过去会引起一系列符号/头文件冲突）
- 不要在 `process_line_result.cpp` 里加 try/catch 把 -189000 吞掉变成 0
- 不要调任何 RRT 算法参数（m_maxNodes / m_step_size / J4_6_constraints）—— 在 lib 真正同步前调了不生效
- 在 ViewPoint / 数据层修复，不在 JBIExporter 等输出层补丁
- 用户终端有乱码看不见 `std::cout`/`cerr` 输出，**所有验证用 VS 调试器**（断点、Watch 窗口、Modules 窗口、断点气泡），不要让用户跑 cerr 探针

### 相关上下文
- 上一份排查提示词：
  - `D:\test\QtWidgetsApplication3\planRRTBetween_debug_prompt.md`（讲 RRT 内部参数、[RRT_STATS] 日志，需在 lib 同步后才能用）
  - `D:\test\QtWidgetsApplication3\planRRTBetween_link_check_prompt.md`（前一轮的链接检查提示词，本提示词是其执行落地版）
- 关键代码定位：
  - `BallScan.cpp:1473` makeRRTContext 调用、`:1481` processLineResult 调用
  - `process_line_result.cpp:174` "RRTContext 未完整配置" cerr（已确认未触发）
  - `process_line_result.cpp:182` PlanPathRRTconnect_new 调用（goal_ind 异常的位置）
- 主工程 vcxproj 关键行：`QtWidgetsApplication2.vcxproj:65, 70`（IncludePath）、`:75, 84`（AdditionalDependencies）
- PathPlan 工程：`D:\test\Vision_1\vision_ljs\vision_ljs\vision_cpp.vcxproj`，行 27（ProjectName=PathPlan）、行 44/50（StaticLibrary）、行 187/190（PlanPathRRTconnect.cpp / rrtExtend.cpp 包含）
