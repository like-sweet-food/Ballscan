## PlanPathRRTconnect.cpp 是否真的被链接进 QtWidgetsApplication3 — 排查提示词

背景项目：D:\test\QtWidgetsApplication3（Qt Widgets，Yaskawa 机器人切边路径规划）

### 现象
在 `D:\test\QtWidgetsApplication3\process_line_result.cpp:182` 调用 `PathPlan::PlanPathRRTconnect_new(...)` 之后单步：
- `res.goal_ind` 返回 `-189000` 这种大负数（不是预期的 0 或 1，典型未初始化内存值）
- 在 `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp` 中打断点**不命中**，没法步入

### 强烈怀疑的根因
`PlanPathRRTconnect.cpp` 这份实现根本没被编译进 QtWidgetsApplication3，调用链接到的是别处的同名符号（旧 .lib / 空实现 / stub），所以：
- 函数没真正向 `Result_BZD` 写入字段 → `goal_ind` 是栈垃圾 → `-189000`
- 当前进程加载的 PDB 与源不匹配 → 断点不能 bind

### 请按这个顺序排查（不要改算法代码）

**第 1 步：在 .vcxproj 里查 PlanPathRRTconnect**

对 `D:\test\QtWidgetsApplication3\*.vcxproj` 和 `*.vcxproj.filters`：
- grep `PlanPathRRTconnect` —— 是否作为 `<ClCompile Include="...\PlanPathRRTconnect.cpp" />` 列出？
- grep `IZ_new_transcode` —— 该目录是否出现在 `<AdditionalIncludeDirectories>`？
- grep `<ExcludedFromBuild>` —— 当前 Configuration|Platform 下是否被排除？
- 查 `<Link>` 节：`<AdditionalDependencies>` 与 `<AdditionalLibraryDirectories>` 有没有指向某个 `IZ_new_transcode*.lib`？找出那份 .lib 实际路径

三选一报回：
- (a) 作为 `<ClCompile>` 直接编译
- (b) 仅通过预编译 .lib 链接
- (c) 完全没出现（链接器靠别的途径解析符号，或根本没解析到）

如果是 (b)：把 .lib 修改时间和 `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp` 修改时间贴出来对比。.lib 早于 .cpp 就是确凿证据：链的是旧实现。

**第 2 步：用 VS 的 Modules / 断点气泡确认**

复现时（在 `process_line_result.cpp:182` 命中断点时）：
- `Debug → Windows → Modules`：找包含 `PlanPathRRTconnect` 的模块，记录 Path、Symbol Status、Timestamp
- `Debug → Windows → Output`：搜 "PDB" 关键字，看是否有 "PDB does not match image" 警告
- 鼠标悬停在 `PlanPathRRTconnect.cpp` 内那个打不进的红色断点上，把气泡提示原文贴回来。如果是 "No symbols have been loaded for this document" — 100% 确认这份源码没被链接

**第 3 步：查 Result_BZD 默认值**

打开 `PlanPathRRTconnect.h`（或 `Result_BZD` 定义所在头文件）：
- `goal_ind` 字段类型，**有没有 in-class initializer**（`int goal_ind = 0;`）
- `Result_BZD` 有没有默认构造函数

如果 `goal_ind` 没默认值且结构是 trivially-default-constructible —— 那么调用方写 `Result_BZD res = pp.PlanPathRRTconnect_new(...)`，只要函数体没真正赋值（空 stub 或链错了符号），`res.goal_ind` 就是栈上随机字节，与 `-189000` 现象一致。

**第 4 步：不依赖断点的判断（最直接）**

在 `D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp` 内 `PlanPathRRTconnect_new` 函数的**第一行**插一行临时探针：
```cpp
std::cerr << "[PlanPathRRTconnect_new ENTER] " << __FILE__ << ":" << __LINE__ << std::endl;
```
完整 Rebuild（不是 Build —— 必须强制重编以打破任何 stale .obj/.lib 缓存）然后跑复现。看 stderr：
- **没有这行** → 这份 .cpp 完全没参与链接，第 1 步的 .vcxproj 配置就是根源
- **有这行但断点仍打不进** → .cpp 被链接了，但 PDB 不匹配。Rebuild 一次就好
- **有这行且 goal_ind 仍是垃圾值** → 真的进了这份函数但函数内部没给 goal_ind 赋值，需查函数体逻辑

确认根因后**立即删除**这行 cerr。

### 报回结论时请给

1. 第 1 步的 .vcxproj 检查结论（a/b/c），如果是 b 给两个修改时间
2. 第 2 步 VS Modules 状态截图或文字 + 断点气泡原文
3. 第 3 步 `Result_BZD` 与 `goal_ind` 的定义片段
4. 第 4 步插 cerr 后 stderr 是否出现 `[PlanPathRRTconnect_new ENTER]`

### 约束
- 不要动 `PlanPathRRTconnect.cpp` 的算法逻辑
- 不要在 `process_line_result.cpp` 里 try/catch 吞错把 -189000 改成 0
- 第 4 步的 cerr 是临时探针，结论拿到后删掉
- 在搞清"链接的到底是不是这份源码"之前，**不要**调任何 `m_maxNodes` / `m_step_size` / `J4_6_constraints` 参数 —— 参数调了也不会生效
- 在 ViewPoint / 数据层修复，不在 JBIExporter 等输出层补丁

### 相关上下文
- 上一份排查提示词：`D:\test\QtWidgetsApplication3\planRRTBetween_debug_prompt.md`（讲 RRT 内部参数与 [RRT_STATS] 日志，本提示词的前置是它）
- 调用链：`BallScan.cpp:1481 processLineResult → process_line_result.cpp:337 planRRTBetween → process_line_result.cpp:182 PathPlan::PlanPathRRTconnect_new`
- RRTContext 静态装配已检查无问题（BallScan.cpp:1467-1481 + makeRRTContext @ 130-147，四个函数指针硬编码赋值，不会留空）
- "RRTContext 未完整配置" / "未找到机器人 X 的 RobotTechParameters" 这两条 cerr **未在你的复现里出现过**，所以已排除"上下文没装好"这条路径
