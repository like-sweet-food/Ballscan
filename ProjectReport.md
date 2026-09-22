# QtWidgetsApplication2 项目详细报告

> 最后更新：2026-03-27
> 编译环境：VS2022 / MSVC v143 / C++20 / Release x64 / Qt 5.14.2

---

## 1. 项目概述

**QtWidgetsApplication2** 是一个基于 Qt 的 **工业机器人离线视觉规划系统**（Robot Offline View Planning System）。该系统用于自动规划 Yaskawa 机器人的扫描路径和测量路径，并将结果导出为 Yaskawa JBI 机器人程序文件，可直接用于工厂机器人执行。

**核心能力：**
- 基于点云的 PCA 分析与滑动窗口网格生成，规划表面扫描轨迹
- 基于 BiRRT（双向快速随机树）的无碰撞路径规划
- 通过 FK/IK 运动学求解进行可达性验证
- 多级碰撞检测与安全校验（关节限位 → IK可行性 → 环境碰撞 → 自碰撞）
- 导出 Yaskawa JBI 格式的机器人程序（支持 8 轴：6 机器人轴 + 转台 + 滑轨）

---

## 2. 项目目录结构

```
D:\test\QtWidgetsApplication2\
├── main.cpp                              # 应用程序入口（35 行）
├── QtWidgetsApplication2.h               # 主窗口头文件（17 行）
├── QtWidgetsApplication2.cpp             # 主窗口实现（11 行）
├── QtWidgetsApplication2.ui              # Qt UI 设计文件（29 行）
├── QtWidgetsApplication2.qrc             # Qt 资源文件（空）
├── QtWidgetsApplication2.vcxproj         # Visual Studio 项目文件（143 行）
├── QtWidgetsApplication2.sln             # Visual Studio 解决方案文件
│
├── robot_planner_entry.cpp               # 机器人规划算法入口（197 行）
│
├── Types.h                               # 共享数据类型定义（137 行）
├── RobotConfig.h / RobotConfig.cpp       # YAML 配置加载（191 + 126 行）
├── ViewPlanner.h / ViewPlanner.cpp       # 视觉规划核心算法（132 + 1003 行）
├── BiRRTPlanner.h / BiRRTPlanner.cpp     # 双向 RRT 路径规划（164 + 334 行）
├── JBIExporter.h / JBIExporter.cpp       # Yaskawa JBI 文件导出（140 + 269 行）
│
├── [已弃用，未参与编译]
│   ├── RobotKinematics.h / .cpp          # 原 FK/IK 封装层（已移出编译）
│   └── CollisionChecker.h / .cpp         # 原碰撞检测封装层（已移出编译）
│
├── ProjectReport.md                      # 项目详细报告（本文件）
├── ProjectLogic.md                       # 项目架构逻辑文档
├── robot_planner_README.md               # 规划模块开发文档
│
└── .vs/                                  # Visual Studio 缓存

D:\test\3part\m_include\                  # 二次开发接口（只读）
├── parameters_type.h                     # 核心类型 + 函数指针 typedef
├── kinematicsApi.h                       # 单例 API 包装（bind + 调用）
├── robot_robotKinematicsCollisionInterface.h  # 静态接口实现类
├── math_utils.h                          # 数学工具函数（rotm2eul_ZYX 等）
├── ConfigureCheck.h                      # 构型检查
└── linkage_axis_joint_limit_check.h      # 连杆轴限位
```

### 代码量统计

| 文件 | 行数 | 说明 |
|------|------|------|
| ViewPlanner.cpp | 1003 | 主规划算法（项目最大文件） |
| BiRRTPlanner.cpp | 334 | BiRRT 路径规划 |
| JBIExporter.cpp | 269 | JBI 文件导出 |
| robot_planner_entry.cpp | 197 | 算法入口与流程编排 |
| RobotConfig.h | 191 | 配置结构体定义与默认值 |
| BiRRTPlanner.h | 164 | BiRRT 接口定义 |
| JBIExporter.h | 140 | JBI 导出接口定义 |
| Types.h | 137 | 数据类型定义 |
| ViewPlanner.h | 132 | 视点规划接口定义 |
| RobotConfig.cpp | 126 | 配置加载实现 |
| **合计（有效代码）** | **~2,730** | 不含弃用文件和文档 |

---

## 3. 构建系统与编译环境

### 编译配置

| 项目 | 值 |
|------|-----|
| **构建工具** | MSBuild (Visual Studio 2022) |
| **编译器** | MSVC v143 (VC2022) |
| **C++ 标准** | C++20 (`/std:c++20`) |
| **字符编码** | UTF-8 (`/utf-8`) |
| **目标平台** | x64 |
| **有效配置** | Release x64（Debug 存在 CRT 不匹配问题） |
| **编译状态** | 0 错误, 0 警告 |

### Qt 集成

| 项目 | 值 |
|------|-----|
| **Qt 版本** | 5.14.2 (msvc2017_64) |
| **Qt 路径** | `C:\Qt\5.14.2\msvc2017_64` |
| **Qt 构建工具** | MOC / UIC / RCC（MSBuild 集成） |
| **使用的 Qt 模块** | Core, Gui, Widgets, Xml, Sql, OpenGL, Concurrent, OpenGLExtensions, Charts |

### 命令行编译

```bat
"D:\VS2022 v17\MSBuild\Current\Bin\MSBuild.exe" ^
  "D:\test\QtWidgetsApplication2\QtWidgetsApplication2.vcxproj" ^
  /p:Configuration=Release /p:Platform=x64 /t:Build /nologo
```

> **注意**：只能用 Release 配置，Debug 会因三方库 CRT 不匹配报 LNK2038 错误。
>
> **注意**：编译前确保程序未在运行，否则链接报 LNK1104（exe 被锁）。

---

## 4. 第三方依赖库

| 库 | 版本 | 用途 | 路径 |
|----|------|------|------|
| **Qt** | 5.14.2 | GUI 框架, 信号/槽机制 | `C:\Qt\5.14.2\msvc2017_64` |
| **OpenCASCADE** | 7.90 | CAD 内核, 几何运算 | `..\3part\opencascade_7.90` |
| **Open Scene Graph (OSG)** | 3.6.5 | 3D 图形渲染 | `..\3part\OSG3.6.5` |
| **Eigen** | 3.4.0 | 线性代数 (Header-only，项目中未实际使用) | `..\3part\eigen-3.4.0` |
| **QtXlsx** | - | Excel 文件读写 | Qt 5.14.2 扩展模块 |
| **PQP** | - | 碰撞检测（Proximity Query Package） | `..\3part\m_lib\PQP_Test.lib` |
| **KinematicsApi** | 自研 | 机器人 FK/IK/碰撞接口 | `..\3part\m_include` |
| **Offline programme** | 自研 | 机器人运动学/碰撞主库 | `..\3part\m_lib` |
| **MRSP_OSG** | 自研 | 机器人路径规划 OSG 封装 | `..\3part\m_lib` |

**链接库总数：** ~70 个（包括 OpenCASCADE ~50 个、OSG 9 个、自研库 5 个等）

> **说明**：项目数学运算（PCA、矩阵操作）全部手写实现，不依赖 Eigen，尽管 Eigen 头文件在 include 路径中。

---

## 5. 数据类型定义（Types.h）

### 基础类型

```cpp
namespace robot_planner {
    using Matrix4d   = std::vector<std::vector<double>>;  // 4×4 行优先
    using Vec3d      = std::array<double, 3>;
    using JointVec6  = std::array<double, 6>;
    using PointCloud = std::vector<Vec3d>;
}
```

### 核心数据结构

```cpp
// 视点表示
struct ViewPoint {
    Matrix4d  globalT;           // 用户坐标系下的 4×4 TCP 位姿
    JointVec6 joints;            // 对应关节角（度）
    bool      isImportant;       // 关键点标记（行首/行尾）
    bool      isPathJoint;       // BiRRT 中间避障点
    int       moveType;          // 0=MoveJ(关节运动), 1=MoveL(直线运动)
    double    partRotateAngle;   // 转台旋转角度
    Matrix4d  partTransform;     // 转台旋转矩阵
};

// BiRRT 节点
struct RRTNode {
    JointVec6 theta;             // 当前关节角（弧度）
    JointVec6 thetaPrev;         // 父节点关节角
    double    dist;              // 到父节点距离
    int       indPrev;           // 父节点索引（1-based，兼容 MATLAB）
    bool      lastFlag;          // 叶子节点标记
};

using RRTTree = std::vector<RRTNode>;

// 规划结果
struct PlanResult {
    std::vector<ViewPoint> waypoints;  // 有序视点列表
    bool        success;               // 规划是否成功
    std::string errorMsg;              // 失败原因
};

// 滑动窗口网格单元
struct GridCell {
    Vec3d  centerPos;            // 中心点（局部坐标）
    Vec3d  centerNormal;         // 表面法向量
    Vec3d  xAxis;                // 扫描方向
    bool   isImportant;          // 关键网格点
};

// 碰撞场景
struct BVHScene {
    double   rotateAngle;        // 转台旋转角（度）
    Matrix4d transformMatrix;    // 变换矩阵
    std::shared_ptr<PQP_Model> partModel;       // 工件 PQP 碰撞模型
    std::shared_ptr<PQP_Model> fixtureModel;    // 夹具 PQP 碰撞模型
    std::shared_ptr<PQP_Model> measTableModel;  // 转台 PQP 碰撞模型
    std::shared_ptr<PQP_Model> envModel;        // 环境 PQP 碰撞模型
    PointCloud rotatedPoints;    // 旋转后工件点云
    PointCloud rotatedFixture;   // 旋转后夹具点云
    PointCloud rotatedMeasTable; // 旋转后转台点云
    PointCloud env;              // 环境点云
};
```

---

## 6. 核心类与架构

### 类关系图

```
main.cpp
  │
  ├─ QApplication
  ├─ MainWindow (QMainWindow)
  │   └─ signal: start() → slot: runRobotPlanner()
  │
  └─ KinematicsApi::instance().bind(
       robot_forward_solution_fn,    // 正运动学
       robot_inverse_solution_fn,    // 逆运动学
       collision_test,               // 碰撞检测
       cylinderCollisionTest,        // 圆柱碰撞
       looseAngleIk,                 // 松弛角度 IK
       setSafeDistance                // 安全距离设置
     )

runRobotPlanner()  [robot_planner_entry.cpp]
  │
  ├─ RobotConfig ←── YAML 配置文件 + robots_tech_params 前端参数
  │
  ├─ ViewPlanner（核心规划器）
  │   ├─ planSurfaceViews()   →  表面扫描轨迹
  │   ├─ planMeasureViews()   →  测量点轨迹
  │   └─ 内部调用 BiRRTPlanner + FK/IK/碰撞函数指针
  │
  ├─ BiRRTPlanner（路径规划器）
  │   ├─ plan()               →  双向 RRT 规划
  │   ├─ smoothPath()         →  路径平滑
  │   └─ interpolateSegment() →  线性插值+碰撞检查
  │
  └─ JBIExporter（文件导出器）
      └─ exportToJBI()        →  生成 .JBI 机器人程序
```

### 各类职责

**ViewPlanner（1003 行）— 主规划器**
- PCA 分析点云，确定主法向量方向（手写 3×3 Jacobi 特征分解，无 Eigen 依赖）
- 滑动窗口网格生成，蛇形扫描覆盖工件表面
- 视点位姿生成与可达性检测（IK + 碰撞 + 自碰撞）
- 测量点路径规划（通过 BiRRT 进行无碰撞路径连接）
- 坐标系变换（用户坐标系 ↔ 基座坐标系 ↔ 工具坐标系）
- 静态数学工具：normalize3, dot3, cross3, multiplyTransforms, invertTransform

**BiRRTPlanner（334 行）— 关节空间路径规划器**
- 双向快速随机探索树（Bidirectional RRT）算法
- 参数：步长 0.17 rad, 目标偏置 10%, 方向偏置 0.7, 最大迭代 5000 次
- 连接阈值 0.87 rad (~50°), 树大小限制 10000 节点
- 路径平滑（随机快捷尝试，最多 100 次）

**JBIExporter（269 行）— Yaskawa 程序导出器**
- 关节角度 → 电机脉冲转换（8 轴脉冲比）
- Yaskawa 轴坐标变换（L=90°-J2, U=J3-90°, B=-J5）
- 支持 8 轴配置（6 机器人轴 + 转台 + 滑轨）
- 自动插入中间过渡点（转台 >10° 或地轨 >50mm 时）
- 速度参数配置（VJ=线速度 cm/min, V=关节速度 %）

**RobotConfig（191 + 126 行）— 配置管理**
- 从 YAML 文件加载所有参数
- 支持从前端 `robots_tech_params` 覆盖关节限位和 Home 位置
- 提供全局配置访问
- 自动填充文件路径（rootPath + partName → 8 条路径）

---

## 7. 核心算法流程

### 阶段一：表面扫描规划（planSurfaceViews）

```
1. PCA 分析
   ├─ 输入：工件三维点云
   ├─ 计算质心
   ├─ 计算 3×3 协方差矩阵
   ├─ Jacobi 特征分解（手写实现，无 Eigen）
   └─ 最小特征值对应列 → 主法向量

2. 坐标系变换
   ├─ 构建局部坐标系（Z 对齐法向量）
   ├─ 将点云变换到局部坐标系
   └─ 保存 局部→全局 变换矩阵

3. 滑动窗口网格生成
   ├─ 离散化点云包围盒
   ├─ 窗口尺寸：[60, 60, 60] mm（可配置）
   ├─ 步长：[60, 60, 60] mm（可配置）
   ├─ 蛇形扫描（奇数行 Y 反向）
   └─ 每个网格单元包含 centerPos, centerNormal, xAxis

4. 视点位姿生成（每个网格单元）
   ├─ Z 轴沿表面法向量方向
   ├─ 计算最优 X 轴（最小化关节变化）
   ├─ 构建用户坐标系下的 TCP 位姿矩阵
   ├─ 通过 IK + 碰撞检测验证可达性
   └─ 不可达时：adjustViewPose() 沿 Z 轴偏移重试（5mm/步，最多 5~10 步）

5. 视点排序与路径连接
   ├─ 按空间顺序连接可达视点
   ├─ 直线路径：collision_test 检测 → 无碰撞则线性插值
   ├─ 有碰撞 → BiRRT 规划无碰撞路径
   └─ 生成 MoveL（直线）或 MoveJ（关节）运动指令

6. HOME → 第一视点的 MoveJ 规划
   ├─ 直接碰撞检测
   ├─ 有碰撞 → BiRRT 规划
   └─ 插入 homeNode (+ midNode)
```

### 阶段二：测量点规划（planMeasureViews）

```
1. 逐点规划
   ├─ 对每个测量点：
   │   ├─ 根据坐标和法向量构建 TCP 位姿
   │   │   Z 轴 = 法向量, X 轴 = computeOptimalXAxis(Z, 参考方向)
   │   │   位置 = 测量点坐标 + measSafeHeight × Z
   │   ├─ 遍历转台角度 rotateRange = [0°, 90°, 180°, 270°]
   │   │   ├─ rotatedTcp = Rz(rotAngle) × tcpPose
   │   │   ├─ checkViewPointCollision() → 直接调用 IK + collision
   │   │   └─ 首个无碰撞解 → bestVp，记录 partRotateAngle
   │   └─ 通过 BiRRT 从上一视点规划无碰撞路径

2. BiRRT 路径规划（关节空间）
   ├─ 双树搜索：起始树 + 目标树
   ├─ 每次迭代：
   │   ├─ 90% 概率：关节空间均匀随机采样
   │   ├─ 10% 概率：直接采样目标（目标偏置）
   │   ├─ 加权方向：随机方向 + 目标方向 × 0.7
   │   ├─ 前进步长：0.17 rad/步
   │   └─ 碰撞检测
   ├─ 连接条件：双树距离 < 0.87 rad
   └─ 最大迭代：5000 次，单树最大 10000 节点

3. 路径平滑
   ├─ 随机快捷尝试（最多 100 次）
   ├─ 移除冗余中间节点
   └─ 对缩短路径重新碰撞检查
```

### 阶段三：JBI 文件导出

```
1. PlanResult → N×8 矩阵 [J1~J6, turnTable, slidePos]
2. insertExtAxisSteps(): 外轴变化 > 阈值时插入过渡步
   （转台 >10°/地轨 >50mm，过渡步中机器人关节角保持不变）
3. Yaskawa 轴坐标变换（L=90°-J2, U=J3-90°, B=-J5）
4. 关节角度 × 脉冲比 → 电机脉冲值
5. 生成 JBI 文件：
   ├─ /JOB 头（工具号、速度参数）
   ├─ //POS 位置数据区（C00000 脉冲值行）
   ├─ //INST 程序体（MOVJ/MOVL 指令）
   └─ END
```

---

## 8. 碰撞检测策略

系统采用多级碰撞检测策略：

| 级别 | 检测方式 | 说明 |
|------|----------|------|
| 1 | 关节限位检查 | IK 求解前检查关节角是否在限位范围内 |
| 2 | IK 可行性检查 | 多解过滤，根据 Yaskawa 轴变换后的限位筛选有效解 |
| 3 | 环境碰撞检测 | 通过外部 `collision_test()` 函数检查 |
| 4 | 自碰撞检测 | 通过 FK 获取 Link2/3/4 位姿，计算连杆夹角（< 27° 判定碰撞） |

**安全距离：** 默认 50mm（可配置，通过 `setCollisionTestSafeDistance` 设置）

---

## 9. Yaskawa 机器人参数

### 关节限位

| 轴 | 最小角度 | 最大角度 |
|----|----------|----------|
| J1 (S轴) | -170° | 170° |
| J2 (L轴) | -35° | 170° |
| J3 (U轴) | -160° | 105° |
| J4 (R轴) | -190° | 190° |
| J5 (B轴) | -140° | 140° |
| J6 (T轴) | -445° | 445° |

### 脉冲比（关节角度 → 电机脉冲）

| 轴 | 脉冲比 |
|----|--------|
| S轴 (J1) | 1413.522 |
| L轴 (J2) | -1604.267 |
| U轴 (J3) | 1994.364 |
| R轴 (J4) | -930.91 |
| B轴 (J5) | 986.08 |
| T轴 (J6) | -536.604 |
| Z轴 (转台) | 11.378 |
| E轴 (滑轨) | 130.383 |

### Yaskawa 轴变换约定

| 标准轴 | Yaskawa 映射 |
|--------|-------------|
| J1 | 直接映射 |
| J2 | 90° - J2 |
| J3 | J3 - 90° |
| J4 | 直接映射 |
| J5 | 取反 (-J5) |
| J6 | 直接映射 |

### Home 位置

```
J1=-90.07°, J2=-70°, J3=-50°, J4=0.07°, J5=60°, J6=-0.08°
```

---

## 10. 坐标系变换

系统涉及三个主要坐标系：

```
基座坐标系 (Base)  ←── T_user ──→  用户坐标系 (User)
                   ←── T_he  ──→  工具坐标系 (Tool/Camera)
```

**变换流程：**
```
用户坐标系 TCP 位姿
  ↓
T_base = inv(T_user) × tcpPose × inv(T_he)  → 基座坐标系
  ↓
提取 ZYX 欧拉角 → XYZWPR（W=绕Z, P=绕Y, R=绕X，单位度）
  ↓
调用 IK(robotName, xyzwpr) → 多组关节解
```

**默认 T_user 矩阵（基座 → 用户坐标系）：**
```
[  0,  1,  0, 8590 ]
[ -1,  0,  0, 4997 ]
[  0,  0,  1,  606 ]
[  0,  0,  0,    1 ]
```

**默认 T_he 矩阵（TCP → 相机手眼标定）：**
```
[ 0.0,   0.706365913117550,  0.707846873826260, -433.34 ]
[ 1.0,   0.0,                0.0,                  -1.26 ]
[ 0.0,   0.707846873826260, -0.706365913117550,  545.93 ]
[ 0.0,   0.0,                0.0,                    1.0 ]
```

**FK 返回值说明：**
```
robot_forward_solution_fn 返回 FowardKinematicsReslutstd：
  .position  — 参考坐标系下 [x, y, z]
  .tcp       — Base 坐标系下 4×4 矩阵
  .ut        — User 坐标系下 4×4 矩阵（项目使用此字段）
```

---

## 11. 配置文件结构（YAML）

```yaml
# 机器人标识
robot_name: YASKAWA_GP20HL
part_name: tail_door
root_path: ..\data\
result_name: tail_door_output

# 关节限位（含 10° 安全裕量）
joint_limit: [[-170,170], [-35,170], [-160,105], [-190,190], [-140,140], [-445,445]]
collision_tolerance: 50.0
angles_thresholds: [120, 120, 120, 120, 120, 120]

# 坐标系变换
T_user: [[0,1,0,8590], [-1,0,0,4997], [0,0,1,606], [0,0,0,1]]
T_he: [[0,0.706,0.707,-433.34], [1,0,0,-1.26], [0,0.707,-0.706,545.93], [0,0,0,1]]

# 算法参数
window_size: [60, 60, 60]          # 滑动窗口尺寸 (mm)
step_size: [60, 60, 60]            # 窗口步长 (mm)
safe_height: 0.0                   # 面扫安全高度
meas_safe_height: 250.0            # 测量安全高度
min_interpolate_dist: 5.0          # 最小插值距离
random_seed: 666                   # BiRRT 随机种子

# 相机参数
ball_radius: 30.0
rotate_theta: 40.0
cam_offset: 100.0
meas_rotate_theta: -40.0

# 转台参数
meas_table_axis_xyz: [8539.80, 2702.78, 0.0]
rotate_range: [0, 90, 180, 270]

# Home 位置
robot_home: [-90.07, -70.0, -50.0, 0.07, 60.0, -0.08]

# 外部轴
slide_pos: -3500.0
```

### 参数来源优先级

| 参数 | 来源 | 说明 |
|------|------|------|
| `robotName` | `robots_tech_params` map key | 前端传入（最高优先级） |
| `jointLimits[0..5]` | `RobotTechParameters.t1min~t6max` | 前端传入 |
| `robotHome` | `RobotTechParameters.initial_angle` | 前端传入 |
| 其他参数 | YAML → defaultConfig() | YAML 优先，缺省用默认值 |

---

## 12. 应用启动流程

```
main(argc, argv)
  │
  ├─ 创建 QApplication
  ├─ 创建 MainWindow 主窗口 (600×400)
  ├─ 绑定运动学函数指针到 KinematicsApi
  │   ├─ robot_forward_solution_fn  (正运动学)
  │   ├─ robot_inverse_solution_fn  (逆运动学)
  │   ├─ collision_test             (碰撞检测)
  │   ├─ cylinderCollisionTest      (圆柱碰撞)
  │   ├─ looseAngleIk              (松弛角度 IK)
  │   └─ setSafeDistance            (安全距离设置)
  ├─ 连接信号: MainWindow::start → runRobotPlanner()
  ├─ 显示窗口
  └─ 进入 Qt 事件循环 (app.exec())
        │
        └─ 当 start 信号触发:
           └─ runRobotPlanner("conf/robot_config.yaml", robots_tech_params)
              ├─ 加载 YAML 配置
              ├─ 从 robots_tech_params 覆盖关节限位和 Home 位置
              ├─ 获取三个函数指针
              ├─ 设置碰撞安全距离
              ├─ 加载点云（工件、夹具、环境）
              ├─ 加载测量点列表
              ├─ 创建 BiRRTPlanner
              ├─ 创建 ViewPlanner
              │
              ├─ 阶段 1: planSurfaceViews()  → 表面扫描轨迹
              ├─ 阶段 2: planMeasureViews()  → 测量点轨迹
              │
              └─ 阶段 3: JBIExporter
                 ├─ exportToJBI(surfaceResult) → _surface.JBI
                 └─ exportToJBI(measResult)    → _meas.JBI
```

---

## 13. 二次开发接口（外部依赖）

### 函数指针类型

```cpp
// 正运动学：robot_name + 关节角(度) + 连杆编号 → 位姿矩阵
using RobotFKFunction = FowardKinematicsReslutstd(*)(
    std::string robot_name,
    std::vector<double> jointsValue,
    LinkName orderNum);

// 逆运动学：robot_name + 末端欧拉角 → 多组关节解
using RobotIKFunction = std::vector<std::vector<double>>(*)(
    std::string robot_name,
    const XYZWPR& xyzwpr);

// 碰撞检测：robot_name + 起始关节 + 终止关节 → 是否碰撞
using CollisionTestFunction = bool(*)(
    const std::string& robot_name,
    const std::vector<double>& robot_startJoint,
    const std::vector<double>& robot_endJoint);
```

### 关键数据结构

```cpp
struct RobotTechParameters {
    std::vector<double> initial_angle;   // → cfg.robotHome
    RobotBrand brand;                    // 品牌枚举
    XYZWPR handEyeMatrix;               // 手眼标定矩阵
    double t1min, t1max;                 // → cfg.jointLimits[0]
    double t2min, t2max;                 // → cfg.jointLimits[1]
    double t3min, t3max;                 // → cfg.jointLimits[2]
    double t4min, t4max;                 // → cfg.jointLimits[3]
    double t5min, t5max;                 // → cfg.jointLimits[4]
    double t6min, t6max;                 // → cfg.jointLimits[5]
    ConfigureCheck::RobotDH robotDh;
    RobotWorkSpace robotWorkSpace;
    double baseHeight;
    LinkageAxisJointLimitCheck* linkageAxisJointLimitCheck;
};
```

前端通过 `std::map<std::string, RobotTechParameters>` 传入，key 即 `robotName`。

---

## 14. UI 界面现状

当前 UI 为最小化占位实现：

```
主窗口 (600×400)
├─ 菜单栏: 存在但无菜单项
├─ 工具栏: 存在但无按钮
├─ 中央控件: 空 QWidget
└─ 状态栏: 存在但无内容
```

UI 目前仅作为应用入口，通过信号触发规划算法，尚未实现用户交互控件。核心价值在算法层。

---

## 15. 已知 TODO 项

| 文件 | 位置 | 说明 | 优先级 |
|------|------|------|--------|
| RobotConfig.cpp | ~70 行 | YAML 解析器实现不完整（仅支持简单 key-value） | 低 |
| RobotConfig.cpp | ~106 行 | 部分 key 的解析和赋值未实现 | 低 |
| RobotConfig.cpp | ~115 行 | 数组/矩阵类型的 YAML 解析未实现 | 低 |
| JBIExporter.cpp | ~74 行 | 跨平台目录创建（Windows _mkdir vs C++17 filesystem） | 低 |

> 以上 TODO 均为非关键功能，不影响基本规划流程（配置可通过默认值 + 前端参数覆盖）。

---

## 16. 关键算法参数

| 参数 | 值 | 位置 | 说明 |
|------|-----|------|------|
| 滑动窗口尺寸 | [60,60,60] mm | RobotConfig | 表面离散化 |
| 滑动窗口步长 | [60,60,60] mm | RobotConfig | 扫描分辨率 |
| 面扫安全高度 | 0 mm | RobotConfig | 抬起间隙 |
| 测点安全高度 | 250 mm | RobotConfig | 测量间隙 |
| 碰撞安全距离 | 50 mm | RobotConfig | 检测裕量 |
| 关节连续性阈值 | [120°×6] | RobotConfig | 最大角变化 |
| BiRRT 步长 | 0.17 rad | BiRRTPlanner | 扩展粒度 |
| BiRRT 目标偏置 | 10% | BiRRTPlanner | 方向概率 |
| BiRRT 方向偏置 | 0.7 | BiRRTPlanner | 朝目标加权 |
| BiRRT 最大迭代 | 5000 | BiRRTPlanner | 搜索上限 |
| BiRRT 连接阈值 | 0.87 rad (~50°) | BiRRTPlanner | 两树连接距离 |
| BiRRT 树大小限制 | 10000 节点 | BiRRTPlanner | 单树上限 |
| 路径平滑尝试 | 100 次 | BiRRTPlanner | 快捷尝试 |
| 自碰撞角度阈值 | 27° | ViewPlanner | 连杆夹角下限 |
| IK 选优权重 | J1×2.5, J2J3×1.0, J4J5J6×3.0 | ViewPlanner | 解选择偏好 |
| 外轴插入阈值 | 转台10°, 地轨50mm | JBIExporter | 过渡插入 |

---

## 17. 近期重构记录

### 2026-03-25：替换 FK/IK/碰撞为直接调用三方接口

**背景：** 原代码通过 `RobotKinematics` 和 `CollisionChecker` 两个包装类间接调用三方接口，与 BlueRay 项目风格不一致，且增加了不必要的中间层。

**修改内容：**

| 文件 | 操作 | 说明 |
|------|------|------|
| `RobotKinematics.h/.cpp` | 移出编译 | FK/IK 包装类，逻辑已内联到 ViewPlanner |
| `CollisionChecker.h/.cpp` | 移出编译 | 碰撞检测包装类，逻辑已内联到 ViewPlanner/BiRRT |
| `ViewPlanner.h/.cpp` | 修改 | 构造函数新增三个函数指针参数 |
| `BiRRTPlanner.h/.cpp` | 修改 | 构造函数新增 collision_test 参数 |
| `robot_planner_entry.cpp` | 修改 | 直接声明三个函数指针 |
| `QtWidgetsApplication2.vcxproj` | 修改 | 删除弃用文件的编译条目 |

**架构对比：**
```
重构前: ViewPlanner → RobotKinematics → KinematicsApi → 外部库
重构后: ViewPlanner → 函数指针 → 外部库（直接调用）
```

**编译结果：** Release x64，0 错误，0 警告。

---

## 18. 总结

**QtWidgetsApplication2** 是一个面向工业应用的机器人离线视觉规划系统，核心功能包括：

1. **点云处理** — PCA 分析（手写 Jacobi 特征分解）+ 滑动窗口，生成表面扫描轨迹
2. **运动规划** — BiRRT 双向搜索，生成无碰撞关节空间路径
3. **运动学集成** — FK/IK 函数指针注入，验证可达性
4. **碰撞检测** — 多级检测策略（限位 → IK可行性 → 环境碰撞 → 自碰撞）
5. **程序导出** — Yaskawa JBI 格式，8 轴配置，可直接用于产线执行

系统基于 C++20 构建，集成 Qt 5.14.2 / OpenCASCADE / OSG 等成熟库，Release 模式编译通过（0 错误 0 警告）。数学运算全部手写实现，不依赖 Eigen。当前 UI 为占位实现，核心价值在算法层。
