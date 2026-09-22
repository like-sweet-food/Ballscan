# QtWidgetsApplication2 项目逻辑文档

> 最后更新：2026-03-27
> 编译环境：VS2022 / MSVC v143 / C++20 / Release x64 / Qt 5.14.2

---

## 1. 项目总览

安川（Yaskawa）机器人离线视觉规划系统。核心能力：

1. 点云 PCA 分析 + 滑动窗口 → 生成表面扫描轨迹
2. BiRRT 双向搜索 → 生成无碰撞关节空间路径
3. FK/IK/碰撞检测 → 验证可达性（**直接调用二次开发接口，无中间封装层**）
4. 导出 Yaskawa JBI 机器人程序文件

---

## 2. 文件结构与职责

```
D:\test\QtWidgetsApplication2\
├── main.cpp                        # 应用入口（35 行），绑定函数指针，连接信号
├── robot_planner_entry.cpp         # 规划入口 runRobotPlanner()（197 行）
├── Types.h                         # 项目内数据类型定义（137 行）
├── RobotConfig.h / .cpp            # YAML 配置加载 + 全局参数（191 + 126 行）
├── ViewPlanner.h / .cpp            # 视点规划核心算法（132 + 1003 行）
├── BiRRTPlanner.h / .cpp           # 双向 RRT 关节空间路径规划（164 + 334 行）
├── JBIExporter.h / .cpp            # Yaskawa JBI 文件导出（140 + 269 行）
├── QtWidgetsApplication2.h/cpp/ui  # Qt 主窗口（占位，17 + 11 + 29 行）
└── QtWidgetsApplication2.vcxproj   # VS 项目文件（143 行）

D:\test\3part\m_include\            # 二次开发接口（只读）
├── parameters_type.h               # 核心类型 + 函数指针 typedef
├── kinematicsApi.h                 # 单例 API 包装（bind + 调用）
├── robot_robotKinematicsCollisionInterface.h  # 静态接口实现类
├── math_utils.h                    # 数学工具函数（rotm2eul_ZYX 等）
├── ConfigureCheck.h                # 构型检查
└── linkage_axis_joint_limit_check.h  # 连杆轴限位
```

---

## 3. 二次开发接口（parameters_type.h）

项目直接调用的三个函数指针类型：

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
struct FowardKinematicsReslutstd {
    std::vector<double>              position; // 末端位置 [x,y,z]
    std::vector<std::vector<double>> tcp;      // TCP 坐标系下 4×4 矩阵
    std::vector<std::vector<double>> ut;       // User-frame 下 4×4 矩阵（常用）
};

struct XYZWPR {
    double x, y, z;   // 位置，单位 mm（Base 坐标系）
    double w, p, r;   // ZYX 欧拉角，单位度（W=绕Z, P=绕Y, R=绕X）
};

enum class LinkName {
    LINK_1 = 1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6,
    TCP = 7, TOOLFRAME = 9
};

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

## 4. 启动流程

```
main.cpp
  │
  ├─ QApplication + MainWindow 创建
  │
  ├─ 绑定函数指针到 KinematicsApi 单例：
  │   collision_test            → RobotKinematicsCollisionInterface::checkCollision
  │   robot_forward_solution_fn → RobotKinematicsCollisionInterface::forwardSolution
  │   robot_inverse_solution_fn → RobotKinematicsCollisionInterface::inverseSolution
  │   cylinderCollisionTest     → RobotKinematicsCollisionInterface::cylinderCollisionTestFunction
  │   looseAngleIk              → RobotKinematicsCollisionInterface::looseAngleIkInverseSolution
  │   setSafeDistance           → RobotKinematicsCollisionInterface::setCollisionTestSafeDistance
  │
  ├─ 信号连接：MainWindow::start → runRobotPlanner("conf/robot_config.yaml")
  │
  └─ app.exec()
```

---

## 5. runRobotPlanner 入口逻辑（robot_planner_entry.cpp, 197 行）

```
runRobotPlanner(cfgPath, robots_tech_params)
  │
  ├─ §1   loadConfig(cfgPath)          ← YAML 加载基础配置
  ├─ §1.5 从 robots_tech_params 取参数：
  │        map.key          → cfg.robotName
  │        rtp.t1min~t6max  → cfg.jointLimits[0..5]
  │        rtp.initial_angle → cfg.robotHome / cfg.lastView
  │
  ├─ §2   获取三个函数指针（与 main.cpp bind 相同实现）
  │        robot_forward_solution_fn
  │        robot_inverse_solution_fn
  │        collision_test
  │        + setCollisionTestSafeDistance(cfg.collisionTolerance)
  │
  ├─ §3   加载输入数据
  │        partPoints     ← part_points.txt（XYZ 文本）
  │        fixturePoints  ← fixture.txt
  │        measurePoints  ← mp_info.xlsx（XYZ + NxNyNz）
  │
  ├─ §4   创建规划器
  │        BiRRTPlanner birrt(cfg, collision_test)
  │        ViewPlanner planner(cfg, birrt, fk, ik, collision)
  │
  ├─ §5   阶段一：planner.planSurfaceViews(partPoints)  → surfaceResult
  ├─ §6   阶段二：planner.planMeasureViews(...)          → measResult
  │
  └─ §7   阶段三：JBIExporter 导出
           exporter.exportToJBI(surfaceResult) → _surface.JBI
           exporter.exportToJBI(measResult)    → _meas.JBI
```

---

## 6. ViewPlanner 核心逻辑（ViewPlanner.h / .cpp, 132 + 1003 行）

### 6.1 构造

```cpp
ViewPlanner(RobotConfig& cfg, BiRRTPlanner& biRrtPlanner,
            RobotFKFunction       robot_forward_solution_fn,
            RobotIKFunction       robot_inverse_solution_fn,
            CollisionTestFunction collision_test);
```

三个函数指针作为成员变量存储，在所有调用点**直接使用**，无中间封装函数。

### 6.2 planSurfaceViews — 面扫描视点规划

```
输入：partPoints（工件点云）
输出：PlanResult（有序路径点列表）

流程：
  1. computeMainNormal(partPoints)
     │  PCA：计算质心 → 3×3 协方差矩阵 → Jacobi 特征分解（手写，无 Eigen）
     │  → 最小特征值对应列
     └→ mainNormal（点云主法向量）

  2. transformToLocalFrame(partPoints, mainNormal)
     │  构建局部坐标系（Z 对齐法向量）→ 点云变换到局部坐标系
     └→ localPoints + local2Global 变换矩阵

  3. buildSlidingWindowGrid(localPoints, min, max)
     │  窗口尺寸 [60,60,60] mm，步长 [60,60,60] mm
     │  蛇形扫描（奇数行 Y 反向）
     └→ gridCells[]（每个含 centerPos, centerNormal, xAxis）

  4. 逐栅格生成视点
     FOR each gridCell:
     │  a. 构建局部 TCP 位姿（buildTcpPoseLocal）
     │  b. 变换到全局：globalT = local2Global × localT
     │  c. 可达性检测：!checkViewPointCollision(globalT, ...) → 直接调用 IK + collision
     │  d. 不可达时：adjustViewPose() 沿 Z 轴偏移重试（5mm/步，最多 5~10 步）
     └→ initViewPoints[]

  5. 逐段直线预测（predictLinePath）
     FOR each pair(i, i+1):
     │  a. 直接调用 collision_test 检测路径碰撞
     │  b. 无碰撞 → 线性插值
     │  c. 有碰撞 → BiRRT 规划无碰路径
     └→ lineProcessResult[]

  6. HOME → 第一视点的 MoveJ 规划（planMoveJ）
     │  直接调用 collision_test 检测
     │  有碰撞 → BiRRT 规划
     └→ 插入 homeNode (+ midNode)

  7. processLineResult → 输出 N×8 矩阵 [J1~J6, turnTable, slidePos]
```

### 6.3 planMeasureViews — 测量点视点规划

```
输入：surfaceResult, partPoints, measurePoints, mainNormal
输出：PlanResult

流程：
  FOR each measurePoint:
  │  1. 构建 TCP 位姿
  │     │  Z 轴 = 测量点法向量 (i,j,k)
  │     │  X 轴 = computeOptimalXAxis(Z, i2j2k2)
  │     │  位置 = 测量点坐标 + measSafeHeight × Z
  │     └→ tcpPose (4×4)
  │
  │  2. 遍历转台角度 rotateRange = [0°, 90°, 180°, 270°]
  │     FOR each rotAngle:
  │     │  a. rotatedTcp = Rz(rotAngle) × tcpPose
  │     │  b. checkViewPointCollision(rotatedTcp, ...) → 直接调用 IK + collision
  │     │  c. 首个无碰撞解 → bestVp，记录 partRotateAngle
  │     └→ break
  │
  │  3. predictLinePath(上一视点, bestVp) → BiRRT 避障
  └→ measViewPoints[]

  processLineResult → 输出
```

### 6.4 checkViewPointCollision — 视点碰撞检测算法（业务逻辑）

这是唯一保留的复合算法函数，内部**直接**调用三个接口：

```
输入：tcpPose（用户坐标系 4×4），lastViews，needJointCons
输出：outJoints（最优关节解），返回 true=碰撞/不可达

步骤：
  1. 坐标变换
     T_base = inv(T_user) × tcpPose × inv(T_he)
     提取 ZYX 欧拉角 → XYZWPR

  2. 直接调用 robot_inverse_solution_fn(robotName, xyzwpr) → rawSols

  3. 关节限位筛选
     FOR each sol: Yaskawa 轴变换(J2=90-J2, J3=J3-90, J5=-J5) → 检查 cfg.jointLimits

  4. 碰撞检测 + 自碰撞
     FOR each valid sol:
       a. 直接调用 collision_test(robotName, jv, jv) → 单姿态检测
       b. 直接调用 robot_forward_solution_fn 获取 Link2/3/4 位姿
          → 计算连杆夹角，< 27° 视为自碰撞

  5. 关节连续性约束（needJointCons=true 时）
     过滤：|sol[i] - prevQ[i]| > anglesThresholds[i] 的解

  6. 选最优解（加权欧氏距离）
     d = 2.5×ΔJ1² + 1.0×(ΔJ2²+ΔJ3²) + 3.0×(ΔJ4²+ΔJ5²+ΔJ6²)
```

### 6.5 辅助方法

| 方法 | 作用 |
|------|------|
| `computeMainNormal` | PCA 求点云主法向量（Jacobi 特征分解，无 Eigen） |
| `transformToLocalFrame` | 点云变换到局部坐标系 |
| `buildSlidingWindowGrid` | 滑动窗口分组，蛇形排列 |
| `computeOptimalXAxis` | 投影去除 Z 分量，求最优 X 轴 |
| `buildTcpPoseLocal` | 从位置 + 法向量 + X 轴构建 TCP 4×4 矩阵 |
| `buildNextBestView` | 从上一视点姿态 + 新法向量构建下一视点 |
| `adjustViewPose` | 沿 Z 轴逐步偏移（5mm/步，最多 5~10 步）重试可达性 |
| `predictLinePath` | 两视点间直线插值 / BiRRT 避障 |
| `processLineResult` | 过滤有效视点，输出 N×8 矩阵 |
| `planMoveJ` | 关节运动规划（直接碰撞检测 / BiRRT） |

### 6.6 静态数学工具

| 方法 | 作用 |
|------|------|
| `normalize3(v)` | 三维向量归一化 |
| `dot3(a, b)` | 点积 |
| `cross3(a, b)` | 叉积 |
| `multiplyTransforms(A, B)` | 4×4 矩阵乘法 |
| `invertTransform(T)` | 齐次变换矩阵求逆（利用正交性） |

---

## 7. BiRRTPlanner 逻辑（BiRRTPlanner.h / .cpp, 164 + 334 行）

### 7.1 算法参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `m_stepRad` | 0.17 rad | 单步扩展步长 |
| `m_goalBias` | 0.1 | 10% 概率直接采样目标点 |
| `m_goalDirectionBias` | 0.7 | 偏向目标方向权重 |
| `m_maxIterations` | 5000 | 最大迭代次数 |
| `m_connectThreshRad` | 0.87 rad (~50°) | 两树连接阈值 |
| 树大小限制 | 10000 节点 | 单树最大节点数 |

### 7.2 plan() 主流程

```
输入：startDeg, goalDeg（度）
输出：路径点列表（度），空=失败

  1. 转弧度，初始化 tree1(start) 和 tree2(goal)

  2. 主循环（最多 5000 次）：
     a. expandTree(tree1 → tree2)
        - 采样：10% goal bias，90% 随机
        - 找最近节点 → 加权方向扩展
          新方向 = normalize(sample-near)*step + normalize(goal-near)*step*0.7
        - 关节限位截断
        - 直接调用 collision_test 检测碰撞
        - 加入树，检测是否与另一树连接（距离 < 0.87 rad）
     b. expandTree(tree2 → tree1)
     c. 树节点 > 10000 → 停止该树

  3. 连接成功 → extractPath 回溯两树 → 合并（弧度转度）
```

### 7.3 smoothPath() 路径平滑

```
最多 100 次随机快捷尝试：
  随机选 i, j（i < j-1）
  直接调用 collision_test(robotName, path[i], path[j])
  无碰撞 → 删除 i+1 ~ j-1 中间节点
```

---

## 8. JBIExporter 逻辑（JBIExporter.h / .cpp, 140 + 269 行）

### 8.1 Yaskawa 轴变换

```
标准轴 → Yaskawa PDPS 坐标系：
  S = J1           （直接映射）
  L = 90° - J2     （L 轴）
  U = J3 - 90°     （U 轴）
  R = J4           （直接映射）
  B = -J5          （B 轴取反）
  T = J6           （直接映射）
```

### 8.2 脉冲比

| 轴 | 脉冲比 | 说明 |
|----|--------|------|
| S (J1) | 1413.522 | |
| L (J2) | -1604.267 | |
| U (J3) | 1994.364 | |
| R (J4) | -930.91 | |
| B (J5) | 986.08 | |
| T (J6) | -536.604 | |
| Z (转台) | 11.378 | |
| E (滑轨) | 130.383 | |

### 8.3 exportToJBI 流程

```
PlanResult → N×8 矩阵 [J1~J6, turnTable, slidePos]
  │
  ├─ insertExtAxisSteps()
  │    外轴变化 > 阈值时插入过渡步（转台>10°/地轨>50mm）
  │    过渡步中机器人关节角保持不变，仅外轴插值
  │
  ├─ 轴变换 convertAxes()
  ├─ jointsToPulses() → 8 轴脉冲值
  │
  ├─ 写文件：
  │    /JOB 头
  │    //POS 位置数据区（C00000 脉冲值行）
  │    //INST 程序体（MOVJ/MOVL 指令）
  │    END
  └→ .JBI 文件
```

---

## 9. RobotConfig 配置（RobotConfig.h / .cpp, 191 + 126 行）

### 9.1 参数来源

| 参数 | 来源 | 说明 |
|------|------|------|
| `robotName` | `robots_tech_params` map key | 前端传入（最高优先级） |
| `jointLimits[0..5]` | `RobotTechParameters.t1min~t6max` | 前端传入 |
| `robotHome` | `RobotTechParameters.initial_angle` | 前端传入 |
| `T_user` | RobotConfig 默认值 / YAML | 基座→用户坐标系 |
| `T_he` | RobotConfig 默认值 / YAML | TCP→相机手眼矩阵 |
| `collisionTolerance` | 默认 50mm / YAML | 碰撞安全距离 |
| `anglesThresholds` | 默认 [120°×6] / YAML | 相邻视点关节变化阈值 |
| `windowSize / stepSize` | 默认 [60,60,60] / YAML | 滑动窗口参数 |
| `safeHeight` | 默认 0mm / YAML | 面扫安全抬起高度 |
| `measSafeHeight` | 默认 250mm / YAML | 测点安全高度 |
| `measTableAxisXYZ` | 默认值 / YAML | 转台旋转中心 |
| `rotateRange` | [0,90,180,270] / YAML | 转台旋转范围 |
| `slidePos` | 默认 -3500mm / YAML | 地轨位置 |
| `randomSeed` | 默认 666 / YAML | BiRRT 随机种子 |

### 9.2 loadConfig 流程

```
loadConfig(cfgPath)
  ├─ defaultConfig() → 结构体成员默认值
  ├─ 打开 YAML 文件，逐行解析 key: value
  │   （注意：数组/矩阵解析尚未完整实现，见 TODO 项）
  ├─ fillFilePaths(cfg) → 自动填充 partPointsPath 等 8 条文件路径
  └→ RobotConfig

runRobotPlanner 中覆盖：
  robots_tech_params.begin()
    .first  → cfg.robotName
    .second → cfg.jointLimits, cfg.robotHome, cfg.lastView
```

---

## 10. 坐标系变换

```
基座坐标系 (Base)  ←─ T_user ─→  用户坐标系 (User)
                   ←─ T_he  ─→  工具坐标系 (Tool/Camera)

视点规划在用户坐标系下进行：
  1. 视点 tcpPose（用户坐标系）
  2. → T_base = inv(T_user) × tcpPose × inv(T_he)  → 基座坐标系
  3. → 提取 ZYX 欧拉角 → XYZWPR
  4. → 调用 IK(robotName, xyzwpr)

FK 返回值：
  robot_forward_solution_fn 返回 FowardKinematicsReslutstd：
    .position  — 参考坐标系下 xyz
    .tcp       — Base 坐标系下矩阵
    .ut        — User-frame 下 4×4 矩阵（项目使用此字段）
```

---

## 11. 数据类型（Types.h, 137 行）

```cpp
namespace robot_planner {
    using Matrix4d   = std::vector<std::vector<double>>;  // 4×4 行优先
    using Vec3d      = std::array<double, 3>;
    using JointVec6  = std::array<double, 6>;
    using PointCloud = std::vector<Vec3d>;

    struct ViewPoint {
        Matrix4d  globalT;              // 用户坐标系 TCP 4×4
        JointVec6 joints;               // 关节角（度）
        bool      isImportant;          // 关键点（行首/行尾）
        bool      isPathJoint;          // BiRRT 中间避障点
        int       moveType;             // 0=MoveJ, 1=MoveL
        double    partRotateAngle;      // 转台角度
        Matrix4d  partTransform;        // 转台变换矩阵
    };

    struct RRTNode {
        JointVec6 theta, thetaPrev;     // 当前/父节点关节角（弧度）
        double dist; int indPrev; bool lastFlag;
    };

    using RRTTree = std::vector<RRTNode>;

    struct PlanResult {
        std::vector<ViewPoint> waypoints;
        bool success; std::string errorMsg;
    };

    struct GridCell {
        Vec3d centerPos, centerNormal, xAxis;
        bool isImportant;
    };

    struct BVHScene { ... };            // 碰撞场景（PQP 模型）
}
```

---

## 12. 接口调用一览

所有 FK/IK/碰撞检测均为**直接调用函数指针**，无中间封装：

### 正运动学（robot_forward_solution_fn）

| 文件:行 | 调用场景 | LinkName |
|---------|---------|----------|
| ViewPlanner.cpp:~126 | HOME 视点 globalT | 7 (TCP) |
| ViewPlanner.cpp:~208 | HOME→首视点 homeNode | 7 |
| ViewPlanner.cpp:~225 | BiRRT 中间节点 midNode | 7 |
| ViewPlanner.cpp:~292 | 测点 HOME 视点 | 7 |
| ViewPlanner.cpp:~727 | BiRRT 路径中间点 | 7 |
| ViewPlanner.cpp:~882 | 自碰撞检测 Link2 位姿 | 2 |
| ViewPlanner.cpp:~883 | 自碰撞检测 Link3 位姿 | 3 |
| ViewPlanner.cpp:~884 | 自碰撞检测 Link4 位姿 | 4 |

### 逆运动学（robot_inverse_solution_fn）

| 文件:行 | 调用场景 |
|---------|---------|
| ViewPlanner.cpp:~836 | checkViewPointCollision 内部求 IK 多解 |

### 碰撞检测（collision_test）

| 文件:行 | 调用场景 | 类型 |
|---------|---------|------|
| ViewPlanner.cpp:~693 | predictLinePath 路径碰撞 | 路径检测 |
| ViewPlanner.cpp:~776 | planMoveJ 路径碰撞 | 路径检测 |
| ViewPlanner.cpp:~877 | checkViewPointCollision 单姿态 | 单点检测(start==end) |
| BiRRTPlanner.cpp:~157 | smoothPath 快捷路径 | 路径检测 |
| BiRRTPlanner.cpp:~220 | expandTree 扩展步碰撞 | 路径检测 |

> 注：行号为近似值，因代码可能有微调。

---

## 13. 关键算法参数

| 参数 | 值 | 位置 |
|------|-----|------|
| 滑动窗口尺寸 | [60,60,60] mm | RobotConfig |
| 滑动窗口步长 | [60,60,60] mm | RobotConfig |
| 面扫安全高度 | 0 mm | RobotConfig |
| 测点安全高度 | 250 mm | RobotConfig |
| 碰撞安全距离 | 50 mm | RobotConfig |
| 关节连续性阈值 | [120°×6] | RobotConfig |
| BiRRT 步长 | 0.17 rad | BiRRTPlanner |
| BiRRT 目标偏置 | 10% | BiRRTPlanner |
| BiRRT 方向偏置 | 0.7 | BiRRTPlanner |
| BiRRT 最大迭代 | 5000 | BiRRTPlanner |
| BiRRT 连接阈值 | 0.87 rad (~50°) | BiRRTPlanner |
| BiRRT 树大小限制 | 10000 节点 | BiRRTPlanner |
| 路径平滑尝试 | 100 次 | BiRRTPlanner |
| 自碰撞角度阈值 | 27° | ViewPlanner |
| IK 选优权重 | J1×2.5, J2J3×1.0, J4J5J6×3.0 | ViewPlanner |
| 外轴插入阈值 | 转台10°, 地轨50mm | JBIExporter |
