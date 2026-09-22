
#include <QtWidgets/QApplication>
#include <QtConcurrent/QtConcurrent>
#include <ui_mainWindow.h>
#include <robot_robotKinematicsCollisionInterface.h>
#include <ConfigureCheck.h>
#include <parameters_type.h>
#include <kinematicsApi.h>
#include <iostream>
#include <map>
#include <set>
#include "PointCloudGenerator.h"
#include "MeasurePointUtils.h"
#include "SurfacePointGen.h"
#include "BallScan.h"
#include "Types.h"
#include "RobotReachability.h"
#include "JBIExporter.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <array>
#include <cctype>
#include <iomanip>
#include <limits>
#include <utility>
#include "RemoveTurntableEC.h"
// 用于 aaa → pathPoints 转换中的 rotm2eul_XYZ（FK 旋转矩阵 → X-Y-Z 欧拉角）。（新增）
#include "math_utils.h"

// （新增）判断一个测点是否为面点。
// 前端测点文件中面点的类型串包含 "Surface"/"surface"，与几何生成面点的类型
// "SurfacePoint" 规则一致。这里直接用子串匹配，最简单、且不需额外头文件。
static inline bool isSurfaceMP(const MeasurePoint& mp)
{
	const std::string& t = mp.vct_vectro_type;
	return t.find("Surface") != std::string::npos
		|| t.find("surface") != std::string::npos;
}

// （新增）判断一个测点是否为切边点。
// 前端测点文件中切边点的类型串包含 "Trim"/"trim"（与 BlueRayMeasurePoint.h 的
// is_trim_type 规则一致）。同样用最简单的子串匹配。
static inline bool isTrimMP(const MeasurePoint& mp)
{
	const std::string& t = mp.vct_vectro_type;
	return t.find("Trim") != std::string::npos
		|| t.find("trim") != std::string::npos;
}

// （新增）判断一个测点是否为 Standard_circle 类型。
// 注意：必须在 isStandardMP 之前判断，因为 "Standard_circle" 也包含子串 "Standard"。
static inline bool isStandardCircleMP(const MeasurePoint& mp)
{
	const std::string& t = mp.vct_vectro_type;
	return t.find("Standard_circle") != std::string::npos
		|| t.find("standard_circle") != std::string::npos;
}

// （新增）判断一个测点是否为 Standard 类型（且排除 Standard_circle）。
static inline bool isStandardMP(const MeasurePoint& mp)
{
	if (isStandardCircleMP(mp)) return false;          // 先排除圆类，避免误判
	const std::string& t = mp.vct_vectro_type;
	return t.find("Standard") != std::string::npos
		|| t.find("standard") != std::string::npos;
}

// （新增）判断测点"第一矢量"(i,j,k) 与世界 Z 轴 (0,0,1) 的夹角是否过小。
// 返回 true 表示夹角 < 阈值（默认 10°），即该面点需要被剔除。
// 点积 (i,j,k)·(0,0,1) = k，故 cos(夹角) = k / |(i,j,k)|，无需算反余弦。
static inline bool isVectorNearZ(const MeasurePoint& mp, double thresholdDeg = 10.0)
{
	const double i = mp.i, j = mp.j, k = mp.k;
	const double norm = std::sqrt(i * i + j * j + k * k);
	if (norm < 1e-9) return false;                 // 零矢量无法判断，保留不删

	double cosAng = k / norm;                       // 与 +Z 的夹角余弦
	if (cosAng > 1.0)  cosAng = 1.0;
	if (cosAng < -1.0) cosAng = -1.0;

	const double cosThresh = std::cos(thresholdDeg * 3.14159265358979323846 / 180.0);
	return cosAng > cosThresh;                       // 夹角 < 阈值 → 删除
}

// （新增）面点自查：在一批面点内两两比对，若两点"位置足够近 + 第一矢量近似垂直"，
// 视为一条棱边/直角拐角，生成一个"斜面点"追加到容器尾部：
//   · 位置  = 两点中点；
//   · 第一矢量 = 两单位法向之和（即角平分线方向），归一化；
//   · 第二矢量 = 方案B——取父点 xAxis 减去其在新法向上的投影后归一化，保证与新法向正交；
//   · 类型串保持 "Surface"（下游 BallScan 用 == "Surface" 选安全高度），
//     "斜点"身份记在 name 字段（SurfaceDiagonal_N）。
// 只在传入时已有的原始点之间配对（不与本函数新增的斜点再配对）。
//   distThreshMm : 位置欧氏距离阈值（默认 10mm）
//   angleTolDeg  : 与 90° 的允许偏差（默认 ±15°）
static void appendDiagonalSurfaceMPs(std::vector<MeasurePoint>& mps,
	double distThreshMm = 10.0,
	double angleTolDeg = 35.0)
{
	const size_t n = mps.size();          // 锁定原始点数量，新增点不参与再配对
	if (n < 2) return;

	const double distSq = distThreshMm * distThreshMm;
	// 90°±tol → 单位法向点积落在 [-sin(tol), sin(tol)]，即 |dot| <= sin(tol)
	const double dotThresh = std::sin(angleTolDeg * 3.14159265358979323846 / 180.0);

	// 预计算每个点第一矢量的模长，零矢量点直接排除
	std::vector<double> norms(n);
	for (size_t i = 0; i < n; ++i)
		norms[i] = std::sqrt(mps[i].i * mps[i].i + mps[i].j * mps[i].j + mps[i].k * mps[i].k);

	std::vector<MeasurePoint> diagonals;
	std::set<std::pair<size_t, size_t>> emitted;   // 已生成的配对，避免重复
	int diagIdx = 0;

	for (size_t a = 0; a < n; ++a)
	{
		if (norms[a] < 1e-9) continue;    // 零矢量无法定向，跳过
		const MeasurePoint& p = mps[a];

		// 关键：只为 a 找"最近的合格伙伴"，每个点最多贡献一个斜点
		//  → 斜点总数 ≤ n，彻底杜绝密集棱边处 O(n²) 的爆炸
		bool hasBest = false;
		size_t best = 0;
		double bestDistSq = distSq;        // 候选必须严格落在距离阈值内
		for (size_t b = 0; b < n; ++b)
		{
			if (b == a || norms[b] < 1e-9) continue;
			const MeasurePoint& q = mps[b];

			// 1) 位置：欧氏距离 ≤ 阈值，且取更近的
			const double dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
			const double d2 = dx * dx + dy * dy + dz * dz;
			if (d2 > bestDistSq) continue;

			// 2) 第一矢量夹角 ≈ 90°（用单位法向点积判断）
			const double dot = (p.i * q.i + p.j * q.j + p.k * q.k) / (norms[a] * norms[b]);
			if (std::fabs(dot) > dotThresh) continue;

			hasBest = true; best = b; bestDistSq = d2;   // 记录最近的合格伙伴
		}
		if (!hasBest) continue;

		// 去重：同一对 (a,best) 只生成一次（best 反向也可能选中 a）
		const std::pair<size_t, size_t> key =
			(a < best) ? std::make_pair(a, best) : std::make_pair(best, a);
		if (!emitted.insert(key).second) continue;

		const MeasurePoint& q = mps[best];
		const double pn = norms[a], qn = norms[best];

		// —— 生成斜面点 ——
		MeasurePoint d;

		// 位置：两点中点
		d.x = 0.5 * (p.x + q.x);
		d.y = 0.5 * (p.y + q.y);
		d.z = 0.5 * (p.z + q.z);

		// 第一矢量：两单位法向之和（角平分线），归一化
		double ni = p.i / pn + q.i / qn;
		double nj = p.j / pn + q.j / qn;
		double nk = p.k / pn + q.k / qn;
		double nn = std::sqrt(ni * ni + nj * nj + nk * nk);
		if (nn < 1e-9) continue;           // 近 90° 不会退化，保险起见
		ni /= nn; nj /= nn; nk /= nn;
		d.i = ni; d.j = nj; d.k = nk;

		// 第二矢量（方案B）：父点 p 的 xAxis 对新法向做正交化后归一化
		double xi = p.i2, xj = p.j2, xk = p.k2;
		double proj = xi * ni + xj * nj + xk * nk;
		xi -= proj * ni; xj -= proj * nj; xk -= proj * nk;
		double xn = std::sqrt(xi * xi + xj * xj + xk * xk);
		if (xn < 1e-9)
		{
			// p 的 xAxis 与新法向共线，退化：改用 q 的 xAxis 再正交化
			xi = q.i2; xj = q.j2; xk = q.k2;
			proj = xi * ni + xj * nj + xk * nk;
			xi -= proj * ni; xj -= proj * nj; xk -= proj * nk;
			xn = std::sqrt(xi * xi + xj * xj + xk * xk);
			if (xn < 1e-9) continue;       // 仍退化则放弃该斜点
		}
		d.i2 = xi / xn; d.j2 = xj / xn; d.k2 = xk / xn;

		d.i_old = d.i;	d.j_old = d.j; d.k_old = d.k;
		d.i2_old = d.i2;	d.j2_old = d.j2; d.k2_old = d.k2;

		d.vct_vectro_type = "SurfacePoint";                 // 与普通面点同类，下游安全高度一致
		d.name = "SurfaceDiagonal_" + std::to_string(++diagIdx);
		d.point_number = 0;

		diagonals.push_back(d);
	}

	std::cout << "[appendDiagonalSurfaceMPs] 原始面点 " << n
		<< " 个，自查生成斜面点 " << diagonals.size() << " 个" << std::endl;
	if (!diagonals.empty())
		mps.insert(mps.end(), diagonals.begin(), diagonals.end());
}

static std::vector<robot_planner::ViewPoint>
insertMovlToMovjTransitions(const std::vector<robot_planner::ViewPoint>& path)
{
	std::vector<robot_planner::ViewPoint> result;
	result.reserve(path.size() * 2);
	for (size_t i = 0; i < path.size(); ++i) {
		result.push_back(path[i]);
		if (i + 1 < path.size() &&
			path[i].moveType == 1 &&
			path[i + 1].moveType == 0)
		{
			robot_planner::ViewPoint trans;
			trans.joints = path[i].joints;
			trans.globalT = path[i].globalT;
			trans.partRotateAngle = path[i].partRotateAngle;
			trans.partTransform = path[i].partTransform;
			trans.moveType = 0;
			trans.isImportant = true;
			trans.isPathJoint = false;
			result.push_back(trans);
		}
	}
	return result;
}

static bool splitCsvLinePreservingEmptyFields(
	const std::string& line,
	std::vector<std::string>& fields)
{
	fields.clear();
	std::string field;
	bool inQuotes = false;

	for (size_t i = 0; i < line.size(); ++i) {
		const char ch = line[i];
		if (ch == '"') {
			if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
				field.push_back('"');
				++i;
			} else {
				inQuotes = !inQuotes;
			}
		} else if (ch == ',' && !inQuotes) {
			fields.push_back(field);
			field.clear();
		} else {
			field.push_back(ch);
		}
	}

	if (inQuotes) {
		return false;
	}
	fields.push_back(field);
	return true;
}

static bool parseFiniteDouble(const std::string& text, double& value)
{
	try {
		size_t parsed = 0;
		value = std::stod(text, &parsed);
		while (parsed < text.size()
			&& std::isspace(static_cast<unsigned char>(text[parsed]))) {
			++parsed;
		}
		return parsed == text.size() && std::isfinite(value);
	} catch (const std::exception&) {
		return false;
	}
}

static bool parseInteger(const std::string& text, int& value)
{
	try {
		size_t parsed = 0;
		const long long parsedValue = std::stoll(text, &parsed);
		while (parsed < text.size()
			&& std::isspace(static_cast<unsigned char>(text[parsed]))) {
			++parsed;
		}
		if (parsed != text.size()
			|| parsedValue < std::numeric_limits<int>::min()
			|| parsedValue > std::numeric_limits<int>::max()) {
			return false;
		}
		value = static_cast<int>(parsedValue);
		return true;
	} catch (const std::exception&) {
		return false;
	}
}

static bool loadPathpointsCsv(
	const std::string& csvPath,
	const std::string& robotName,
	double slidePos,
	std::map<std::string, std::vector<MeasurePointPoseSet>>& output)
{
	if (!std::isfinite(slidePos)) {
		std::cerr << "[pathpoints_csv] cfg.slidePos is not finite." << std::endl;
		return false;
	}

	std::ifstream file(csvPath, std::ios::in | std::ios::binary);
	if (!file.is_open()) {
		std::cerr << "[pathpoints_csv] Cannot open CSV read-only: "
			<< csvPath << std::endl;
		return false;
	}

	std::string line;
	if (!std::getline(file, line)) {
		std::cerr << "[pathpoints_csv] CSV is empty: " << csvPath << std::endl;
		return false;
	}
	if (!line.empty() && line.back() == '\r') {
		line.pop_back();
	}

	std::vector<std::string> header;
	if (!splitCsvLinePreservingEmptyFields(line, header)) {
		std::cerr << "[pathpoints_csv] Invalid quoted CSV header." << std::endl;
		return false;
	}
	if (!header.empty() && header[0].size() >= 3
		&& static_cast<unsigned char>(header[0][0]) == 0xEF
		&& static_cast<unsigned char>(header[0][1]) == 0xBB
		&& static_cast<unsigned char>(header[0][2]) == 0xBF) {
		header[0].erase(0, 3);
	}

	static const std::array<const char*, 34> requiredColumns = {
		"point_number", "name", "vct_vector_type", "is_measure_point",
		"is_accessible", "move_type", "sample_idx", "t_s", "is_anchor",
		"x_mm", "y_mm", "z_mm", "i_deg", "j_deg", "k_deg",
		"i2", "j2", "k2", "r11", "r12", "r13", "r21", "r22",
		"r23", "r31", "r32", "r33", "q1_deg", "q2_deg", "q3_deg",
		"q4_deg", "q5_deg", "q6_deg", "turn_table_deg"
	};
	if (header.size() != requiredColumns.size()) {
		std::cerr << "[pathpoints_csv] Expected 34 columns, got "
			<< header.size() << "." << std::endl;
		return false;
	}

	std::map<std::string, size_t> columnIndex;
	for (size_t i = 0; i < header.size(); ++i) {
		if (!columnIndex.emplace(header[i], i).second) {
			std::cerr << "[pathpoints_csv] Duplicate column: "
				<< header[i] << std::endl;
			return false;
		}
	}
	for (const char* column : requiredColumns) {
		if (columnIndex.find(column) == columnIndex.end()) {
			std::cerr << "[pathpoints_csv] Missing column: "
				<< column << std::endl;
			return false;
		}
	}

	std::vector<MeasurePointPoseSet> loadedPoints;
	loadedPoints.reserve(2048);
	int anchorCount = 0;
	double firstTime = 0.0;
	double lastTime = 0.0;
	double previousTime = -std::numeric_limits<double>::infinity();
	std::array<double, 3> firstXyz = {};
	std::array<double, 3> lastXyz = {};
	std::array<double, 3> xyzMin = {
		std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::infinity()
	};
	std::array<double, 3> xyzMax = {
		-std::numeric_limits<double>::infinity(),
		-std::numeric_limits<double>::infinity(),
		-std::numeric_limits<double>::infinity()
	};
	double maxOrthogonalityError = 0.0;
	double maxDeterminantDeviation = 0.0;
	size_t lineNumber = 1;

	while (std::getline(file, line)) {
		++lineNumber;
		if (!line.empty() && line.back() == '\r') {
			line.pop_back();
		}
		if (line.empty()) {
			std::cerr << "[pathpoints_csv] Empty data row at line "
				<< lineNumber << "." << std::endl;
			return false;
		}

		std::vector<std::string> fields;
		if (!splitCsvLinePreservingEmptyFields(line, fields)
			|| fields.size() != requiredColumns.size()) {
			std::cerr << "[pathpoints_csv] Invalid field count at line "
				<< lineNumber << "." << std::endl;
			return false;
		}

		auto readInt = [&](const char* name, int& value) {
			if (parseInteger(fields[columnIndex.at(name)], value)) {
				return true;
			}
			std::cerr << "[pathpoints_csv] Invalid integer in column "
				<< name << " at line " << lineNumber << "." << std::endl;
			return false;
		};
		auto readDouble = [&](const char* name, double& value) {
			if (parseFiniteDouble(fields[columnIndex.at(name)], value)) {
				return true;
			}
			std::cerr << "[pathpoints_csv] Invalid finite number in column "
				<< name << " at line " << lineNumber << "." << std::endl;
			return false;
		};

		int pointNumber = 0;
		int isMeasurePoint = 0;
		int isAccessible = 0;
		int moveType = 0;
		int sampleIndex = 0;
		int isAnchor = 0;
		double time = 0.0;
		std::array<double, 3> xyz = {};
		std::array<double, 3> euler = {};
		std::array<double, 3> secondary = {};
		std::array<double, 9> rotation = {};
		std::array<double, 6> joints = {};
		double turnTable = 0.0;

		const std::array<const char*, 3> xyzColumns = { "x_mm", "y_mm", "z_mm" };
		const std::array<const char*, 3> eulerColumns = { "i_deg", "j_deg", "k_deg" };
		const std::array<const char*, 3> secondaryColumns = { "i2", "j2", "k2" };
		const std::array<const char*, 9> rotationColumns = {
			"r11", "r12", "r13", "r21", "r22", "r23", "r31", "r32", "r33"
		};
		const std::array<const char*, 6> jointColumns = {
			"q1_deg", "q2_deg", "q3_deg", "q4_deg", "q5_deg", "q6_deg"
		};

		bool valid = readInt("point_number", pointNumber)
			&& readInt("is_measure_point", isMeasurePoint)
			&& readInt("is_accessible", isAccessible)
			&& readInt("move_type", moveType)
			&& readInt("sample_idx", sampleIndex)
			&& readDouble("t_s", time)
			&& readInt("is_anchor", isAnchor);
		for (size_t i = 0; valid && i < xyz.size(); ++i) {
			valid = readDouble(xyzColumns[i], xyz[i]);
		}
		for (size_t i = 0; valid && i < euler.size(); ++i) {
			valid = readDouble(eulerColumns[i], euler[i]);
		}
		for (size_t i = 0; valid && i < secondary.size(); ++i) {
			valid = readDouble(secondaryColumns[i], secondary[i]);
		}
		for (size_t i = 0; valid && i < rotation.size(); ++i) {
			valid = readDouble(rotationColumns[i], rotation[i]);
		}
		for (size_t i = 0; valid && i < joints.size(); ++i) {
			valid = readDouble(jointColumns[i], joints[i]);
		}
		valid = valid && readDouble("turn_table_deg", turnTable);
		if (!valid) {
			return false;
		}

		const int expectedIndex = static_cast<int>(loadedPoints.size()) + 1;
		const std::string& name = fields[columnIndex.at("name")];
		const std::string& vectorType = fields[columnIndex.at("vct_vector_type")];
		if (pointNumber != expectedIndex || sampleIndex != expectedIndex
			|| !vectorType.empty()
			|| isMeasurePoint != 0 || isAccessible != 1
			|| moveType != (expectedIndex == 1 ? 0 : 1)
			|| (isAnchor != 0 && isAnchor != 1)
			|| std::abs(secondary[0]) > 1e-12
			|| std::abs(secondary[1]) > 1e-12
			|| std::abs(secondary[2]) > 1e-12
			|| std::abs(turnTable) > 1e-12) {
			std::cerr << "[pathpoints_csv] Metadata validation failed at line "
				<< lineNumber << "." << std::endl;
			return false;
		}

		if (time < previousTime) {
			std::cerr << "[pathpoints_csv] t_s is not nondecreasing at line "
				<< lineNumber << "." << std::endl;
			return false;
		}
		previousTime = time;
		lastTime = time;
		if (expectedIndex == 1) {
			firstTime = time;
			firstXyz = xyz;
		}
		lastXyz = xyz;
		anchorCount += isAnchor;
		for (size_t axis = 0; axis < 3; ++axis) {
			xyzMin[axis] = std::min(xyzMin[axis], xyz[axis]);
			xyzMax[axis] = std::max(xyzMax[axis], xyz[axis]);
		}

		double orthogonalitySquared = 0.0;
		for (size_t columnA = 0; columnA < 3; ++columnA) {
			for (size_t columnB = 0; columnB < 3; ++columnB) {
				double dot = 0.0;
				for (size_t row = 0; row < 3; ++row) {
					dot += rotation[row * 3 + columnA]
						* rotation[row * 3 + columnB];
				}
				const double error = dot - (columnA == columnB ? 1.0 : 0.0);
				orthogonalitySquared += error * error;
			}
		}
		maxOrthogonalityError = std::max(
			maxOrthogonalityError, std::sqrt(orthogonalitySquared));
		const double determinant =
			rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
			- rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
			+ rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
		maxDeterminantDeviation = std::max(
			maxDeterminantDeviation, std::abs(determinant - 1.0));

		MeasurePointPoseSet mps;
		mps.point_number = pointNumber;
		mps.name = name;
		mps.vct_vector_type = vectorType;
		mps.isMeasurePoint = false;
		mps.is_accessible = true;
		mps.moveType = moveType;
		mps.x = xyz[0]; mps.y = xyz[1]; mps.z = xyz[2];
		mps.i = euler[0]; mps.j = euler[1]; mps.k = euler[2];
		mps.i2 = secondary[0]; mps.j2 = secondary[1]; mps.k2 = secondary[2];

		PoseConfiguration pc;
		pc.point_number = pointNumber;
		pc.point_name = name;
		pc.joints.j1 = joints[0]; pc.joints.j2 = joints[1];
		pc.joints.j3 = joints[2]; pc.joints.j4 = joints[3];
		pc.joints.j5 = joints[4]; pc.joints.j6 = joints[5];
		pc.joints.slider = slidePos;
		pc.joints.turnTable = turnTable;
		pc.xyzwpr.x = xyz[0]; pc.xyzwpr.y = xyz[1]; pc.xyzwpr.z = xyz[2];
		pc.xyzwpr.w = euler[0]; pc.xyzwpr.p = euler[1]; pc.xyzwpr.r = euler[2];
		mps.configurations.push_back(pc);
		loadedPoints.push_back(std::move(mps));
	}

	const bool dataValid = !loadedPoints.empty()
		&& std::abs(firstTime) <= 1e-12
		&& maxOrthogonalityError <= 1e-10
		&& maxDeterminantDeviation <= 1e-10;
	if (!dataValid) {
		std::cerr << "[pathpoints_csv] Generic CSV acceptance checks failed."
			<< " rows=" << loadedPoints.size()
			<< ", first_time=" << firstTime
			<< ", max_orth_error=" << maxOrthogonalityError
			<< ", max_det_deviation=" << maxDeterminantDeviation
			<< std::endl;
		return false;
	}

	std::map<std::string, std::vector<MeasurePointPoseSet>> validatedOutput;
	validatedOutput[robotName] = std::move(loadedPoints);
	output = std::move(validatedOutput);
	std::cout << "[pathpoints_csv] Loaded " << output[robotName].size()
		<< " rows, anchors=" << anchorCount
		<< ", t=[" << firstTime << ", " << lastTime << "]"
		<< ", first_xyz=[" << firstXyz[0] << ", " << firstXyz[1]
		<< ", " << firstXyz[2] << "]"
		<< ", last_xyz=[" << lastXyz[0] << ", " << lastXyz[1]
		<< ", " << lastXyz[2] << "]"
		<< ", xyz_min=[" << xyzMin[0] << ", " << xyzMin[1]
		<< ", " << xyzMin[2] << "]"
		<< ", xyz_max=[" << xyzMax[0] << ", " << xyzMax[1]
		<< ", " << xyzMax[2] << "]"
		<< ", cfg.slidePos=" << slidePos << std::endl;
	return true;
}

void execute()
{
	auto& scene = SceneDocument::instance();

	// ==================================================================
	// 参数准备：读取所有机器人技术参数（关节限位、DH、手眼矩阵等）
	// ==================================================================

	// 存储每个机器人 home 位的滑轨值（mm），供后续循环写入 cfg.slidePos。
	// 因 RobotTechParameters 无 slider 字段，使用局部 map 作为两个循环之间的桥梁。
	std::map<std::string, double> robotSliderMap;

	auto& robots_tech_params = scene.getAllRobotTechParameters();
	for (auto& robotTrch : robots_tech_params)
	{
		auto& robot = SceneDocument::instance().getRobotByName(robotTrch.first);
		auto& params = robotTrch.second;

		double x = params.handEyeMatrix.x;
		if (std::isnan(x))
		{
			QString meassge = "请添加机器人" + QString::fromStdString(robot.getRobotName()) + "手眼矩阵！！！";
			MessageManager::instance()->log(LogLevel::LOG_ERROR, meassge);
			return;
		}
		if (robot.getHomeJoints().size() == 0)
		{
			QString meassge = "请添加机器人" + QString::fromStdString(robot.getRobotName()) + "的初始关节角度！！！";
			MessageManager::instance()->log(LogLevel::LOG_ERROR, meassge);
			return;
		}

		JointAngles jointAngles = Robot::convertToJointAngles(robot.getHomeJoints());
		params.initial_angle = { jointAngles.j1, jointAngles.j2, jointAngles.j3,
								 jointAngles.j4, jointAngles.j5, jointAngles.j6 };

		// 保存滑轨初始值：JointAngles.slider 即 home 位外部轴位置（mm）。
		// NaN 表示前端未填写滑轨字段，保留原值留给后续判断处理。
		robotSliderMap[robotTrch.first] = jointAngles.slider;

		params.brand = robot.getRobotBrandName();
		params.baseHeight = robot.getBaseHight();

		// 关节角限制
		params.t1max = robot.findJointByLinkEnum(LinkName::LINK_1)->limits.value().maxPos;
		params.t1min = robot.findJointByLinkEnum(LinkName::LINK_1)->limits.value().minPos;
		params.t2max = robot.findJointByLinkEnum(LinkName::LINK_2)->limits.value().maxPos;
		params.t2min = robot.findJointByLinkEnum(LinkName::LINK_2)->limits.value().minPos;
		params.t3max = robot.findJointByLinkEnum(LinkName::LINK_3)->limits.value().maxPos;
		params.t3min = robot.findJointByLinkEnum(LinkName::LINK_3)->limits.value().minPos;
		params.t4max = robot.findJointByLinkEnum(LinkName::LINK_4)->limits.value().maxPos;
		params.t4min = robot.findJointByLinkEnum(LinkName::LINK_4)->limits.value().minPos;
		params.t5max = robot.findJointByLinkEnum(LinkName::LINK_5)->limits.value().maxPos;
		params.t5min = robot.findJointByLinkEnum(LinkName::LINK_5)->limits.value().minPos;
		params.t6max = robot.findJointByLinkEnum(LinkName::LINK_6)->limits.value().maxPos;
		params.t6min = robot.findJointByLinkEnum(LinkName::LINK_6)->limits.value().minPos;

		// DH
		params.robotDh.a1 = robot.getDHParams()[0].a;
		params.robotDh.a2 = robot.getDHParams()[1].a;
		params.robotDh.a3 = robot.getDHParams()[2].a;
		params.robotDh.a4 = robot.getDHParams()[3].a;
		params.robotDh.a5 = robot.getDHParams()[4].a;
		params.robotDh.a6 = robot.getDHParams()[5].a;

		params.robotDh.alpha1 = robot.getDHParams()[0].alpha;
		params.robotDh.alpha2 = robot.getDHParams()[1].alpha;
		params.robotDh.alpha3 = robot.getDHParams()[2].alpha;
		params.robotDh.alpha4 = robot.getDHParams()[3].alpha;
		params.robotDh.alpha5 = robot.getDHParams()[4].alpha;
		params.robotDh.alpha6 = robot.getDHParams()[5].alpha;

		params.robotDh.d1 = robot.getDHParams()[0].d;
		params.robotDh.d2 = robot.getDHParams()[1].d;
		params.robotDh.d3 = robot.getDHParams()[2].d;
		params.robotDh.d4 = robot.getDHParams()[3].d;
		params.robotDh.d5 = robot.getDHParams()[4].d;
		params.robotDh.d6 = robot.getDHParams()[5].d;

		if (robot.getRobotBrandName() == RobotBrand::FANUC || robot.getRobotBrandName() == RobotBrand::YASKAWA)
		{
			auto link2Joint = robot.findJointByLinkEnum(LinkName::LINK_2);
			if (link2Joint->m_jointLimitCheck.has_value())
			{
				params.linkageAxisJointLimitCheck = &link2Joint->m_jointLimitCheck.value();
			}
			else
			{
				MessageManager::instance()->log(LogLevel::LOG_ERROR, "LINK2的联动轴限位缺失！！！");
				return;
			}
		}
	}

	// ==================================================================
	// 零件检查
	// ==================================================================
	auto&& partNames = scene.getPartsByType(PartType::PART_WORKPIECE);
	if (partNames.empty())
	{
		MessageManager::instance()->log(LogLevel::LOG_ERROR, "被测零件缺失！！！");
		return;
	}

	auto& toturnTableAssignMeasurePoint = scene.getToturnTableAssignMeasurePoint();

	// ==================================================================
	// 面点生成：从前端提供的零件点云生成面扫覆盖点
	// ==================================================================
	std::vector<PointWithNormal> initPointCloud;
	std::vector<MeasurePoint> surfaceMPs;
	auto&& partNames2 = scene.getPartsByType(PartType::PART_WORKPIECE);
	for (auto& partName : partNames2)
	{
		auto triangleMeshData = scene.getNodeManager()->getTriangleMeshData(partName);
		auto upPointCloud = PointCloudGenerator::generateUniformPointCloudWithNormals(triangleMeshData, 0.005);
		triangleMeshData.clear();
		std::vector<PointWithNormal> downPointCloud = PointCloudGenerator::uniformDownsample(upPointCloud, 20);

		auto shapeNode = scene.getNodeManager()->findShapeNode(partName);
		if (shapeNode == nullptr)
		{
			MessageManager::instance()->log(LogLevel::LOG_ERROR,
				"零件:" + QString::fromStdString(partName) + "的显示节点缺失！！！");
			continue;
		}

		const osg::Matrix& matrix = shapeNode->getAppliedTransforms();
		gp_Trsf trsf = TransformConverter::toGpTrsf(matrix);

		for (auto& point : downPointCloud)
		{
			gp_Pnt p(point.X, point.Y, point.Z);
			p.Transform(trsf);
			point.X = p.X();
			point.Y = p.Y();
			point.Z = p.Z();

			gp_Vec n(point.I, point.J, point.K);
			n.Transform(trsf);

			if (n.Magnitude() > gp::Resolution())
			{
				auto direction = n.Normalized();
				point.I = direction.X();
				point.J = direction.Y();
				point.K = direction.Z();
			}
		}
		initPointCloud.insert(initPointCloud.end(), downPointCloud.begin(), downPointCloud.end());
	}
	surfaceMPs = generate_surface_points(initPointCloud);

	// ==================================================================
	// 全局结果容器
	// ==================================================================
	std::map<std::string, std::vector<MeasurePointPoseSet>> un_measure_point_result;
	std::map<std::string, std::vector<MeasurePointPoseSet>> measure_point_result;
	std::unordered_map<std::string, std::pair<double, double>> final_unreach_shift_angle;
	std::map<std::string, robot_planner::JointVec6> robot_last_views;

	// ==================================================================
	// 主循环：遍历每个机器人
	// ==================================================================
	for (auto& task : toturnTableAssignMeasurePoint)
	{
		auto robotNmae = task.first;
		MessageManager::instance()->log(LogLevel::LOG_INFO,
			"开始机器人：" + QString::fromStdString(robotNmae) + "姿态求解！！！");

		auto robotMeasures = scene.getMeasurePoint();
		// 前端测点文件按 vct_vectro_type 分流：
		//   含 "surface" → 面点 fileSurfaceMPs，并入阶段1；
		//   含 "trim"    → 切边点 fileTrimMPs，喂给阶段2；
		//   其余(孔hole/槽slot/球sphere/矩形rectangular)不纳入测量流程。
		const std::vector<MeasurePoint>& fileMPs = robotMeasures[robotNmae];
		std::vector<MeasurePoint> fileSurfaceMPs;   // 前端文件中的面点（先原样收集，暂不过 Z 过滤）
		std::vector<MeasurePoint> fileTrimMPs;      // 前端文件中的切边点
		std::vector<MeasurePoint> fileStandardMPs;        // 前端文件中的 Standard 点
		std::vector<MeasurePoint> fileStandardCircleMPs;  // 前端文件中的 Standard_circle 点
		for (const auto& mp : fileMPs)
		{
			if (isSurfaceMP(mp))
				fileSurfaceMPs.push_back(mp);        // 拐角自查需要完整集合，先全部收集
			else if (isTrimMP(mp))
				fileTrimMPs.push_back(mp);
			else if (isStandardCircleMP(mp))         // 必须先判圆类，"Standard_circle" 含子串 "Standard"
				fileStandardCircleMPs.push_back(mp);
			else if (isStandardMP(mp))
				fileStandardMPs.push_back(mp);
		}

		// （新增）面点自查：相邻且第一矢量近 90° 的一对 → 生成"斜面点"并入同一容器
		appendDiagonalSurfaceMPs(fileSurfaceMPs);

		// （新增）整合后再统一做 Z 过滤：原始面点 + 斜面点一起剔除第一矢量与 Z 夹角 <10° 的
		{
			std::vector<MeasurePoint> keptSurfaceMPs;
			keptSurfaceMPs.reserve(fileSurfaceMPs.size());
			for (const auto& mp : fileSurfaceMPs)
				if (!isVectorNearZ(mp))
					keptSurfaceMPs.push_back(mp);
			fileSurfaceMPs.swap(keptSurfaceMPs);
		}

		// ------------------------------------------------------------------
		// 构建 RobotConfig（从原 BallScan::allocation() 内部搬到此处）
		// 三个阶段共享同一个 cfg 实例
		// ------------------------------------------------------------------
		robot_planner::RobotConfig cfg;
		cfg.robotName = robotNmae;
		auto tech_it = robots_tech_params.find(robotNmae);
		if (tech_it != robots_tech_params.end())
		{
			const RobotTechParameters& rtp = tech_it->second;
			cfg.jointLimits[0] = { rtp.t1min, rtp.t1max };
			cfg.jointLimits[1] = { rtp.t2min, rtp.t2max };
			cfg.jointLimits[2] = { rtp.t3min, rtp.t3max };
			cfg.jointLimits[3] = { rtp.t4min, rtp.t4max };
			cfg.jointLimits[4] = { rtp.t5min, rtp.t5max };
			cfg.jointLimits[5] = { rtp.t6min, rtp.t6max };
			cfg.T_he = rtp.handEyeMatrix;
			if (rtp.initial_angle.size() >= 6)
				for (int i = 0; i < 6; ++i)
					cfg.robotHome[i] = rtp.initial_angle[i];
			cfg.lastView = cfg.robotHome;

			// 从 home 位提取滑轨位置，覆盖默认 -3500.0
			{
				JointAngles homeJa = Robot::convertToJointAngles(
					scene.getRobotByName(robotNmae).getHomeJoints());
				std::cout << "[DEBUG][main] homeJa.slider = " << homeJa.slider
					<< (std::isnan(homeJa.slider) ? "  ← NaN，不覆盖，保持默认 -3500.0" : "  ← 有效，将覆盖 slidePos")
					<< std::endl;
				if (!std::isnan(homeJa.slider))
					cfg.slidePos = homeJa.slider;
				std::cout << "[DEBUG][main] cfg.slidePos 最终值 = " << cfg.slidePos << std::endl;
			}
		}

		// ------------------------------------------------------------------
		// 创建 BallScan 和 ScanState（每个机器人一份，跨三个阶段传递）
		// ------------------------------------------------------------------
		BallScan ballScan;
		BallScan::ScanState state;
		state.lastJoints = cfg.robotHome;
		std::vector<robot_planner::ViewPoint> combinedEdgePath;
		std::vector<robot_planner::ViewPoint> allSurfacePath;
		std::vector<robot_planner::ViewPoint> allStandardPath;   // Standard/Standard_circle 可达视点路径（独立于面点）

		auto angleShiftMeasurePonits = task.second;

		// 这里得到两个站位的位置环境
		// 得到导轨数值然后更新测试环境
		// 写一个循环区别两个站位
		// 
		// ════════════════════════════════════════════════════════════════
		// 阶段1：面点 → 转台重试循环
		//
		// 第一个转台角度测所有面点，不可达的面点滚到下一个转台角度重试，
		// 直到所有转台角度都测完。
		// ════════════════════════════════════════════════════════════════
		{
			std::vector<MeasurePoint> init_un_measure_point;
			bool isFirstAngle = true;

			for (auto& angleTask : angleShiftMeasurePonits)
			{
				double angle = angleTask.first;
				if (angle == -400) continue;

				auto homeJ = scene.getRobotByName(robotNmae).getHomeJoints();
				auto homeJoint = Robot::convertToJointAngles(homeJ);
				double shift = homeJoint.slider;

				// 合并待测点：第一个角度全部面点，后续角度只测不可达点
				std::vector<MeasurePoint> allShiftTaskPoints;
				if (isFirstAngle)
				{
					allShiftTaskPoints.insert(allShiftTaskPoints.end(),
						surfaceMPs.begin(), surfaceMPs.end());          // 几何生成面点


					//allShiftTaskPoints.insert(allShiftTaskPoints.end(),
					//	fileSurfaceMPs.begin(), fileSurfaceMPs.end());  // （新增）前端文件面点


					isFirstAngle = false;
				}
				for (const auto& mp : init_un_measure_point)
				{
					allShiftTaskPoints.push_back(mp);
				}

				// 全部可达则提前退出
				if (allShiftTaskPoints.empty()) break;

				// 设置转台角度
				auto robot = scene.findRobotByName(robotNmae);
				auto& turnTables = scene.getTurnTables();
				auto& turnTable = turnTables[robotNmae];
				turnTable.setTurnTableAngle(angle);
				 
				// 变换面点跟随零件
				// 注意：面点没有自己的 shapeNode，用零件节点获取变换矩阵
				std::vector<MeasurePoint> finalTaskPoints;
				gp_Trsf measurePointTrsf;

				if (!allShiftTaskPoints.empty())
				{
					auto firstPointShapeNode = scene.getNodeManager()
						->findShapeNode(partNames[0]);
					if (firstPointShapeNode == nullptr)
					{
						QString meassge = "零件:" +
							QString::fromStdString(partNames[0])
							+ "的显示节点缺失！！！";
						MessageManager::instance()->log(
							LogLevel::LOG_ERROR, meassge);
						return;
					}
					auto matrix = firstPointShapeNode->getAppliedTransforms();
					measurePointTrsf = TransformConverter::toGpTrsf(matrix);
				}

				for (auto point : allShiftTaskPoints)
				{
					finalTaskPoints.push_back(
						MeasurePointUtils::TransformMeasurePoint(
							point, measurePointTrsf));
					final_unreach_shift_angle[point.name] = { shift, angle };
				}

				// 调用面点可达性分析
				std::map<std::string, std::vector<MeasurePointPoseSet>> robots_measure_points;
				std::map<std::string, std::vector<MeasurePointPoseSet>> un_measure_point;
				std::vector<robot_planner::ViewPoint> surfacePath;


				ballScan.analyzeSurfacePoints(
					robotNmae, finalTaskPoints, cfg, state,
					surfacePath,
					robots_measure_points[robotNmae],
					un_measure_point[robotNmae]);

				// analyzeSurfacePoints 内部不感知当前转台角度；在汇总路径前补齐，
				// 使后续 aaa 能在同一角度内匹配弓字形面点与文件测点。
				for (auto& vp : surfacePath)
					vp.partRotateAngle = angle;
				allSurfacePath.insert(allSurfacePath.end(), surfacePath.begin(), surfacePath.end());

				// 收集可达结果
				for (auto& [rName, poseSets] : robots_measure_points)
				{
					measure_point_result[rName].insert(
						measure_point_result[rName].end(),
						poseSets.begin(), poseSets.end());
				}

				// 滚动不可达：从原始面点列表中找回未变换的坐标，滚到下一个角度
				init_un_measure_point.clear();
				for (const auto& [k, poseSets] : un_measure_point)
				{
					un_measure_point_result[k].insert(
						un_measure_point_result[k].end(),
						poseSets.begin(), poseSets.end());
					for (auto& ps : poseSets)
					{
						auto mp = MeasurePointUtils::FindMeasurePointByName(
							surfaceMPs, ps.name);
						if (!mp) continue;
						init_un_measure_point.push_back(*mp);
					}
				}
			} // end 阶段1 转台循环
		}

		// ════════════════════════════════════════════════════════════════
		// 阶段2：切边点 + 旋转切边点 合并可达性分析（每转台角度单次 SA）
		//
		// 设计要点：
		//   1) 进入循环前一次性构造 allCombinedPoints：每个切边点产生两条
		//      MeasurePoint —— 一条 name 不变（normal 姿态），一条 name 追加
		//      ROTATED_NAME_SUFFIX("_R")（旋转姿态），xyz/ijk 完全相同。
		//      这就是"通过切边点生成旋转切边点"的实现。
		//   2) 每个转台角度只调一次 ballScan.analyzeEdgePointsCombined：
		//      内部做一次 SA 排序 + 链式 TCP 构造 + 可达性判定；
		//      构造完 tcp_pose 后，若测点 name 带 _R 后缀，则右乘 Rx(-45°)。
		//      4 个有效转台角度共 4 次退火。
		//   3) 不可达点的 carry-over：用名字（含 _R 后缀）在 allCombinedPoints
		//      里回查到原始未变换 MeasurePoint，作为下一个转台角度的输入。
		//      下一轮 setTurnTableAngle + measurePointTrsf 会重新把它变换到
		//      新角度坐标，等价于"重新算出新角度下的（旋转）切边点"。
		//   4) state 从阶段1末尾继续，关节连续性由 analyzeEdgePointsCombined 内部维护。
		// ════════════════════════════════════════════════════════════════

		// 进入循环前一次性构造 allCombinedPoints（normal + rotated 副本）
		// 后续 carry-over 必须在 allCombinedPoints 里查找（不再用 fileTrimMPs），
		// 否则名字带 _R 后缀的副本无法被找到。
		std::vector<MeasurePoint> allCombinedPoints;
		allCombinedPoints.reserve(fileTrimMPs.size() * 2);
		for (const auto& mp : fileTrimMPs)
		{
			allCombinedPoints.push_back(mp);                                  // normal 副本
			MeasurePoint mpRot = mp;
			mpRot.name = mp.name + BallScan::ROTATED_NAME_SUFFIX;             // 旋转副本（xyz/ijk 不变）
			allCombinedPoints.push_back(mpRot);
		}

		// SA 起点参考：阶段1末尾位姿；每个角度结束后若有过渡点则更新为过渡点位姿
		robot_planner::Matrix4d placeEndTcp = state.lastViewPose;
		{
			std::vector<MeasurePoint> init_un_measure_point;
			bool isFirstAngle = true;

			// 预计算有效转台角度列表，用于判断最后一个角度（最后一个不加过渡点）
			std::vector<double> validRotAngles;
			for (auto& at : angleShiftMeasurePonits)
				if (at.first != -400.0) validRotAngles.push_back(at.first);

			for (auto& angleTask : angleShiftMeasurePonits)
			{
				double angle = angleTask.first;
				if (angle == -400) continue;

				auto homeJ = scene.getRobotByName(robotNmae).getHomeJoints();
				auto homeJoint = Robot::convertToJointAngles(homeJ);
				double shift = homeJoint.slider;

				// 1) 准备本轮输入：第一轮 = 全集（含 _R 副本）；后续轮 = 上轮不可达 carry
				std::vector<MeasurePoint> allShiftTaskPoints;
				if (isFirstAngle)
				{
					allShiftTaskPoints.insert(allShiftTaskPoints.end(),
						allCombinedPoints.begin(), allCombinedPoints.end());
					isFirstAngle = false;
				}
				for (const auto& mp : init_un_measure_point)
				{
					allShiftTaskPoints.push_back(mp);
				}

				if (allShiftTaskPoints.empty()) break;

				// 2) 设置转台角度
				auto robot = scene.findRobotByName(robotNmae);
				auto& turnTables = scene.getTurnTables();
				auto& turnTable = turnTables[robotNmae];
				turnTable.setTurnTableAngle(angle);

				// 3) 取场景 transform：注意 _R 副本不是场景节点，必须用 name
				//    去除 _R 后缀后找原 shapeNode；任选输入里第一个 normal 测点即可
				std::vector<MeasurePoint> finalTaskPoints;
				gp_Trsf measurePointTrsf;

				if (!allShiftTaskPoints.empty())
				{
					// 找一个不带 _R 后缀的测点 name 用于查找 shapeNode；
					// 若输入全是 _R 副本（极少见），就把后缀剥掉再查。
					std::string shapeQueryName;
					const std::string rotSuffix = BallScan::ROTATED_NAME_SUFFIX;
					for (const auto& p : allShiftTaskPoints)
					{
						if (p.name.size() < rotSuffix.size() ||
							!std::equal(rotSuffix.rbegin(), rotSuffix.rend(),
								p.name.rbegin()))
						{
							shapeQueryName = p.name;
							break;
						}
					}
					if (shapeQueryName.empty())
					{
						const std::string& fn = allShiftTaskPoints[0].name;
						shapeQueryName = fn.substr(0, fn.size() - rotSuffix.size());
					}

					auto firstPointShapeNode = scene.getNodeManager()
						->findShapeNode(shapeQueryName);
					if (firstPointShapeNode == nullptr)
					{
						QString meassge = "测点:" +
							QString::fromStdString(shapeQueryName)
							+ "的显示节点缺失！！！";
						MessageManager::instance()->log(
							LogLevel::LOG_ERROR, meassge);
						return;
					}
					auto matrix = firstPointShapeNode->getAppliedTransforms();
					measurePointTrsf = TransformConverter::toGpTrsf(matrix);
				}

				for (auto point : allShiftTaskPoints)
				{
					finalTaskPoints.push_back(
						MeasurePointUtils::TransformMeasurePoint(
							point, measurePointTrsf));
					final_unreach_shift_angle[point.name] = { shift, angle };
				}

				// 4) 调合并分析：内部一次 SA + 链式 TCP（_R 后缀右乘 Rx(-45°)）
				std::map<std::string, std::vector<MeasurePointPoseSet>> robots_measure_points;
				std::map<std::string, std::vector<MeasurePointPoseSet>> un_measure_point;

				std::vector<robot_planner::ViewPoint> edgePath;
				ballScan.analyzeEdgePointsCombined(
					robotNmae, finalTaskPoints, -45.0, cfg, state,
					placeEndTcp, angle,
					robots_measure_points[robotNmae],
					un_measure_point[robotNmae],
					edgePath);
				combinedEdgePath.insert(combinedEdgePath.end(),
					edgePath.begin(), edgePath.end());

				// 5) 收集可达结果
				for (auto& [rName, poseSets] : robots_measure_points)
				{
					measure_point_result[rName].insert(
						measure_point_result[rName].end(),
						poseSets.begin(), poseSets.end());
				}

				// 6) 滚动不可达：用名字（含 _R 后缀）在 allCombinedPoints 中回查
				//    原始未变换 MeasurePoint 作为下一轮输入。
				init_un_measure_point.clear();
				for (const auto& [k, poseSets] : un_measure_point)
				{
					un_measure_point_result[k].insert(
						un_measure_point_result[k].end(),
						poseSets.begin(), poseSets.end());
					for (auto& ps : poseSets)
					{
						auto mp = MeasurePointUtils::FindMeasurePointByName(
							allCombinedPoints, ps.name);
						if (!mp) continue;
						init_un_measure_point.push_back(*mp);
					}
				}

				// 7) 过渡点：最后一个角度不加；其他角度且 carry 非空才加
				//    将过渡点位姿作为下一轮 SA 的起点参考 placeEndTcp，
				//    同时插入 combinedEdgePath 作为安全中间位姿避免转台旋转碰撞
				bool isLastAngle = validRotAngles.empty() ||
					(angle == validRotAngles.back());
				if (!isLastAngle && !init_un_measure_point.empty())
				{
					robot_planner::JointVec6 transJoints;
					placeEndTcp = BallScan::computeTransitionPose(
						state.lastViewPose, cfg.mainNormal, cfg.shiftDistance, cfg, state,
						&transJoints);

					robot_planner::ViewPoint transVp;
					transVp.globalT = placeEndTcp;
					transVp.joints = transJoints;
					transVp.isImportant = false;
					transVp.isPathJoint = true;
					transVp.moveType = 0;
					transVp.partRotateAngle = angle;
					combinedEdgePath.push_back(transVp);
				}
			} // end 合并阶段 转台循环
		}

		// ════════════════════════════════════════════════════════════════
		// 阶段3：Standard / Standard_circle 测点 → 转台重试循环（锥形 N 视点）
		//
		// 框架照搬阶段1：第一个转台角度测全部 Standard 点，不可达点滚到下一个
		// 转台角度重试；可达/不可达分别收集到 measure_point_result /
		// un_measure_point_result；可达路径追加到独立容器 allStandardPath。
		// 两类测点分别调用 analyzeStandardPoints：
		//   Standard       → tiltDeg=40, coneCount=6 （步进 60°）
		//   Standard_circle→ tiltDeg=40, coneCount=12（步进 30°）
		// ════════════════════════════════════════════════════════════════
		{
			// ── 表驱动：把「源容器 + 锥形参数」打包成一张表，对两类测点跑同一套循环 ──
			// 阶段3 与前两阶段的区别：阶段1 只有面点、阶段2 只有切边点，各自单流；
			// 而阶段3 要处理 Standard 和 Standard_circle 两类，且锥形参数不同
			//   （Standard: coneCount=6 步进60°；Standard_circle: coneCount=12 步进30°）。
			// 为避免把下面整段「转台重试循环」复制粘贴两遍，这里用一个小结构体数组
			// 描述每类测点的差异（源容器 src / 半顶角 tiltDeg / 一圈视点数 coneCount），
			// 再用一个 for 遍历这张表，对每类各执行一遍完全相同的循环体。
			struct StdGroup {
				std::vector<MeasurePoint>* src;   // 指向该类测点的源容器
				double tiltDeg;                   // 锥半顶角（绕局部 x 轴掀开的角度）
				int    coneCount;                 // 一圈视点数（步进角 = 360/coneCount）
			};
			StdGroup stdGroups[] = {
				{ &fileStandardMPs,       30, 6  },   // Standard
				{ &fileStandardCircleMPs, 30, 12 },   // Standard_circle
			};

			// 依次处理每一类测点；循环体内统一用 grp.src / grp.tiltDeg / grp.coneCount，
			// 逻辑与阶段1 的面点转台重试循环完全一致。
			for (auto& grp : stdGroups)
			{
				if (grp.src->empty()) continue;   // 该类没有测点则跳过

				std::vector<MeasurePoint> init_un_measure_point;
				bool isFirstAngle = true;

				for (auto& angleTask : angleShiftMeasurePonits)
				{
					double angle = angleTask.first;
					if (angle == -400) continue;

					auto homeJ = scene.getRobotByName(robotNmae).getHomeJoints();
					auto homeJoint = Robot::convertToJointAngles(homeJ);
					double shift = homeJoint.slider;

					// 合并待测点：第一个角度全部 Standard 点，后续角度只测不可达点
					std::vector<MeasurePoint> allShiftTaskPoints;
					if (isFirstAngle)
					{
						allShiftTaskPoints.insert(allShiftTaskPoints.end(),
							grp.src->begin(), grp.src->end());
						isFirstAngle = false;
					}
					for (const auto& mp : init_un_measure_point)
						allShiftTaskPoints.push_back(mp);

					// 全部可达则提前退出
					if (allShiftTaskPoints.empty()) break;

					// 设置转台角度
					auto robot = scene.findRobotByName(robotNmae);
					auto& turnTables = scene.getTurnTables();
					auto& turnTable = turnTables[robotNmae];
					turnTable.setTurnTableAngle(angle);

					// 变换测点跟随零件（与阶段1一致，用零件节点的变换矩阵）
					std::vector<MeasurePoint> finalTaskPoints;
					gp_Trsf measurePointTrsf;
					{
						auto firstPointShapeNode = scene.getNodeManager()
							->findShapeNode(partNames[0]);
						if (firstPointShapeNode == nullptr)
						{
							QString meassge = "零件:" +
								QString::fromStdString(partNames[0])
								+ "的显示节点缺失！！！";
							MessageManager::instance()->log(
								LogLevel::LOG_ERROR, meassge);
							return;
						}
						auto matrix = firstPointShapeNode->getAppliedTransforms();
						measurePointTrsf = TransformConverter::toGpTrsf(matrix);
					}

					for (auto point : allShiftTaskPoints)
					{
						finalTaskPoints.push_back(
							MeasurePointUtils::TransformMeasurePoint(
								point, measurePointTrsf));
						final_unreach_shift_angle[point.name] = { shift, angle };
					}

					// 调用 Standard 测点可达性分析（锥形 N 视点）
					std::map<std::string, std::vector<MeasurePointPoseSet>> robots_measure_points;
					std::map<std::string, std::vector<MeasurePointPoseSet>> un_measure_point;
					std::vector<robot_planner::ViewPoint> standardPath;

					// 仅为锥形测点分析临时放宽 J4(下标3)/J6(下标5) 前后关节角约束到 160°：
					// 绕法向扫锥时腕部 J4/J6 跟随滚动，接缝处或腕冗余翻转会让相邻锥形视点
					// 角差 >120°，被默认阈值误杀。这些是同位置、各自独立的 MOVJ 测点，大幅
					// 腕动可执行，故放宽以救回解。调用后立即还原，使后续 MOVL 直线预测
					// (measLinePathPredict) 仍用 120° 不受影响、点数不膨胀。
					auto savedAnglesTh = cfg.anglesThresholds;
					cfg.anglesThresholds[3] = 160.0;
					cfg.anglesThresholds[5] = 160.0;

					ballScan.analyzeStandardPoints(
						robotNmae, finalTaskPoints, cfg, state,
						standardPath,
						robots_measure_points[robotNmae],
						un_measure_point[robotNmae],
						grp.tiltDeg, grp.coneCount);

					cfg.anglesThresholds = savedAnglesTh;   // 还原，避免污染 MOVL 段

					// Standard 视点 partRotateAngle 在 analyzeStandardPoints 内写死为 0.0，
					// 这里改写为本轮转台角度 angle，使其在 aaa 中按正确角度分组、
					// 在对应碰撞环境下做直线预测。写入独立容器 allStandardPath（不并入面点）。
					for (auto& vp : standardPath)
						vp.partRotateAngle = angle;
					allStandardPath.insert(allStandardPath.end(),
						standardPath.begin(), standardPath.end());

					// 收集可达结果
					for (auto& [rName, poseSets] : robots_measure_points)
					{
						measure_point_result[rName].insert(
							measure_point_result[rName].end(),
							poseSets.begin(), poseSets.end());
					}

					// 滚动不可达：用名字在源容器中回查未变换的测点，滚到下一个角度。
					// 注意：analyzeStandardPoints 输出的 name 带 _C00.._Cnn 锥形后缀，
					// 一个测点的多个锥形视点可能部分可达部分不可达。这里以“原测点名”
					// 去重——只要该测点尚无任何锥形视点可达，才整体滚到下一角度重试。
					{
						for (const auto& [k, poseSets] : un_measure_point)
							un_measure_point_result[k].insert(
								un_measure_point_result[k].end(),
								poseSets.begin(), poseSets.end());

						auto stripConeSuffix = [](const std::string& n) -> std::string {
							auto p = n.rfind("_C");
							if (p != std::string::npos && p + 2 < n.size())
								return n.substr(0, p);
							return n;
						};
						// 本轮“有可达锥形视点”的原测点名集合
						std::set<std::string> reachedBaseNames;
						for (const auto& [rName, poseSets] : robots_measure_points)
							for (const auto& ps : poseSets)
								reachedBaseNames.insert(stripConeSuffix(ps.name));

						init_un_measure_point.clear();
						std::set<std::string> carriedBaseNames;  // 去重，避免同测点多个 _Cxx 重复滚动
						for (const auto& [k, poseSets] : un_measure_point)
						{
							for (auto& ps : poseSets)
							{
								std::string base = stripConeSuffix(ps.name);
								if (reachedBaseNames.count(base)) continue;   // 已有锥形视点可达
								if (!carriedBaseNames.insert(base).second) continue;
								auto mp = MeasurePointUtils::FindMeasurePointByName(
									*grp.src, base);
								if (!mp) continue;
								init_un_measure_point.push_back(*mp);
							}
						}
					}
				} // end 阶段3 转台循环
			} // end 两类 Standard 测点
		}

		// 在切边路径末尾追加 home 位回原点视点，使 JBI 程序最后执行 MOVJ 回 home。
		// globalT 填零矩阵（此处不调 FK），partRotateAngle/partTransform 跟随末尾点，
		// 确保落入同一 aaa 分组被正确导出。（新增）
		if (!combinedEdgePath.empty()) {
			robot_planner::Matrix4d zeroMat(4, std::vector<double>(4, 0.0));
			robot_planner::ViewPoint homeVp;
			homeVp.joints          = cfg.robotHome;
			homeVp.globalT         = zeroMat;
			homeVp.moveType        = 0;
			homeVp.isImportant     = true;
			homeVp.isPathJoint     = true;
			homeVp.partRotateAngle = combinedEdgePath.back().partRotateAngle;
			homeVp.partTransform   = combinedEdgePath.back().partTransform;
			combinedEdgePath.push_back(homeVp);
		}

		// homeVp 的 globalT 原为全零矩阵，会导致 measLinePathPredict 向机器原点插值，
		// 所有插值位姿 IK 失败 → 大量 -2 占位 → markAndCleanMinusTwo 把 homeVp 标为
		// moveType=-1 → spliceRRT 失败时 homeVp 被降级为 MOVL。
		// 修复：用 FK(cfg.robotHome) 覆盖已入队的 homeVp.globalT，使插值正确。（新增）
		if (!combinedEdgePath.empty()) {
			const std::vector<double> homeQ(cfg.robotHome.begin(), cfg.robotHome.end());
			const auto fkRes = KinematicsApi::instance().fk(robotNmae, homeQ, LinkName::TCP);
			robot_planner::Matrix4d homeGT = robot_planner::eye4();
			if (fkRes.ut.size() >= 4) {
				bool valid = true;
				for (int r = 0; r < 4 && valid; ++r)
					if (fkRes.ut[static_cast<std::size_t>(r)].size() < 4) valid = false;
				if (valid)
					for (int r = 0; r < 4; ++r)
						for (int c = 0; c < 4; ++c)
							homeGT[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] =
								fkRes.ut[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
			}
			combinedEdgePath.back().globalT = homeGT;
		}

		// homeVp 的 home 回归段是 MOVJ，不应进入 measLinePathPredict 的 Cartesian
		// 插值循环（笛卡尔直线路径必然穿越碰撞区 → 大量中间点被标 isImportant=true
		// → 泄漏到 JBI）。在 aaa 分组前把它摘出，applyMeasLinePathPredict 完成后
		// 直接追加到对应角度组末尾，以 MOVJ 形式进入 JBI。（新增）
		robot_planner::ViewPoint savedHomeVp;
		bool hasSavedHomeVp = false;
		if (!combinedEdgePath.empty()) {
			savedHomeVp    = combinedEdgePath.back();  // 保存已有正确 globalT 的 homeVp
			combinedEdgePath.pop_back();               // 从路径摘出，不参与插值
			hasSavedHomeVp = true;
		}

		std::map<double, std::vector< robot_planner::ViewPoint>> aaa;
			// 将测点文件产生的 Standard、Standard_circle、Trim（含 _R）
			// 统一按“转台角度 + 基准测点”分组，再以弓字形面点为骨架组装 aaa。
			// 同位置整组插入，未匹配整组追加到对应角度末尾。
			struct MeasurePoseGroup {
				double angle = 0.0;
				std::string sourcePointId;
				robot_planner::Vec3d sourcePosition = {};
				bool hasSourcePosition = false;
				bool inserted = false;
				std::vector<robot_planner::ViewPoint> poses;
			};

			std::map<double, std::vector<MeasurePoseGroup>> measureGroupsByAngle;
			auto collectMeasureGroups = [&](const std::vector<robot_planner::ViewPoint>& source) {
				for (const auto& vp : source) {
					auto& groups = measureGroupsByAngle[vp.partRotateAngle];
					// Trim 的 normal / _R 是同一个基准切边点的两种姿态，
					// 分组时去掉来源 ID 中的 _R 后缀，使两者整组插入。
					std::string groupPointId = vp.sourcePointId;
					const std::string rotatedIdPart =
						std::string(BallScan::ROTATED_NAME_SUFFIX) + "|";
					const auto rotatedIdPos = groupPointId.rfind(rotatedIdPart);
					if (rotatedIdPos != std::string::npos)
						groupPointId.erase(rotatedIdPos,
							std::string(BallScan::ROTATED_NAME_SUFFIX).size());
				auto groupIt = std::find_if(groups.begin(), groups.end(),
					[&](const MeasurePoseGroup& group) {
						return group.sourcePointId == groupPointId
							&& group.hasSourcePosition == vp.hasSourcePointPosition;
					});
				if (groupIt == groups.end()) {
					MeasurePoseGroup group;
					group.angle = vp.partRotateAngle;
					group.sourcePointId = groupPointId;
					group.sourcePosition = vp.sourcePointPosition;
					group.hasSourcePosition = vp.hasSourcePointPosition;
					groups.push_back(std::move(group));
					groupIt = groups.end() - 1;
				}
				groupIt->poses.push_back(vp);
				}
			};
			collectMeasureGroups(allStandardPath);
			collectMeasureGroups(combinedEdgePath);

			// 文件坐标经过同一刚体变换后理论上应完全一致；保留 0.5 mm 容差，
			// 吸收文件小数截断/浮点变换误差，同时远小于面扫点间距。
			constexpr double kSamePositionToleranceMm = 300;
			const double samePositionToleranceSq =
				kSamePositionToleranceMm * kSamePositionToleranceMm;
			auto sameSourcePosition = [&](const robot_planner::ViewPoint& surfaceVp,
				const MeasurePoseGroup& group) {
				if (!surfaceVp.hasSourcePointPosition || !group.hasSourcePosition)
					return false;
				const double dx = surfaceVp.sourcePointPosition[0] - group.sourcePosition[0];
				const double dy = surfaceVp.sourcePointPosition[1] - group.sourcePosition[1];
				const double dz = surfaceVp.sourcePointPosition[2] - group.sourcePosition[2];
				return dx * dx + dy * dy + dz * dz <= samePositionToleranceSq;
			};

			for (const auto& surfaceVp : allSurfacePath) {
				auto& ordered = aaa[surfaceVp.partRotateAngle];
				ordered.push_back(surfaceVp);

				auto groupsIt = measureGroupsByAngle.find(surfaceVp.partRotateAngle);
				if (groupsIt == measureGroupsByAngle.end()) continue;
				for (auto& group : groupsIt->second) {
					if (group.inserted || !sameSourcePosition(surfaceVp, group)) continue;
					ordered.insert(ordered.end(), group.poses.begin(), group.poses.end());
					group.inserted = true;
				}
			}

			// 未匹配组（以及本角度没有弓字形面点的组）统一放到对应角度末尾。
			for (auto& angleGroups : measureGroupsByAngle) {
				auto& ordered = aaa[angleGroups.first];
				for (auto& group : angleGroups.second) {
					if (group.inserted) continue;
					ordered.insert(ordered.end(), group.poses.begin(), group.poses.end());
					group.inserted = true;
				}
			}

			for (auto& it : aaa)
			{
				double angle = it.first;
				// 设置转台角度
				auto robot = scene.findRobotByName(robotNmae);
				auto& turnTables = scene.getTurnTables();
				auto& turnTable = turnTables[robotNmae];
				turnTable.setTurnTableAngle(angle);

				// 在当前转台角度的碰撞环境下对本角度路径做直线预测 + RRT 规划（新增）
  				BallScan::applyMeasLinePathPredict(it.second, cfg);

			}

			// 所有角度组的 applyMeasLinePathPredict 完成后，将 homeVp 追加到它所属角度组
			// 的末尾。homeVp 的 moveType=0 保证直接以 MOVJ 写入 JBI，不再经历插值。（新增）
			if (hasSavedHomeVp && !aaa.empty()) {
				aaa[savedHomeVp.partRotateAngle].push_back(savedHomeVp);
			}

		//==================================================================
		//直线轨迹预测 + 后处理：切边点和旋转切边点可达性分析全部完成后统一执行
		//对应 MATLAB: MeasLinePathPredict（逐段） → processLineResult
		//applyMeasLinePathPredict 内部已调用 processLineResult
		//==================================================================
		//BallScan::applyMeasLinePathPredict(combinedEdgePath, cfg);
		//if (!combinedEdgePath.empty()) {
		//	state.lastViewPose = combinedEdgePath.back().globalT;
		//	state.lastJoints = combinedEdgePath.back().joints;
		//}
		//combinedEdgePath从viewpoint变成Measurepointset格式给到前端场景中,编号，xyzijk，对应的姿态关节角，滑轨，转台数据都要
		std::map<std::string, std::vector<MeasurePointPoseSet>> pathPoints;
		//
		// ── pathPoints 装填：把 aaa 各角度组（applyMeasLinePathPredict 处理后
		// 的面点 + 切边点 + 过渡点 + homeVp）转成 MeasurePointPoseSet 喂给前端
		// scene 渲染。
		//   编号策略：按 aaa 自然顺序（转台角度升序）全局自增，从 1 起；
		//             name = "path_<idx>"，vct_vector_type 留空。
		//   xyz / ijk：用 FK(joints) 重算，避免 vp.globalT 在 RRT 段中可能为
		//             eye4 / 全零的边界情况；ijk 为 X-Y-Z 欧拉角（度）。
		//   滑轨：cfg.slidePos（与 JBIExporter 写 row[7] 同源），单位 mm。
		//   转台：vp.partRotateAngle，单位度。
		// 全部新增、无既有行改动。（新增）
		{
			static const double NEW_RAD_TO_DEG = 180.0 / 3.14159265358979323846;
			int pathIdx = 1;
			for (auto& it : aaa) {
				for (const auto& vp : it.second) {
					MeasurePointPoseSet mps;
					mps.point_number    = pathIdx;
					mps.name            = "path_" + std::to_string(pathIdx);
					mps.vct_vector_type = "";
					mps.isMeasurePoint  = false;
					mps.is_accessible   = true;
					mps.moveType = vp.moveType;

					// FK 计算用户坐标系下 TCP 位姿
					const std::vector<double> q(vp.joints.begin(), vp.joints.end());
					const auto fkRes = KinematicsApi::instance().fk(
						robotNmae, q, LinkName::TCP);

					if (fkRes.position.size() >= 3) {
						mps.x = fkRes.position[0];
						mps.y = fkRes.position[1];
						mps.z = fkRes.position[2];
					} else {
						mps.x = 0.0; mps.y = 0.0; mps.z = 0.0;
					}

					// 从 fkRes.ut 取 3×3 旋转矩阵 → rotm2eul_XYZ → 度
					bool hasFullUt = (fkRes.ut.size() >= 3)
						&& (fkRes.ut[0].size() >= 3)
						&& (fkRes.ut[1].size() >= 3)
						&& (fkRes.ut[2].size() >= 3);
					if (hasFullUt) {
						std::vector<std::vector<double>> R(3, std::vector<double>(3));
						for (int r = 0; r < 3; ++r)
							for (int c = 0; c < 3; ++c)
								R[r][c] = fkRes.ut[r][c];
						const auto eul = rotm2eul_XYZ(R);
						if (eul.size() >= 3) {
							mps.i = eul[0] * NEW_RAD_TO_DEG;
							mps.j = eul[1] * NEW_RAD_TO_DEG;
							mps.k = eul[2] * NEW_RAD_TO_DEG;
						} else {
							mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
						}
					} else {
						mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
					}
					mps.i2 = 0.0; mps.j2 = 0.0; mps.k2 = 0.0;

					// 关节角 + 滑轨 + 转台 + 笛卡尔位姿 写入 PoseConfiguration
					PoseConfiguration pc;
					pc.point_number = pathIdx;
					pc.point_name   = mps.name;
					pc.joints.j1 = vp.joints[0];
					pc.joints.j2 = vp.joints[1];
					pc.joints.j3 = vp.joints[2];
					pc.joints.j4 = vp.joints[3];
					pc.joints.j5 = vp.joints[4];
					pc.joints.j6 = vp.joints[5];
					pc.joints.slider    = cfg.slidePos;        // 滑轨（mm）
					pc.joints.turnTable = vp.partRotateAngle;  // 转台（度）
					pc.xyzwpr.x = mps.x; pc.xyzwpr.y = mps.y; pc.xyzwpr.z = mps.z;
					pc.xyzwpr.w = mps.i; pc.xyzwpr.p = mps.j; pc.xyzwpr.r = mps.k;
					mps.configurations.push_back(pc);

					pathPoints[robotNmae].push_back(mps);
					++pathIdx;
				}
			}
		}

		//std::map<std::string, std::vector<MeasurePointPoseSet>> csvPathPoints;
		//// Change only this path to switch to another compatible path-point CSV.
		//const std::string pathpointsCsvPath =
		//	R"(D:\test\QtWidgetsApplication4\data\paper_pso_pathpoints_updated.csv)";
		//if (!loadPathpointsCsv(
		//	pathpointsCsvPath, robotNmae, cfg.slidePos, csvPathPoints)) {
		//	MessageManager::instance()->log(
		//		LogLevel::LOG_ERROR,
		//		QString::fromStdString("Failed to load path-points CSV: " + pathpointsCsvPath));
		//	return;
		//}
		//pathPoints = std::move(csvPathPoints);
	 	scene.addMeasurePathPoint(pathPoints);

		// ==================================================================
		// 调试：将 applyMeasLinePathPredict 处理后的完整路径（面点 + 切边点，
		// 取自 aaa 各转台角度的 it.second）序列化为 JSON。与上方
		// combinedEdgePath_debug.json 的区别：此处是处理后数据（直线预测 /
		// RRT 拼接 / MOVL 安全降级），且范围含面点；内容与 JBI 导出用的
		// predictedAllPath 一致（JBI 还会再过 insertMovlToMovjTransitions）。（新增）
		// ==================================================================
		{
 			std::ofstream ofsPred("D:/output/predictedAllPath_debug.json");
			if (ofsPred.is_open()) {
				// 矩阵转 JSON 的局部 lambda（与上方块各自独立）（新增）
				auto mat4ToJsonPred = [](const robot_planner::Matrix4d& m) -> std::string {
					std::ostringstream ss;
					ss << "[";
					for (size_t r = 0; r < m.size(); ++r) {
						ss << "[";
						for (size_t c = 0; c < m[r].size(); ++c) {
							ss << m[r][c];
							if (c + 1 < m[r].size()) ss << ",";
						}
						ss << "]";
						if (r + 1 < m.size()) ss << ",";
					}
					ss << "]";
					return ss.str();
					};

				// 按转台角度升序拼接 aaa 中所有处理后路径点（新增）
				std::vector<robot_planner::ViewPoint> predictedDump;
				for (auto& it : aaa)
					predictedDump.insert(predictedDump.end(),
						it.second.begin(), it.second.end());

				ofsPred << "[\n";
				for (size_t i = 0; i < predictedDump.size(); ++i) {
					const auto& vp = predictedDump[i];
					ofsPred << "  {\n";
					ofsPred << "    \"globalT\": " << mat4ToJsonPred(vp.globalT) << ",\n";
					ofsPred << "    \"joints\": [" << vp.joints[0] << "," << vp.joints[1] << ","
						<< vp.joints[2] << "," << vp.joints[3] << "," << vp.joints[4] << ","
						<< vp.joints[5] << "],\n";
					ofsPred << "    \"isImportant\": " << (vp.isImportant ? "true" : "false") << ",\n";
					ofsPred << "    \"isPathJoint\": " << (vp.isPathJoint ? "true" : "false") << ",\n";
					ofsPred << "    \"moveType\": " << vp.moveType << ",\n";
					ofsPred << "    \"partRotateAngle\": " << vp.partRotateAngle << ",\n";
					ofsPred << "    \"partTransform\": " << mat4ToJsonPred(vp.partTransform) << "\n";
					ofsPred << "  }";
					if (i + 1 < predictedDump.size()) ofsPred << ",";
					ofsPred << "\n";
				}
				ofsPred << "]\n";
			}
   		}

		// ==================================================================
		// JBI 离线程序导出（覆盖版）：使用 applyMeasLinePathPredict 处理后的路径
		//
		// 上方 643-659 行的导出块用的是原始未处理视点（allSurfacePath +
		// combinedEdgePath）。aaa 中各转台角度的 it.second 已在前面经
		// applyMeasLinePathPredict 完成直线预测 / RRT 拼接 / MOVL 安全降级，
		// 这里把所有角度的处理后路径按转台角度升序（std::map 自然顺序）拼接，
		// 重新导出到同一文件，覆盖上方的原始路径版本。（新增）
		// ==================================================================
		{
			// 拼接所有转台角度经 applyMeasLinePathPredict 处理后的路径点（新增）
			std::vector<robot_planner::ViewPoint> predictedAllPath;
			for (auto& it : aaa)
				predictedAllPath.insert(predictedAllPath.end(),
					it.second.begin(), it.second.end());

			if (!predictedAllPath.empty()) {
				robot_planner::JBIExporter exporter(cfg);
				exporter.setLinearSpeed(100);
				exporter.setJointSpeed(5);
				exporter.setToolNumber(0);

				robot_planner::PlanResult result;
				result.success = true;
				result.waypoints = insertMovlToMovjTransitions(predictedAllPath);
   				exporter.exportToJBI(result, "D:/output/JBI", "YASKAWA");
  			}
		}

		// ==================================================================
		// 转台信息删除后处理：把上方刚导出的含转台(EC/外部轴)的
		// YASKAWA.JBI 转成无转台版本 YASKAWA_noEC.JBI。
		// 逻辑见 RemoveTurntableEC.cpp（移植自 remove_turntable_EC.m）。（新增）
		// ==================================================================
		removeTurntableEC(
			"D:/output/JBI/YASKAWA.JBI",
			"D:/output/JBI/YASKAWA_noEC_C++.JBI");

		// ==================================================================
		// 去重：从不可达集中移除已在可达集中出现的点
		// 按 name 比较——阶段2 与阶段3 的同名点 x/y/z 不同（阶段3 是 45°
		// 旋转后的坐标），不能直接用 MeasurePointPoseSet::operator< 判等。
		// ==================================================================
		std::set<std::string> measured_names;
		for (auto const& [key, vec] : measure_point_result) {
			for (const auto& point : vec) {
				measured_names.insert(point.name);
			}
		}

		for (auto& [key, vec] : un_measure_point_result)
		{
			vec.erase(
				std::remove_if(vec.begin(), vec.end(), [&](const MeasurePointPoseSet& p) {
					return measured_names.count(p.name) > 0;
					}),
				vec.end()
			);

			// 将最后一次测试的 shift/angle 写入不可达点的配置
			for (auto& ps : vec)
			{
				double lastShift = 0.0, lastAngle = 0.0;
				auto itSA = final_unreach_shift_angle.find(ps.name);
				if (itSA != final_unreach_shift_angle.end())
				{
					lastShift = itSA->second.first;
					lastAngle = itSA->second.second;
				}
				for (auto& cfg : ps.configurations)
				{
					cfg.joints.slider = lastShift;
					cfg.joints.turnTable = lastAngle;
				}
			}
		}

		// ==================================================================
		// 同名去重：同一测点若在多轮转台角度中都不可达，会被多次追加到
		// un_measure_point_result。按 name 去重，优先保留 configurations
		// 非空（有姿态解但因其它原因被判不可达）的条目。
		// ==================================================================
		for (auto& [key, vec] : un_measure_point_result) {
			std::stable_sort(vec.begin(), vec.end(),
				[](const MeasurePointPoseSet& a, const MeasurePointPoseSet& b) {
					if (a.name != b.name) return a.name < b.name;
					return !a.configurations.empty() && b.configurations.empty();
				});
			vec.erase(std::unique(vec.begin(), vec.end(),
				[](const MeasurePointPoseSet& a, const MeasurePointPoseSet& b) {
					return a.name == b.name;
				}),
				vec.end());
		}

		// ==================================================================
		// 排序：有解的姿态放前面，无解的放后面
		// ==================================================================
		for (auto& [key, pose_sets] : un_measure_point_result) {
			std::stable_partition(pose_sets.begin(), pose_sets.end(),
				[](const MeasurePointPoseSet& mp) {
					return !mp.configurations.empty();
				});
		}

		// ==================================================================
		// 更新场景
		// ==================================================================
		SceneDocument::instance().updateReachMeasurePathPoint(measure_point_result);
		SceneDocument::instance().updateUnReachMeasurePathPoint(un_measure_point_result);

		MessageManager::instance()->log(LogLevel::LOG_INFO,
			"完成机器人：" + QString::fromStdString(robotNmae) + "姿态求解！！！");
	}
}

int main(int argc, char* argv[])
{
	qRegisterMetaType<LogLevel>("LogLevel");
	QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
	QApplication app(argc, argv);
	MainWindow w;

	CollisionTestFunction collision_test = &RobotKinematicsCollisionInterface::checkCollision;
	RobotFKFunction robot_forward_solution_fn = &RobotKinematicsCollisionInterface::forwardSolution;
	RobotIKFunction robot_inverse_solution_fn = &RobotKinematicsCollisionInterface::inverseSolution;
	CylinderCollisionTestFunction cylinderCollisionTestFunction = &RobotKinematicsCollisionInterface::cylinderCollisionTestFunction;
	KinematicsApi::instance().bind(collision_test, cylinderCollisionTestFunction, robot_forward_solution_fn,
		robot_inverse_solution_fn, &RobotKinematicsCollisionInterface::looseAngleIkInverseSolution,
		&RobotKinematicsCollisionInterface::setCollisionTestSafeDistance);
	w.show();
	//我要获得线程
	QObject::connect(&w, &MainWindow::start, [&w]()
		{
			auto* watcher = new QFutureWatcher<void>(&w);
			QObject::connect(watcher, &QFutureWatcher<void>::finished,
				watcher, [watcher]()
				{
					OperateTreeManager::instance().displayMeasurePathPoints(
						SceneDocument::instance().getMeasurePathPoint());
					watcher->deleteLater();
				});
			watcher->setFuture(QtConcurrent::run(execute));
		});
	return app.exec();
}
