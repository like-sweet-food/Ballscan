#include "JBIExporter.h"

#include <direct.h>

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <sstream>
#include <ctime>

namespace robot_planner {

JBIExporter::JBIExporter(const RobotConfig& cfg)
    : m_cfg(cfg)
{
}

void JBIExporter::setLinearSpeed(int cmPerMin)  { m_linearSpeed = cmPerMin; }
void JBIExporter::setJointSpeed(int percent)    { m_jointSpeed = percent; }
void JBIExporter::setToolNumber(int t)          { m_toolNumber = t; }
void JBIExporter::setPulseRatios(const YaskawaPulseRatios& ratios) { m_pulseRatios = ratios; }

bool JBIExporter::exportToJBI(const PlanResult& result,
                              const std::string& outputDir,
                              const std::string& jobName) const
{
    if (!result.success || result.waypoints.empty()) {
        std::cerr << "[JBIExporter] 导出失败：规划结果为空或规划未成功" << std::endl;
        return false;
    }

    // 对应 MATLAB data = [J1..J6, turntable, slider, motiontype]
    std::vector<std::vector<double> > data;
    data.reserve(result.waypoints.size());

    std::cout << "[DEBUG][JBIExporter] m_cfg.slidePos = " << m_cfg.slidePos
              << "（写入每行 row[7]）" << std::endl;
    for (const auto& vp : result.waypoints) {
        std::vector<double> row(9, 0.0);
        for (int i = 0; i < 6; ++i) {
            row[i] = vp.joints[i];
        }
        row[6] = vp.partRotateAngle;
        row[7] = m_cfg.slidePos;
        row[8] = static_cast<double>(vp.moveType);
        data.push_back(row);
    }

    return exportMatrixToJBI(data, outputDir, jobName);
}

bool JBIExporter::exportCsvToJBI(const std::string& csvPath,
                                 const std::string& outputDir,
                                 const std::string& jobName) const
{
    const std::vector<std::vector<double> > data = readCsvMatrix(csvPath);
    if (data.empty()) {
        std::cerr << "[JBIExporter] CSV 读取失败或数据为空：" << csvPath << std::endl;
        return false;
    }
    return exportMatrixToJBI(data, outputDir, jobName);
}

bool JBIExporter::exportMatrixToJBI(
    const std::vector<std::vector<double> >& data,
    const std::string& outputDir,
    const std::string& jobName) const
{
    if (data.empty()) {
        std::cerr << "[JBIExporter] 数据为空" << std::endl;
        return false;
    }

    const size_t cols = data.front().size();
    if (cols != 7 && cols != 8 && cols != 9) {
        std::cerr << "[JBIExporter] 数据格式错误：需要 n x 7、n x 8 或 n x 9 矩阵" << std::endl;
        return false;
    }
    for (size_t i = 0; i < data.size(); ++i) {
        if (data[i].size() != cols) {
            std::cerr << "[JBIExporter] 数据格式错误：各行列数不一致" << std::endl;
            return false;
        }
    }

    const bool has8Axis = (cols == 8);
    const bool has9Axis = (cols == 9);
    std::cout << "[DEBUG][JBIExporter] data 列数 cols = " << cols
              << "，has8Axis=" << has8Axis << "，has9Axis=" << has9Axis << std::endl;
    if (!has8Axis && !has9Axis)
        std::cout << "[DEBUG][JBIExporter] 警告：未识别为外部轴模式，slider 列将被忽略！" << std::endl;

    std::vector<std::vector<double> > jointData = data;
    if (has8Axis || has9Axis) {
        jointData = insertExtAxisSteps(jointData);
    }

    const size_t n = jointData.size();

    std::vector<std::vector<double> > transformedData = jointData;

    std::vector<std::vector<double> > pulsejointData = transformedData;
    for (size_t i = 0; i < n; ++i) {
        pulsejointData[i][1] = transformedData[i][1] - 90.0;    // L: J2_moto - 90
        pulsejointData[i][2] = transformedData[i][2] - (-90.0); // U: J3_moto + 90
    }

    if (!ensureDirectory(outputDir)) {
        std::cerr << "[JBIExporter] 无法创建输出目录：" << outputDir << std::endl;
        return false;
    }

    std::string filePath = outputDir + "\\" + jobName + ".JBI";
    // 对齐 MATLAB fopen(..., 'n', 'UTF-8')：
    // 这里按二进制方式直接写入 UTF-8 字节，避免依赖本地代码页转换。
    std::ofstream file(filePath.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "[JBIExporter] 无法创建文件：" << filePath << std::endl;
        return false;
    }

    file << "/JOB\r\n";
    file << "//NAME " << jobName << "\r\n";
    file << "//POS\r\n";
    if (!has8Axis && !has9Axis) {
        file << "///NPOS " << n << ",0,0,0,0,0\r\n";
    } else if (has8Axis) {
        file << "///NPOS " << n << "," << n << ",0,0,0,0\r\n";
    } else {
        file << "///NPOS " << n << "," << n << "," << n << ",0,0,0\r\n";
    }
    file << "///TOOL " << m_toolNumber << "\r\n";
    file << "///POSTYPE PULSE\r\n";
    file << "///PULSE\r\n";

    for (size_t i = 0; i < n; ++i) {
        const std::array<long, 8> pulses = jointsToPulses(
            pulsejointData[i][0], pulsejointData[i][1], pulsejointData[i][2],
            pulsejointData[i][3], pulsejointData[i][4], pulsejointData[i][5],
            has9Axis ? pulsejointData[i][6] : 0.0,
            has8Axis ? pulsejointData[i][6] : (has9Axis ? pulsejointData[i][7] : 0.0));

        file << "C" << std::setw(5) << std::setfill('0') << i
             << "="
             << pulses[0] << "," << pulses[1] << "," << pulses[2] << ","
             << pulses[3] << "," << pulses[4] << "," << pulses[5]
             << "\r\n";
    }

    if (has8Axis) {
        for (size_t i = 0; i < n; ++i) {
            const long extPulse = static_cast<long>(std::llround(pulsejointData[i][6] * m_pulseRatios.E));
            file << "BC" << std::setw(5) << std::setfill('0') << i
                 << "=" << extPulse << "\r\n";
        }
    } else if (has9Axis) {
        for (size_t i = 0; i < n; ++i) {
            const long extPulse = static_cast<long>(std::llround(pulsejointData[i][7] * m_pulseRatios.E));
            if (i == 0)
                std::cout << "[DEBUG][JBIExporter] 第0点 slider原值=" << pulsejointData[i][7]
                          << "，E脉冲比=" << m_pulseRatios.E
                          << "，输出脉冲=" << extPulse << std::endl;
            file << "BC" << std::setw(5) << std::setfill('0') << i
                 << "=" << extPulse << "\r\n";
        }
        for (size_t i = 0; i < n; ++i) {
            const long extPulse = static_cast<long>(std::llround(pulsejointData[i][6] * m_pulseRatios.Z));
            file << "EC" << std::setw(5) << std::setfill('0') << i
                 << "=" << extPulse << "\r\n";
        }
    } else {
        std::cout << "注意：未检测到外部轴数据，跳过BC、EC行生成" << std::endl;
    }

    file << "//INST\r\n";
    file << "///DATE " << buildDateString() << "\r\n";
    file << "///ATTR SC,RW\r\n";
    if (!has8Axis && !has9Axis) {
        file << "///GROUP1 RB1\r\n";
    } else if (has8Axis) {
        file << "///GROUP1 RB1,BS1\r\n";
    } else {
        file << "///GROUP1 RB1,BS1,ST1\r\n";
    }
    file << "NOP\r\n";

    for (size_t i = 0; i < n; ++i) {
        double motiontype_i = 0.0;
        if (cols == 9) {
            motiontype_i = jointData[i][8];
        } else if (cols == 8) {
            motiontype_i = jointData[i][7];
        } else if (cols == 7) {
            motiontype_i = jointData[i][6];
        }

        if (motiontype_i == 0.0) {
            if (!has8Axis && !has9Axis) {
                file << "MOVJ C" << std::setw(5) << std::setfill('0') << i
                     << " VJ=" << std::fixed << std::setprecision(2) << static_cast<double>(m_jointSpeed) << "\r\n";
            } else if (has8Axis) {
                file << "MOVJ C" << std::setw(5) << std::setfill('0') << i
                     << " BC" << std::setw(5) << std::setfill('0') << i
                     << " VJ=" << std::fixed << std::setprecision(2) << static_cast<double>(m_jointSpeed) << "\r\n";
            } else {
                file << "MOVJ C" << std::setw(5) << std::setfill('0') << i
                     << " BC" << std::setw(5) << std::setfill('0') << i
                     << " EC" << std::setw(5) << std::setfill('0') << i
                     << " VJ=" << std::fixed << std::setprecision(2) << static_cast<double>(m_jointSpeed) << "\r\n";
            }
        } else if (motiontype_i == 1.0) {
            if (!has8Axis && !has9Axis) {
                file << "MOVL C" << std::setw(5) << std::setfill('0') << i
                     << " V=" << std::fixed << std::setprecision(1) << static_cast<double>(m_linearSpeed) << "\r\n";
            } else if (has8Axis) {
                file << "MOVL C" << std::setw(5) << std::setfill('0') << i
                     << " BC" << std::setw(5) << std::setfill('0') << i
                     << " V=" << std::fixed << std::setprecision(1) << static_cast<double>(m_linearSpeed) << "\r\n";
            } else {
                file << "MOVL C" << std::setw(5) << std::setfill('0') << i
                     << " BC" << std::setw(5) << std::setfill('0') << i
                     << " EC" << std::setw(5) << std::setfill('0') << i
                     << " V=" << std::fixed << std::setprecision(1) << static_cast<double>(m_linearSpeed) << "\r\n";
            }
        } else {
            std::cerr << "[JBIExporter] 第 " << (i + 1) << " 行的运动类型非法（应为0或1），已跳过。" << std::endl;
        }
    }

    file << "END\r\n";
    file.close();

    std::cout << "[JBIExporter] JBI 文件已导出：" << filePath
              << "（共 " << n << " 个路径点）" << std::endl;
    return true;
}

std::array<long, 8> JBIExporter::jointsToPulses(
    double j1, double j2, double j3,
    double j4, double j5, double j6,
    double turnTableDeg,
    double sliderMm) const
{
    return {
        static_cast<long>(std::round(j1          * m_pulseRatios.S)),
        static_cast<long>(std::round(j2          * m_pulseRatios.L)),
        static_cast<long>(std::round(j3          * m_pulseRatios.U)),
        static_cast<long>(std::round(j4          * m_pulseRatios.R)),
        static_cast<long>(std::round(j5          * m_pulseRatios.B)),
        static_cast<long>(std::round(j6          * m_pulseRatios.T)),
        static_cast<long>(std::round(turnTableDeg * m_pulseRatios.Z)),
        static_cast<long>(std::round(sliderMm    * m_pulseRatios.E))
    };
}

std::vector<std::vector<double> > JBIExporter::insertExtAxisSteps(
    const std::vector<std::vector<double> >& jointData) const
{
    if (jointData.empty()) {
        return jointData;
    }

    const size_t cols = jointData.front().size();
    if (cols < 7) {
        return jointData;
    }

    std::vector<std::vector<double> > newData;
    newData.push_back(jointData[0]);

    for (size_t i = 0; i + 1 < jointData.size(); ++i) {
        const std::vector<double>& curr = jointData[i];
        const std::vector<double>& next = jointData[i + 1];

        newData.push_back(next);
        std::vector<double> temp = curr;

        if (cols >= 8) {
            if (curr[7] != next[7]) {
                std::vector<double> tmp = curr;
                tmp[7] = next[7];
                newData.back() = tmp;
                newData.push_back(next);
                temp = tmp;
            }

            if (curr[6] != next[6]) {
                std::vector<double> tmp = temp;
                tmp[6] = next[6];
                if (tmp != next) {
                    newData.back() = tmp;
                    newData.push_back(next);
                }
            }
        } else {
            if (curr[6] != next[6]) {
                std::vector<double> tmp = curr;
                tmp[6] = next[6];
                newData.back() = tmp;
                newData.push_back(next);
            }
        }
    }

    return newData;
}

std::vector<std::vector<double> > JBIExporter::readCsvMatrix(
    const std::string& csvPath) const
{
    std::ifstream file(csvPath.c_str(), std::ios::in | std::ios::binary);
    std::vector<std::vector<double> > data;

    if (!file.is_open()) {
        std::cerr << "[JBIExporter] 无法打开 CSV 文件：" << csvPath << std::endl;
        return data;
    }

    std::string line;
    size_t lineNo = 0;
    while (std::getline(file, line)) {
        ++lineNo;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }

        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            if (!cell.empty() && cell.front() == '"') {
                cell.erase(cell.begin());
            }
            if (!cell.empty() && cell.back() == '"') {
                cell.pop_back();
            }

            try {
                row.push_back(std::stod(cell));
            } catch (const std::exception&) {
                std::cerr << "[JBIExporter] CSV 第 " << lineNo
                          << " 行存在非数值字段：" << cell << std::endl;
                return std::vector<std::vector<double> >();
            }
        }

        if (!row.empty()) {
            data.push_back(row);
        }
    }

    return data;
}

bool JBIExporter::ensureDirectory(const std::string& outputDir) const
{
    if (outputDir.empty()) {
        return false;
    }
    const int rc = _mkdir(outputDir.c_str());
    return (rc == 0 || errno == EEXIST);
}

std::string JBIExporter::buildDateString() const
{
    // 对齐 MATLAB:
    // currentTime = datetime('now', 'TimeZone', 'Asia/Shanghai');
    // Asia/Shanghai 固定为 UTC+8，不依赖当前机器本地时区。
    const std::time_t now = std::time(0) + 8 * 60 * 60;
    struct tm tmBuf;
    gmtime_s(&tmBuf, &now);

    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M", &tmBuf);
    return std::string(buf);
}

} // namespace robot_planner
