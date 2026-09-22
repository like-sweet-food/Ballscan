#pragma once

#include <array>
#include <string>
#include <vector>

#include "RobotConfig.h"
#include "Types.h"

namespace robot_planner {

struct YaskawaPulseRatios {
    double S = 1413.522;
    double L = -1604.267;
    double U = 1994.364;
    double R = -930.91;
    double B = 986.08;
    double T = -536.604;
    double Z = 11.378;
    double E = 173.8466;
};

class JBIExporter {
public:
    explicit JBIExporter(const RobotConfig& cfg);

    bool exportToJBI(const PlanResult& result,
                     const std::string& outputDir,
                     const std::string& jobName) const;

    bool exportCsvToJBI(const std::string& csvPath,
                        const std::string& outputDir,
                        const std::string& jobName) const;

    bool exportMatrixToJBI(const std::vector<std::vector<double> >& data,
                           const std::string& outputDir,
                           const std::string& jobName) const;

    void setLinearSpeed(int cmPerMin);
    void setJointSpeed(int percent);
    void setToolNumber(int t);
    void setPulseRatios(const YaskawaPulseRatios& ratios);

private:
    std::vector<std::vector<double> > insertExtAxisSteps(
        const std::vector<std::vector<double> >& jointData) const;

    std::vector<std::vector<double> > readCsvMatrix(
        const std::string& csvPath) const;

    std::array<long, 8> jointsToPulses(
        double j1, double j2, double j3,
        double j4, double j5, double j6,
        double turnTableDeg, double sliderMm) const;

    bool ensureDirectory(const std::string& outputDir) const;
    std::string buildDateString() const;

private:
    const RobotConfig& m_cfg;
    YaskawaPulseRatios m_pulseRatios;
    int m_linearSpeed = 100;
    int m_jointSpeed = 5;
    int m_toolNumber = 0;
};

} // namespace robot_planner
