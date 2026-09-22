#include "RemoveTurntableEC.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <vector>

namespace {

// 若名字(小写)不以 _noec 结尾才追加 _noEC，避免重复（对齐 addNoEcSuffix）
std::string addNoEcSuffix(const std::string& name)
{
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower.size() >= 5 && lower.compare(lower.size() - 5, 5, "_noec") == 0)
        return name;
    return name + "_noEC";
}

// 从一个路径中取出文件基名(去掉目录与扩展名)，作为找不到 //NAME 时的回退程序名
std::string baseNameNoExt(const std::string& path)
{
    size_t slash = path.find_last_of("/\\");
    std::string fileName = (slash == std::string::npos) ? path : path.substr(slash + 1);
    size_t dot = fileName.find_last_of('.');
    if (dot != std::string::npos)
        fileName = fileName.substr(0, dot);
    return fileName;
}

// 去掉字符串首尾的空白
std::string trim(const std::string& s)
{
    size_t b = s.find_first_not_of(" \t");
    if (b == std::string::npos)
        return std::string();
    size_t e = s.find_last_not_of(" \t");
    return s.substr(b, e - b + 1);
}

} // namespace

bool removeTurntableEC(const std::string& inFile, const std::string& outFile)
{
    // 二进制读入全部字节，不做编码转换（对齐 readBytes）
    std::ifstream ifs(inFile, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "[remove_turntable_EC] 无法打开输入文件：" << inFile << std::endl;
        return false;
    }
    std::string rawText((std::istreambuf_iterator<char>(ifs)),
        std::istreambuf_iterator<char>());
    ifs.close();

    // EOL 探测（对齐 sourceEol）
    std::string sourceEol;
    if (rawText.find("\r\n") != std::string::npos)    sourceEol = "\r\n";
    else if (rawText.find('\r') != std::string::npos) sourceEol = "\r";
    else                                               sourceEol = "\n";

    // 统一规整成 \n，并记录末尾是否有换行（对齐 hasFinalEol）
    std::string text = std::regex_replace(rawText, std::regex("\r\n|\r|\n"), "\n");
    const bool hasFinalEol = !text.empty() && text.back() == '\n';

    // 按 \n 切分（getline 在末尾有换行时不会多出空行，行为同 MATLAB 去末尾）
    std::vector<std::string> lines;
    {
        std::istringstream ss(text);
        std::string cur;
        while (std::getline(ss, cur)) lines.push_back(cur);
    }

    // 先取程序名：第一行 //NAME（对齐 getProgramName），找不到则用输出文件基名
    const std::regex reNameGet(R"(^//NAME\s+(.+?)\s*$)");
    std::string newProgramName = addNoEcSuffix(baseNameNoExt(outFile));
    for (const auto& l : lines) {
        std::smatch m;
        if (std::regex_match(l, m, reNameGet)) {
            newProgramName = addNoEcSuffix(m[1].str());
            break;
        }
    }

    const std::regex reEcPos(R"(^EC\d{5}=)");
    const std::regex reName(R"(^(//NAME\s+)(.+?)\s*$)");
    const std::regex reNpos(R"(^(///NPOS\s+)([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,\s]+)(.*)$)");
    const std::regex reGroup(R"(^(///GROUP\d+\s+)(.*)$)");
    const std::regex reEcRef(R"(\sEC\d{5}(?=\s|$))");
    const std::regex reSt(R"(^ST\d+$)");

    std::vector<std::string> outLines;
    int removedEcPositionLines = 0, removedEcMotionRefs = 0;
    int updatedGroupLines = 0, updatedNposLines = 0, updatedNameLines = 0;

    for (auto& line : lines) {
        // 删除整行 EC 位置变量行
        if (std::regex_search(line, reEcPos)) { ++removedEcPositionLines; continue; }

        std::smatch m;
        // //NAME 行 → 新程序名
        if (std::regex_match(line, m, reName)) {
            line = m[1].str() + newProgramName;
            ++updatedNameLines;
        }
        // ///NPOS 行 → 第3字段(EC点数)置0
        if (std::regex_match(line, m, reNpos)) {
            line = m[1].str() + m[2].str() + "," + m[3].str() + ",0," +
                m[5].str() + "," + m[6].str() + "," + m[7].str() + m[8].str();
            ++updatedNposLines;
        }
        // ///GROUP 行 → 去掉所有 ST\d+ 令牌
        if (std::regex_match(line, m, reGroup)) {
            std::vector<std::string> parts, keep;
            std::istringstream ps(m[2].str());
            std::string part;
            while (std::getline(ps, part, ',')) parts.push_back(trim(part));
            for (const auto& p : parts)
                if (!std::regex_match(p, reSt)) keep.push_back(p);
            if (keep.size() != parts.size()) {
                std::string joined;
                for (size_t k = 0; k < keep.size(); ++k) {
                    if (k) joined += ",";
                    joined += keep[k];
                }
                line = m[1].str() + joined;
                ++updatedGroupLines;
            }
        }
        // 删除运动行中的 EC 引用（lookahead 不消费尾部空白，正确处理相邻引用）
        auto refBegin = std::sregex_iterator(line.begin(), line.end(), reEcRef);
        removedEcMotionRefs += static_cast<int>(
            std::distance(refBegin, std::sregex_iterator()));
        line = std::regex_replace(line, reEcRef, "");

        outLines.push_back(line);
    }

    // 用 \n 拼回（对齐 strjoin），再据 hasFinalEol 决定结尾换行
    std::string outText;
    for (size_t k = 0; k < outLines.size(); ++k) {
        if (k) outText += "\n";
        outText += outLines[k];
    }
    if (hasFinalEol) outText += "\n";
    // 还原原 EOL
    if (sourceEol != "\n")
        outText = std::regex_replace(outText, std::regex("\n"), sourceEol);

    std::ofstream ofs(outFile, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
        std::cerr << "[remove_turntable_EC] 无法创建输出文件：" << outFile << std::endl;
        return false;
    }
    ofs.write(outText.data(), static_cast<std::streamsize>(outText.size()));
    ofs.close();

    std::cout << "[remove_turntable_EC] 已生成：" << outFile << std::endl;
    std::cout << "[remove_turntable_EC] 更新 //NAME 行：" << updatedNameLines << std::endl;
    std::cout << "[remove_turntable_EC] 更新 ///NPOS 行：" << updatedNposLines << std::endl;
    std::cout << "[remove_turntable_EC] 更新 ///GROUP 行：" << updatedGroupLines << std::endl;
    std::cout << "[remove_turntable_EC] 删除 EC 位置行：" << removedEcPositionLines << std::endl;
    std::cout << "[remove_turntable_EC] 删除 EC 运动引用：" << removedEcMotionRefs << std::endl;
    return true;
}
