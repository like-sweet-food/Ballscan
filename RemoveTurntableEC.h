#pragma once

#include <string>

// 把一个含转台(EC/外部轴)信息的 Yaskawa JBI 文件转换成无转台版本。
// 逻辑移植自 remove_turntable_EC.m：
//   - 删除 EC 位置变量行与运动行中的 EC 引用；
//   - //NAME 程序名加 _noEC 后缀；
//   - ///NPOS 第3字段(EC点数)置 0；
//   - ///GROUP 去掉所有 ST\d+ 令牌；
//   - 保留原文件的换行风格(CRLF/CR/LF)与末尾换行。
// 纯文本后处理，不做编码转换。成功返回 true，失败返回 false 并打印到 stderr。
bool removeTurntableEC(const std::string& inFile, const std::string& outFile);
