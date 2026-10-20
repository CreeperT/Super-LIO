/**
 * @file csv_dump.h
 * @brief 调试 CSV 落盘助手 —— 路径、开关、单文件轮转统一由 lio.dump.* 控制。
 *
 * 背景：这些 CSV 原来是硬编码路径（`/tmp/m20_*.csv` 与
 * `/home/creepert/Super-LIO/src/super_lio/logs/*.csv`）。后者在 Jetson 上不存在
 * （用户是 ysc）⇒ 一直静默失败；前者每次重定位都无限增长。
 *
 * 两个入口（都返回 nullptr 表示"这一路关掉了"）：
 *   1) FILE*   —— 沿用原有 std::fprintf 的写法：
 *        if (auto* f = dump_file("m20_upd", g_dump_upd, "t,dx0,...")) std::fprintf(f, ...);
 *   2) ostream —— 沿用原有 `<<` 的写法：
 *        if (auto* os = dump_stream("solver_time", g_dump_solver_time)) *os << ... << "\n";
 *
 * 行为：
 *   · 目录不存在自动创建（默认 g_dump_dir，空则 <源码根>/logs）；
 *   · 打开失败只登一次 WARNING（受 lio.log.* 控制），之后返回 nullptr，不影响解算；
 *   · 文件为空时写表头 header（可省略）；
 *   · 单文件超过 lio.dump.max_file_mb 自动轮转：<name>.csv → .1 → .2（保留 2 份）。
 */

#ifndef CSV_DUMP_H_
#define CSV_DUMP_H_

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <unordered_map>

#include <glog/logging.h>

#include "lio/params.h"

namespace LI2Sup {

namespace dump_detail {

inline std::string dump_path(const char* name) {
  const std::string dir = g_dump_dir.empty() ? (g_root_dir + "/logs") : g_dump_dir;
  return dir + "/" + name + ".csv";
}

inline bool over_limit(const std::string& path) {
  if (g_dump_max_mb <= 0.0) return false;
  std::error_code ec;
  const auto sz = std::filesystem::file_size(path, ec);
  if (ec) return false;
  return sz > static_cast<std::uintmax_t>(g_dump_max_mb * 1024.0 * 1024.0);
}

// 流版必须用 tellp()：文件实际大小要等缓冲区 flush 才更新，用 file_size 会漏轮转
inline bool over_limit(std::ofstream& os) {
  if (g_dump_max_mb <= 0.0) return false;
  const auto pos = os.tellp();
  if (pos < 0) return false;
  return static_cast<double>(pos) > g_dump_max_mb * 1024.0 * 1024.0;
}

inline void rotate(const std::string& path) {
  std::error_code ec;
  std::filesystem::remove(path + ".2", ec);
  std::filesystem::rename(path + ".1", path + ".2", ec);
  std::filesystem::rename(path, path + ".1", ec);
  if (g_log_enable && g_log_warn) LOG(WARNING) << "[Dump] rotated " << path;
}

inline bool ensure_dir(const std::string& path) {
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
  return !ec || std::filesystem::exists(std::filesystem::path(path).parent_path(), ec);
}

inline bool empty_file(const std::string& path) {
  std::error_code ec;
  const auto sz = std::filesystem::file_size(path, ec);
  return ec || sz == 0;
}

}  // namespace dump_detail

/// stdio 版：返回已打开的 FILE*（"a" 模式），超限自动轮转。header 仅在新文件上写。
inline std::FILE* dump_file(const char* name, bool enabled, const char* header = nullptr) {
  if (!enabled || !g_dump_enable) return nullptr;

  static std::unordered_map<std::string, std::FILE*> files;
  const std::string path = dump_detail::dump_path(name);
  std::FILE*& f = files[name];

  if (f != nullptr && dump_detail::over_limit(path)) {
    std::fclose(f);
    f = nullptr;
    dump_detail::rotate(path);
  }

  if (f == nullptr) {
    if (!dump_detail::ensure_dir(path)) {
      if (g_log_enable && g_log_warn) LOG(WARNING) << "[Dump] cannot create dir for " << path;
      return nullptr;
    }
    const bool fresh = dump_detail::empty_file(path);
    f = std::fopen(path.c_str(), "a");
    if (f == nullptr) {
      if (g_log_enable && g_log_warn) LOG(WARNING) << "[Dump] cannot open " << path;
      return nullptr;
    }
    if (fresh && header != nullptr) {
      std::fprintf(f, "%s\n", header);
      std::fflush(f);
    }
  }
  return f;
}

/// ostream 版：返回已打开的流（追加写），超限自动轮转。header 仅在新文件上写。
inline std::ostream* dump_stream(const char* name, bool enabled, const char* header = nullptr) {
  if (!enabled || !g_dump_enable) return nullptr;

  static std::unordered_map<std::string, std::ofstream> files;
  const std::string path = dump_detail::dump_path(name);
  auto it = files.find(name);

  if (it != files.end() && it->second.is_open() && dump_detail::over_limit(it->second)) {
    it->second.close();
    dump_detail::rotate(path);
    it->second.open(path, std::ios::trunc);
    if (header != nullptr) it->second << header << "\n";
  }

  if (it == files.end() || !it->second.is_open()) {
    if (!dump_detail::ensure_dir(path)) {
      if (g_log_enable && g_log_warn) LOG(WARNING) << "[Dump] cannot create dir for " << path;
      return nullptr;
    }
    const bool fresh = dump_detail::empty_file(path);
    std::ofstream ofs(path, std::ios::app);
    if (!ofs) {
      if (g_log_enable && g_log_warn) LOG(WARNING) << "[Dump] cannot open " << path;
      return nullptr;
    }
    if (fresh && header != nullptr) ofs << header << "\n";
    it = files.insert_or_assign(name, std::move(ofs)).first;
  }
  return &it->second;
}

}  // namespace LI2Sup

#endif  // CSV_DUMP_H_
