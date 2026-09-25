#ifndef TIPTILT_LOGGER_H
#define TIPTILT_LOGGER_H

#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

enum class LogLevel {
    Info,
    Warning,
    Error,
    Debug
};

class Logger {
public:
    static Logger& instance();

    void initializeSession(int mode);
    void log(LogLevel level, const std::string& message);
    void writeCsv(const std::string& fileName, const std::string& line, bool mirrorToCurrentDirectory = false);
    std::string sessionDirectory() const;

private:
    Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream& csvStreamLocked(const std::string& fileName, bool mirrorToCurrentDirectory);
    static std::string levelToString(LogLevel level);
    static std::string timestampForLog();
    static std::string timestampForDirectory();

    mutable std::mutex mutex_;
    std::string sessionDirectory_;
    bool initialized_ = false;
    std::ofstream appLog_;
    std::unordered_map<std::string, std::ofstream> csvFiles_;
    std::unordered_map<std::string, std::ofstream> mirroredCsvFiles_;
};

void log_info(const std::string& message);
void log_warning(const std::string& message);
void log_error(const std::string& message);
void log_debug(const std::string& message);

#endif
