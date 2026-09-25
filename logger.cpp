#include "logger.h"

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "utilheaders.h"

Logger& Logger::instance() {
    static Logger logger;
    return logger;
}

void Logger::initializeSession(int mode) {
    std::lock_guard<std::mutex> guard(mutex_);

    sessionDirectory_ = std::string("output\\") + timestampForDirectory();
    std::filesystem::create_directories(sessionDirectory_);
    std::snprintf(SAVEPATH, sizeof(SAVEPATH), "%s", sessionDirectory_.c_str());

    appLog_.close();
    csvFiles_.clear();
    mirroredCsvFiles_.clear();

    appLog_.open((sessionDirectory_ + "\\app.log").c_str(), std::ios::out | std::ios::trunc);
    initialized_ = true;

    if (mode == 1) {
        csvStreamLocked("Xlog.txt", false);
        csvStreamLocked("Ylog.txt", false);
        csvStreamLocked("XVoltageOnly.csv", true);
        csvStreamLocked("YVoltageOnly.csv", true);
    } else if (mode == 2) {
        csvStreamLocked("Shifts_Uncorrected.csv", false);
        csvStreamLocked("Shifts.csv", false);
        csvStreamLocked("Autoguider.csv", false);
        csvStreamLocked("IgnoredShift.csv", false);
        csvStreamLocked("oplog.txt", false);
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    if (!initialized_) {
        initializeSession(0);
    }
    std::lock_guard<std::mutex> guard(mutex_);

    const std::string line = timestampForLog() + " [" + levelToString(level) + "] " + message;
    std::cout << line << std::endl;
    if (appLog_.is_open()) {
        appLog_ << line << std::endl;
        appLog_.flush();
    }
}

void Logger::writeCsv(const std::string& fileName, const std::string& line, bool mirrorToCurrentDirectory) {
    if (!initialized_) {
        initializeSession(0);
    }
    std::lock_guard<std::mutex> guard(mutex_);

    std::ofstream& stream = csvStreamLocked(fileName, mirrorToCurrentDirectory);
    stream << line << std::endl;
    stream.flush();

    if (mirrorToCurrentDirectory) {
        std::ofstream& mirroredStream = mirroredCsvFiles_.at(fileName);
        mirroredStream << line << std::endl;
        mirroredStream.flush();
    }
}

std::string Logger::sessionDirectory() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return sessionDirectory_;
}

std::ofstream& Logger::csvStreamLocked(const std::string& fileName, bool mirrorToCurrentDirectory) {
    auto it = csvFiles_.find(fileName);
    if (it == csvFiles_.end()) {
        std::ofstream stream((sessionDirectory_ + "\\" + fileName).c_str(), std::ios::out | std::ios::trunc);
        it = csvFiles_.emplace(fileName, std::move(stream)).first;
    }

    if (mirrorToCurrentDirectory && mirroredCsvFiles_.find(fileName) == mirroredCsvFiles_.end()) {
        std::ofstream mirrored(fileName.c_str(), std::ios::out | std::ios::trunc);
        mirroredCsvFiles_.emplace(fileName, std::move(mirrored));
    }

    return it->second;
}

std::string Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warning:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Debug:
            return "DEBUG";
    }
    return "INFO";
}

std::string Logger::timestampForLog() {
    std::time_t now = std::time(nullptr);
    std::tm localTime = *std::localtime(&now);
    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    return stream.str();
}

std::string Logger::timestampForDirectory() {
    std::time_t now = std::time(nullptr);
    std::tm localTime = *std::localtime(&now);
    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

void log_info(const std::string& message) {
    Logger::instance().log(LogLevel::Info, message);
}

void log_warning(const std::string& message) {
    Logger::instance().log(LogLevel::Warning, message);
}

void log_error(const std::string& message) {
    Logger::instance().log(LogLevel::Error, message);
}

void log_debug(const std::string& message) {
    Logger::instance().log(LogLevel::Debug, message);
}
