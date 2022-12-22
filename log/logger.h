#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/logger.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/bundled/printf.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/msvc_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sstream>
#include <atomic>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <pthread.h>
#include <fstream>
#include <iostream>

void split(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = "/")
{
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}

class BZLLogger final {
public:
	// let logger like stream
	struct LogStream : public std::ostringstream
	{
	public:
		LogStream(const spdlog::source_loc& loc, spdlog::level::level_enum lvl, const char* prefix)
			: loc_(loc)
			, lvl_(lvl)
			, prefix_(prefix)
		{
		}

		~LogStream()
		{
			flush();
		}

		void flush()
		{
			BZLLogger::Get().Log(loc_, lvl_, (prefix_ + str()).c_str());
		}

	private:
		spdlog::source_loc loc_;
		spdlog::level::level_enum lvl_ = spdlog::level::info;
		const char* prefix_;
	};

public:
	static BZLLogger& Get() {
		static BZLLogger logger;
		return logger;
	}

	bool Init(const char* log_file_path) {
		if (is_inited_) return true;
		try
		{
			// check log path and try to create log directory
			std::vector<std::string> v;
			split(std::string(log_file_path), v, "/");
			const std::string file_name = v.back();
			v.pop_back();
			std::string parent_path(getenv("HOME") + std::string("/"));
			for(std::string &s : v) {
			    parent_path += s + "/";
			    if(-1 == access(parent_path.c_str(), F_OK))
			    	mkdir(parent_path.c_str(), 0755);
			}
			/*
			std::fstream fs;
			fs.open(parent_path + file_name);
			if(!fs.is_open()){
				fs.clear();
				fs.open(parent_path + file_name, std::ios::out | std::ios::app);
				fs.close();
			}
			*/

			// initialize spdlog
			constexpr std::size_t log_buffer_size = 32 * 1024; // 32kb
			// constexpr std::size_t max_file_size = 50 * 1024 * 1024; // 50mb
			spdlog::init_thread_pool(log_buffer_size, std::thread::hardware_concurrency() / 2);
			std::vector<spdlog::sink_ptr> sinks;
			auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(parent_path + file_name, 0, 2);
			sinks.push_back(daily_sink);

#ifdef _LOG_CONSOLE
			auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
			sinks.push_back(console_sink);
#endif
			spdlog::set_default_logger(std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end()));
			spdlog::set_pattern("%s(%#): [%L %D %T.%e %P %t %!] %v");
			spdlog::flush_on(spdlog::level::warn);
			spdlog::set_level(log_level_);
		}
		catch (std::exception_ptr &e)
		{
			assert(false);
			return false;
		}
		is_inited_ = true;
		return true;
	}

	void ShutDown() { spdlog::shutdown(); }

	template <typename... Args>
	void Log(const spdlog::source_loc& loc, spdlog::level::level_enum lvl, const char* fmt, const Args &... args)
	{
		spdlog::log(loc, lvl, fmt, args...);
	}

	template <typename... Args>
	void Printf(const spdlog::source_loc& loc, spdlog::level::level_enum lvl, const char* fmt, const Args &... args)
	{
		spdlog::log(loc, lvl, fmt::sprintf(fmt, args...).c_str());
	}

	spdlog::level::level_enum level() {
		return log_level_;
	}

	void SetLevel(spdlog::level::level_enum lvl) {
		log_level_ = lvl;
		spdlog::set_level(lvl);

	}

	void SetFlushOn(spdlog::level::level_enum lvl) {
		spdlog::flush_on(lvl);
	}

	static const char* GetShortName(std::string path) {
		if (path.empty())
			return path.data();

		size_t pos = path.find_last_of("/\\");
		return path.data() + ((pos == path.npos) ? 0 : pos + 1);
	}

private:
	BZLLogger() = default;
	~BZLLogger() = default;

	BZLLogger(const BZLLogger&) = delete;
	void operator=(const BZLLogger&) = delete;

private:
	std::atomic<bool> is_inited_ = ATOMIC_VAR_INIT(false);
	spdlog::level::level_enum log_level_ = spdlog::level::trace;
};

// // got short filename(exlude file directory)
#define __FILENAME__ (BZLLogger::GetShortName(__FILE__))

// // use like stream , e.g. STM_WARN() << "warn log: " << 1;
#define STM_TRACE() BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::trace, "")
#define STM_DEBUG() BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::debug, "")
#define STM_INFO()	BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::info, "")
#define STM_WARN()	BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::warn, "")
#define STM_ERROR() BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::err, "")
#define STM_FATAL() BZLLogger::LogStream({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::critical, "")

// // use spd lib, e.g. LOG_WARN("warn log, {1}, {1}, {2}", 1, 2);
#define LOG_TRACE(msg,...) { if (logger::get().level() == spdlog::level::trace) spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::trace, msg, ##__VA_ARGS__); };
#define LOG_DEBUG(msg,...) spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::debug, msg, ##__VA_ARGS__)
#define LOG_INFO(msg,...)  spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::info, msg, ##__VA_ARGS__)
#define LOG_WARN(msg,...)  spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::warn, msg, ##__VA_ARGS__)
#define LOG_ERROR(msg,...) spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::err, msg, ##__VA_ARGS__)
#define LOG_FATAL(msg,...) spdlog::log({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::critical, msg, ##__VA_ARGS__)

// // use like sprintf, e.g. PRINT_WARN("warn log, %d-%d", 1, 2);
#define PRINT_TRACE(msg,...) BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::trace, msg, ##__VA_ARGS__);
#define PRINT_DEBUG(msg,...) BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::debug, msg, ##__VA_ARGS__);
#define PRINT_INFO(msg,...)  BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::info, msg, ##__VA_ARGS__);
#define PRINT_WARN(msg,...)  BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::warn, msg, ##__VA_ARGS__);
#define PRINT_ERROR(msg,...) BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::err, msg, ##__VA_ARGS__);
#define PRINT_FATAL(msg,...) BZLLogger::get().printf({__FILENAME__, __LINE__, __FUNCTION__}, spdlog::level::critical, msg, ##__VA_ARGS__);