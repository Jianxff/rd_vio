#include <memory>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <rdvio/types.h>
#include <rdvio/util/debug.h>

namespace rdvio {

static spdlog::logger *logger() {
    static std::unique_ptr<spdlog::logger> s_logger;
    if (!s_logger) {
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::trace);
        console_sink->set_pattern("%Y-%m-%d %T.%e - [RDVIO][%^%l%$] %v");
        s_logger = std::make_unique<spdlog::logger>(
            "rdvio", spdlog::sinks_init_list{console_sink});
        s_logger->set_level(spdlog::level::trace);
    }
    return s_logger.get();
}

void log_message(LogLevel level, const char *format, ...) {
    static std::vector<char> msgbuf(512);
    va_list vargs1, vargs2;
    va_start(vargs1, format);
    va_copy(vargs2, vargs1);
    int len = vsnprintf(nullptr, 0, format, vargs1);
    va_end(vargs1);
    if (msgbuf.size() < len + 1) {
        msgbuf.resize(len + 1);
    }
    vsnprintf(msgbuf.data(), msgbuf.size(), format, vargs2);
    va_end(vargs2);
    spdlog::level::level_enum lvl;
    switch (level) {
    case LOG_DEBUG:
        lvl = spdlog::level::debug;
        break;
    case LOG_INFO:
        lvl = spdlog::level::info;
        break;
    case LOG_NOTICE:
        lvl = spdlog::level::info;
        break;
    case LOG_WARNING:
        lvl = spdlog::level::warn;
        break;
    case LOG_ERR:
        lvl = spdlog::level::err;
        break;
    case LOG_CRIT:
        lvl = spdlog::level::critical;
        break;
    case LOG_ALERT:
        lvl = spdlog::level::critical;
        break;
    case LOG_EMERG:
        lvl = spdlog::level::critical;
        break;
    }
    logger()->log(lvl, msgbuf.data());
}

} // namespace rdvio
