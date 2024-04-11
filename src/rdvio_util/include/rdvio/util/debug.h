#pragma once

namespace rdvio {

enum LogLevel {
    LOG_DEBUG = -1,  /**< debug message                      **/
    LOG_INFO = 0,    /**< informational message              **/
    LOG_NOTICE = 1,  /**< normal, but significant, condition **/
    LOG_WARNING = 2, /**< warning conditions                 **/
    LOG_ERR = 3,     /**< error conditions                   **/
    LOG_CRIT = 4,    /**< critical conditions                **/
    LOG_ALERT = 5,   /**< action must be taken immediately   **/
    LOG_EMERG = 6    /**< system is unusable                 **/
};

void log_message(LogLevel level, const char *format, ...);

} // namespace rdvio

#define log_emergency(...) log_message(LOG_EMERG, __VA_ARGS__)
#define log_alert(...) log_message(LOG_ALERT, __VA_ARGS__)
#define log_critical(...) log_message(LOG_CRIT, __VA_ARGS__)
#define log_error(...) log_message(LOG_ERR, __VA_ARGS__)
#define log_warning(...) log_message(LOG_WARNING, __VA_ARGS__)
#define log_notice(...) log_message(LOG_NOTICE, __VA_ARGS__)
#define log_info(...) log_message(LOG_INFO, __VA_ARGS__)

#ifdef BUILD_LOG_DEBUG
#define log_debug(...) log_message(LOG_DEBUG, __VA_ARGS__)
#define runtime_assert(condition, message)                                     \
    do {                                                                       \
        if (!(condition)) {                                                    \
            log_error("Assertion failed at " __FILE__                          \
                      ":%d : %s\nWhen testing condition:\n    %s",             \
                      __LINE__, message, #condition);                          \
            abort();                                                           \
        }                                                                      \
    } while (0)
#else
#define log_debug(...)
#define runtime_assert(...)
#endif

