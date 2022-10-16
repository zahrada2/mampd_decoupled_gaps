/**
 * File:    utils.cpp
 *
 * Date:    26.9.19
 * Author:  Jan Mikula
 * E-mail:  mikulj14@fel.cvut.cz
 *
 */

#include "utils.hpp"

using namespace cvrplib;

/// --------------------------------------------------------------------------------------------------------------------
/// Clock
/// --------------------------------------------------------------------------------------------------------------------

SimpleClock::SimpleClock()
{
    timeInSeconds = 0;
    start = time::high_resolution_clock::now();
    running = true;
}

void SimpleClock::pause()
{
    auto end = time::high_resolution_clock::now();
    timeInSeconds += static_cast<double>(time::duration_cast<time::nanoseconds>(end - start).count()) * 1e-9;
    running = false;
}

void SimpleClock::unpause()
{
    start = time::high_resolution_clock::now();
    running = true;
}

void SimpleClock::restart()
{
    timeInSeconds = 0;
    start = time::high_resolution_clock::now();
    running = true;
}

const double SimpleClock::getTimeInSeconds() const
{
    auto end = time::high_resolution_clock::now();
    if (running) {
        return timeInSeconds +
               static_cast<double>(time::duration_cast<time::nanoseconds>(end - start).count()) * 1e-9;
    } else {
        return timeInSeconds;
    }
}

/// --------------------------------------------------------------------------------------------------------------------
/// Logging
/// --------------------------------------------------------------------------------------------------------------------

void cvrplib::initLogging()
{
    logging::add_common_attributes();
    logging::register_simple_formatter_factory<log::severity_level, char>("Severity");
    logging::add_console_log(std::cout, keywords::format
            = "[%TimeStamp%] [" CVRPLIB_LOG_CONST_CVRPLIB_NAME "] [%Severity%] >> %Message%");
    logging::core::get()->set_filter(log::severity >= log::trace);
}

void cvrplib::initLogging(log::severity_level level)
{
    logging::core::get()->set_filter(log::severity >= level);
}

void cvrplib::testLogging()
{
    CVRPLIB_LOG_TRACE("Test");
    CVRPLIB_LOG_DEBUG("Test");
    CVRPLIB_LOG_INFO("Test");
    CVRPLIB_LOG_WARNING("Test");
    CVRPLIB_LOG_ERROR("Test");
    CVRPLIB_LOG_FATAL("Test");
    CVRPLIB_LOG_INFO(
            "Color test: "
                    << CVRPLIB_LOGSTYLE_COL_BLACK << "black"
                    << CVRPLIB_LOGSTYLE_COL_RED << "red"
                    << CVRPLIB_LOGSTYLE_COL_GREEN << "green"
                    << CVRPLIB_LOGSTYLE_COL_YELLOW << "yellow"
                    << CVRPLIB_LOGSTYLE_COL_BLUE << "blue"
                    << CVRPLIB_LOGSTYLE_COL_MAGENTA << "magenta"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTGREY << "lightgrey"
                    << CVRPLIB_LOGSTYLE_COL_DARKGREY << "darkgrey"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTRED << "lightred"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTGREEN << "lightgreen"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTYELLOW << "lightyellow"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTBLUE << "lightblue"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTMAGENTA << "lightmagenta"
                    << CVRPLIB_LOGSTYLE_COL_LIGHTCYAN << "lightcyan"
                    << CVRPLIB_LOGSTYLE_COL_WHITE << "white");
}

/// --------------------------------------------------------------------------------------------------------------------
/// Random
/// --------------------------------------------------------------------------------------------------------------------

using namespace cvrplib::random;

RandomGenerator gen;

void cvrplib::random::initRandom()
{
    gen = RandomGenerator(static_cast<const uint32_t &>(getCurrentTimeNS()));
}

void cvrplib::random::initRandomDeterministic(int seed)
{
    gen = RandomGenerator(static_cast<const uint32_t &>(seed));
}

RandomGenerator &cvrplib::random::rng()
{
    return gen;
}

/* int cvrplib::random::getRandomInteger(){ */
/* } */

const int cvrplib::random::getRandomAmongAvailable(
        const unsigned availableCount,
        const std::vector<bool> &available
)
{
    auto r = random::UniformIntDistribution<unsigned>{0, availableCount - 1}(gen);
    for (unsigned i = 0; i < available.size(); ++i) {
        if (available[i]) {
            if (r == 0) {
                /* printf("With the parameter n=%d, I found %d.\n", availableCount, i); */
                return i;
            }
            --r;
        }
    }
    return -1;
}

__syscall_slong_t cvrplib::random::getCurrentTimeNS()
{
    struct timespec tm{};
    clock_gettime(CLOCK_REALTIME, &tm);
    return tm.tv_nsec;
}

/// --------------------------------------------------------------------------------------------------------------------
/// Strings
/// --------------------------------------------------------------------------------------------------------------------

#include <boost/algorithm/string.hpp>

std::string cvrplib::toLower(std::string str)
{
    return boost::algorithm::to_lower_copy(str);
}

std::vector<std::string> cvrplib::split (std::string s, std::string delimiter) {
      size_t pos_start = 0, pos_end, delim_len = delimiter.length();
      std::string token;
      std::vector<std::string> res;

      while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
      }

      res.push_back (s.substr (pos_start));
      return res;
}

/// --------------------------------------------------------------------------------------------------------------------
/// General
/// --------------------------------------------------------------------------------------------------------------------

void cvrplib::initUtils(boost::log::trivial::severity_level level, int seed, bool useTrueRandom)
{
    initLogging(level);
    if (!useTrueRandom){
      initRandomDeterministic(seed);
    } else {
      initRandom();
    }

}
