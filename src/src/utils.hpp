/**
 * File:    utils.hpp
 *
 * Date:    20.9.19
 * Author:  Jan Mikula
 * E-mail:  mikulj14@fel.cvut.cz
 *
 */

#ifndef CVRP_UTILS_HPP
#define CVRP_UTILS_HPP

/// --------------------------------------------------------------------------------------------------------------------
/// Exit codes
/// --------------------------------------------------------------------------------------------------------------------

#define CVRPLIB_EXIT_SUCCESS (0)
#define CVRPLIB_EXIT_FAILURE (1)

/// --------------------------------------------------------------------------------------------------------------------
/// Clock
/// --------------------------------------------------------------------------------------------------------------------

#include <chrono>

namespace cvrplib {

    namespace time = std::chrono;

    class SimpleClock {
    public:
        SimpleClock();

        void pause();

        void unpause();

        void restart();

        const double getTimeInSeconds() const;

    private:
        time::time_point<time::high_resolution_clock> start;
        double timeInSeconds;
        bool running;
    };

}

/// --------------------------------------------------------------------------------------------------------------------
/// Logging
/// --------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>

#define CVRPLIB_LOG_CONST_CVRPLIB_NAME "cvrplib"

#define CVRPLIB_LOGSTYLE_NORMAL "\e[0m"
#define CVRPLIB_LOGSTYLE_BOLD "\e[1m"
#define CVRPLIB_LOGSTYLE_DIM "\e[2m"
#define CVRPLIB_LOGSTYLE_COL_BLACK "\e[30m"
#define CVRPLIB_LOGSTYLE_COL_RED "\e[31m"
#define CVRPLIB_LOGSTYLE_COL_GREEN "\e[32m"
#define CVRPLIB_LOGSTYLE_COL_YELLOW "\e[33m"
#define CVRPLIB_LOGSTYLE_COL_BLUE "\e[34m"
#define CVRPLIB_LOGSTYLE_COL_MAGENTA "\e[35m"
#define CVRPLIB_LOGSTYLE_COL_CYAN "\e[36m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTGREY "\e[37m"
#define CVRPLIB_LOGSTYLE_COL_DARKGREY "\e[90m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTRED "\e[91m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTGREEN "\e[92m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTYELLOW "\e[93m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTBLUE "\e[94m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTMAGENTA "\e[95m"
#define CVRPLIB_LOGSTYLE_COL_LIGHTCYAN "\e[96m"
#define CVRPLIB_LOGSTYLE_COL_WHITE "\e[97m"

#define CVRPLIB_LOG_TRACE(x) BOOST_LOG_TRIVIAL(trace) << CVRPLIB_LOGSTYLE_NORMAL \
    << CVRPLIB_LOGSTYLE_DIM << " " << x << CVRPLIB_LOGSTYLE_NORMAL
#define CVRPLIB_LOG_DEBUG(x) BOOST_LOG_TRIVIAL(debug) << CVRPLIB_LOGSTYLE_NORMAL \
    << CVRPLIB_LOGSTYLE_DIM << " " << x << CVRPLIB_LOGSTYLE_NORMAL
#define CVRPLIB_LOG_INFO(x) BOOST_LOG_TRIVIAL(info) << CVRPLIB_LOGSTYLE_NORMAL \
    << " " << x << CVRPLIB_LOGSTYLE_NORMAL
#define CVRPLIB_LOG_WARNING(x) BOOST_LOG_TRIVIAL(warning) << CVRPLIB_LOGSTYLE_NORMAL \
    << CVRPLIB_LOGSTYLE_COL_YELLOW << " " << x << CVRPLIB_LOGSTYLE_NORMAL
#define CVRPLIB_LOG_ERROR(x) BOOST_LOG_TRIVIAL(error) << CVRPLIB_LOGSTYLE_NORMAL \
    << CVRPLIB_LOGSTYLE_COL_RED << " " << x << CVRPLIB_LOGSTYLE_NORMAL
#define CVRPLIB_LOG_FATAL(x) BOOST_LOG_TRIVIAL(fatal) << CVRPLIB_LOGSTYLE_NORMAL \
    << CVRPLIB_LOGSTYLE_COL_RED << CVRPLIB_LOGSTYLE_BOLD << " " << x << CVRPLIB_LOGSTYLE_NORMAL

namespace cvrplib {

    namespace logging = boost::log;
    namespace log = boost::log::trivial;
    namespace keywords = boost::log::keywords;

    void initLogging();

    void initLogging(log::severity_level level);

    void testLogging();

}

/// --------------------------------------------------------------------------------------------------------------------
/// Random
/// --------------------------------------------------------------------------------------------------------------------

#include <ctime>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>

namespace cvrplib {

    namespace random {

        using RandomGenerator = boost::random::mt19937;
        template<typename T>
        using UniformIntDistribution = boost::random::uniform_int_distribution<T>;
        template<typename T>
        using UniformRealDistribution = boost::random::uniform_real_distribution<T>;

        void initRandom();

        void initRandomDeterministic(int seed);

        /* int getRandomInteger(){ */

        RandomGenerator &rng();

        const int getRandomAmongAvailable(
                const unsigned availableCount,
                const std::vector<bool> &available
        );

        __syscall_slong_t getCurrentTimeNS();

    }

}

/// --------------------------------------------------------------------------------------------------------------------
/// Strings
/// --------------------------------------------------------------------------------------------------------------------

namespace cvrplib {

    std::string toLower(std::string str);

    std::vector<std::string> split (std::string s, std::string delimiter);
}

/// --------------------------------------------------------------------------------------------------------------------
/// General
/// --------------------------------------------------------------------------------------------------------------------

namespace cvrplib {

    void initUtils(log::severity_level level = log::info, int seed = 0, bool useTrueRandom = false);

}


#endif //CVRP_UTILS_HPP
