/**
 * File:    methods.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_METHODS_HPP
#define CVRP_METHODS_HPP

#include "structures.hpp"
#include "Solution.hpp"
#include <time.h>       /* time */ 
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <regex>


namespace fs = boost::filesystem;

namespace cvrplib {

    namespace methods {

        struct References {
            MainType &cost;
            const Goals &goals;
            solutions::Solution &pathRes;
            solutions::Solution &pathAux1;
            solutions::Solution &pathAux2;
            solutions::Solution &pathAux3;
        };

        void fmivns(const References &refs, 
                unsigned maxIter = 10
        );

        void simpleTwoPhase(const References &refs,
                unsigned maxIter = 10
        );
        
        void simplified_vns(const References &refs,
                unsigned maxIter = 10
        );

    }

}

#endif //CVRP_METHODS_HPP
