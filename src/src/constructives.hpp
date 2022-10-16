/**
 * File:    constructives.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_CONSTRUCTIVES_HPP
#define CVRP_CONSTRUCTIVES_HPP

#include "Solution.hpp"
/* #include "tinyxml2.h" */
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <regex>

namespace cvrplib {

    namespace constructives {
      
      
        /* void loadRoute(std::string filename, MAPF_solution &mapf_solution, solutions::Solution &pathRes, int maxIter); */

        void rand(
                solutions::Solution &pathRes,
                MAPF_solution &mapf_solution,
                MainType &cost
        );
        
        void randWithOrders(
                solutions::Solution &pathRes,
                MAPF_solution &mapf_solution,
                MainType &cost
        );
        
        void clarkeWright(
                solutions::Solution &pathRes,
                MainType &cost
        );
        
        void mcwsa(
                solutions::Solution &pathRes,
                MainType &cost
        );
        
        void mcwsa_alternative(
                solutions::Solution &pathRes,
                MainType &cost
        );

    }

}

#endif //CVRP_CONSTRUCTIVES_HPP
