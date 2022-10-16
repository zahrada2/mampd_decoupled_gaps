/**
 * File:    local_search.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_LOCAL_SEARCH_HPP
#define CVRP_LOCAL_SEARCH_HPP

#include "Solution.hpp"
#define RECOUNT_CHECK false

/* int mapf_iter = 0; */

namespace cvrplib {

    namespace ls {


      using FunptrOperator = bool (*)(
              solutions::Solution &,
              MAPF_solution &,
              MainType &
      );
  
      using PtbFunptrOperator = void (solutions::Solution::*)(
              uint16_t,
              uint16_t,
              Pos,
              Pos

      );

      bool exploreVnsNeighbourhoods(
          std::vector<FunptrOperator> neighborhoods,
          solutions::Solution &pathRes,
          MAPF_solution &best_mapf,
          MainType &cost);
        

      bool exchange(
          solutions::Solution &pathAux, 
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool exchangeFA(
          solutions::Solution &pathAux, 
          MAPF_solution &best_mapf,
          MainType &cost);



      bool relocate(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool relocateFA(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool twoOpt(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool threeOpt(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool twoOptFA(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);
      
      bool threeOptFA(
          solutions::Solution &pathAux,
          MAPF_solution &best_mapf,
          MainType &cost);

      bool exploreNeighborhoods(
              solutions::Solution &pathRes,
              MainType &cost
      );

        /* bool twoOpt( */
        /*         solutions::Solution &pathRes, */
        /*         MainType &cost */
        /* ); */

        bool twoString(
                solutions::Solution &pathRes,
                MainType &cost,
                solutions::Pos X,
                solutions::Pos Y
        );

        bool onePoint(
                solutions::Solution &pathRes,
                MainType &cost
        );

        inline bool orOpt2(
                solutions::Solution &pathRes,
                MainType &cost
        )
        {
            return twoString(pathRes, cost, 0, 2);
        }

        inline bool orOpt3(
                solutions::Solution &pathRes,
                MainType &cost
        )
        {
            return twoString(pathRes, cost, 0, 3);
        }

        inline bool orOpt4(
                solutions::Solution &pathRes,
                MainType &cost
        )
        {
            return twoString(pathRes, cost, 0, 4);
        }

        bool twoPoint(
                solutions::Solution &pathRes,
                MainType &cost
        );

        void perturbate(
                std::vector<PtbFunptrOperator> neighborhoods,
                solutions::Solution &pathRes,
                MainType &cost,
                uint16_t max_steps = 10);
        
        void perturbatevtwo(
                std::vector<PtbFunptrOperator> neighborhoods,
                solutions::Solution &pathRes,
                MainType &cost,
                uint16_t max_steps = 10);


        void vnd(
                solutions::Solution &pathRes,
                MainType &cost,
                const Goals &goals,
                bool &interrupt
        );

        void vndParam(
                solutions::Solution &pathRes,
                MainType &cost,
                const Goals &goals,
                bool &interrupt,
                unsigned operatorsSeq
        );

        void rvnd(
                solutions::Solution &pathRes,
                MainType &cost,
                const Goals &goals,
                bool &interrupt
        );

    }

}

#endif //CVRP_LOCAL_SEARCH_HPP
