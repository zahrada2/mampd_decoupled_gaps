#ifndef __VRPPROBLEMLOADER_H__
#define __VRPPROBLEMLOADER_H__

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "vrpnode.hpp"

namespace imr {
    namespace vrp {

        enum ProblemType
        {
          VRP = 0,
          CVRP = 1,
          TSP = 2,
          NUMBER = 3
        };

        struct Problem
        {
            ProblemType problem_type;
            int numVehicles;
            int dimension;
            int capacity;
            /* std::vector<uint16_t> goals; */
            /* std::vector<std::pair<int, int>> goals; */
            std::vector<VRPNode> goals;
            std::vector<uint16_t> starting_depots;
            std::vector<uint16_t> ending_depots;
            std::vector<int> demands;
            std::vector<int> waiting_times;

        };

      namespace Spec {
         enum TSpec {
            NAME,
            TYPE,
            COMMENT,
            DIMENSION,
            CAPACITY,
            VEHICLES,
            EDGE_WEIGHT_TYPE,
            EDGE_WEIGHT_FORMAT,
            EDGE_DATA_FORMAT,
            NODE_COORD_TYPE,
            DISPLAY_DATA_TYPE,
            NUMBER
         };
      } //end namespace 

      namespace DataSection {
         enum Type {
            NODE_COORD_SECTION,
            DEPOT_SECTION,
            STARTING_DEPOT_SECTION,
            ENDING_DEPOT_SECTION,
            DEMAND_SECTION,
            EDGE_DATA_SECTION,
            FIXED_EDGES_SECTION,
            DISPLAY_DATA_SECTION,
            TOUR_SECTION,
            EDGE_WEIGHT_SECTION,
            WAITING_TIME_SECTION,
            NUMBER
         };
      } //end namespace 


        class VRPProblemLoader{
          public:

            VRPProblemLoader();

            bool loadFile(std::string filename);

            Problem getProblemData();


          private:

            Problem problem_data;
            bool specified_starting_depots;
            bool specified_ending_depots;

            static std::string KeywordsStr[];
            static std::string DataSectionStr[];
            static std::string ProblemTypeStr[];

            bool getSpecification(std::string name, Spec::TSpec &spec);

            bool getDataSection(std::string name, DataSection::Type & type);
            
            bool isDataSection(std::string name);

            bool parseDataSection(std::ifstream &fin, Problem &new_problem);

            bool detectType(std::string filename);

            std::vector<std::string> split (std::string s, std::string delimiter);

        };
    }
}



#endif
