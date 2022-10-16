/**
 * File:    MAPF_interface.hpp
 *
 * Date:    09/02/2021
 * Author:  David Zahradka 
 * E-mail:  zahrada2@fel.cvut.cz
 *
 */

#ifndef MAPF_INTERFACE_HPP 
#define MAPF_INTERFACE_HPP

#include "utils.hpp"
#include "MapData.h"
#include "MAPF/structs.h"

namespace cvrplib {

        enum PBSversion{
            v0,  //brute-force
            v1,  //prioritized
            v2,  //reuse constraints
            v2b, //reuse constraints and old paths
            v3,  //reuse constraints + online learning
            v3b  //reuse constraints and old paths + online learning
        };

        using MainType = long long;
        using Pos = unsigned;


        class MAPF_interface {
        public:
            
            /// --------------------------------------------------------------------------------------------------------
            /// Public constructors + destructor
            /// --------------------------------------------------------------------------------------------------------

            /* MAPF_interface(const MAPF_interface&) = default; */
            
            MAPF_interface(int _seed = 0, bool _useTTD = false, bool _blockGoals = false);

            /* MAPF_interface(MAPF_interface &&) = default; */

            /* virtual ~MAPF_interface() = default; */
            
            void initializeMAPF();

            virtual bool setMap(boost::shared_ptr<MapData> map_);

            /// --------------------------------------------------------------------------------------------------------
            /// Connection function
            /// --------------------------------------------------------------------------------------------------------
            
            virtual MAPF_solution computeCost(MAPF_input input);
            virtual MainType estimateCost(MAPF_input input);
            virtual bool computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace);
            virtual bool computeTwoOpt(int agentID, int startPlace, int endPlace);
            virtual bool computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB);
            virtual bool computeThreeOpt(int agentID, int placeI, int placeJ, int placeK);
            virtual MAPF_solution getBestSolution();
            virtual void setInitSolution(MAPF_solution solution, MAPF_input input);
            virtual void saveInstanceIntoXML(std::string fileName, MAPF_input& input, MapData& map_data);
            virtual void saveSolutionIntoXML(std::string fileName, MAPF_solution solution);
            virtual void saveSolutionMapfIR(std::string fileName, MAPF_solution solution, std::vector<uint16_t> starting_depots, std::vector<uint16_t> ending_depots,
                int numVehicles, std::string map_name, std::vector<std::pair<int, int>> vertex_locations, int cost_lower_bound, int final_cost, std::string method);

            virtual void saveSolutionMampdIR(std::string fileName, MAPF_solution solution,
                                            int numVehicles, std::string map_name, std::string instance_name, std::vector<std::pair<int, int>> vertex_locations, double vrp_cost, std::vector<std::vector<int>> assignments, MAPF_input mapf_task);
            virtual bool lastInstanceSolutionFound();

            virtual int getAg(std::vector<int> goals_in_path, MAPF_solution solution);

        protected:

            /// --------------------------------------------------------------------------------------------------------
            /// Protected constructor
            /// --------------------------------------------------------------------------------------------------------

            bool useTTD;
            bool blockGoals;
            int seed; //seed for the random generator
            std::vector<double> found_costs;
            /* explicit MAPF_interface( */
            /*         /1* const Instance &inst, *1/ */
            /*         /1* std::string name = "unknown" *1/ */
            /* ); */

        private:
            /// A vector mapping vertex indices to coordinates. vertexMap[i] = X of vertex i, vertexMap[i+1] = Y of vertex i
            /// Thus, iterate by increments of two
            //std::vector<uint16_t> vertexMap;

            boost::shared_ptr<MapData> map;
            static const auto INVALID_PATH_FLAG = MainType(-1);

        };

}

#endif //MAPF_INTERFACE_HPP
