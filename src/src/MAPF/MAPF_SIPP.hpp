#ifndef MAPF_SIPP_HPP
#define MAPF_SIPP_HPP

#include <random>
#include "SIPP.hpp"

namespace cvrplib {

    class MAPF_Prioritized:public MAPF_interface{
    public:
        MAPF_Prioritized(int _seed = 0, bool _useTTD = false, bool _blockGoals = false):MAPF_interface(_seed, _useTTD, _blockGoals){calls = 0; last_replanned_agent = -1; bad_goals = 0; lastSolutionFound = false; planner = SIPP();}
        Config config;
        void checkSolution(MAPF_solution solution);
        MAPF_solution computeCost(MAPF_input input) override;
        bool setMap(boost::shared_ptr<MapData> map_) override;
        MainType estimateCost(MAPF_input input_) override;
        bool computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace) override;
        bool computeTwoOpt(int agentID, int startPlace, int endPlace) override;
        virtual bool computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB) override;
        bool computeThreeOpt(int agentID, int placeI, int placeJ, int placeK) override;
        MAPF_solution getBestSolution() override;
        void setInitSolution(MAPF_solution solution, MAPF_input input) override;
        void saveSolutionIntoXML(std::string fileName, MAPF_solution solution) override;
        /* void saveSolutionMapfIR(std::string fileName, MAPF_solution solution) override; */
        void saveInstanceIntoXML(std::string fileName, MAPF_input& input, MapData& map_data) override;
        void saveSolutionMapfIR(std::string fileName, MAPF_solution solution, std::vector<uint16_t> starting_depots, std::vector<uint16_t> ending_depots,
                int numVehicles, std::string map_name, std::vector<std::pair<int,int>>, int cost_lower_bound, int final_cost, std::string method) override;

        void saveSolutionMampdIR(std::string fileName, MAPF_solution solution, int numVehicles, std::string map_name, std::string instance_name, std::vector<std::pair<int,int>> vertex_locations, double vrp_cost, std::vector<std::vector<int>> assignments, MAPF_input mapf_task) override;

        bool lastInstanceSolutionFound() override;
        int getAg(std::vector<int> goals_in_path, MAPF_solution solution) override;
    protected:
        bool changePriorities(int bad_i);
        void setPriorities(MAPF_input input);
        void calculateHValues();
        std::vector<Path> findPartialPath(std::vector<int> subgoals_ids, int agent_id, std::vector<Path> obstacles = {});
        Path makeJoinedPath(std::vector<Path> sequenced_path);
        std::vector<std::vector<int>> priorities;
        std::vector<int> current_priorities;
        Agent curagent;
        MAPF_input init_input;
        boost::shared_ptr<MapData> map;
        int calls;
        std::vector<std::vector<std::vector<int>>> h_values;
        MAPF_solution init_solution, best_solution;
        int last_replanned_agent;
        int bad_goals;
        bool lastSolutionFound;
        SIPP planner;
    };
}

#endif //MAPF_SIPP_HPP
