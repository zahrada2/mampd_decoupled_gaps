#ifndef PBS_HPP
#define PBS_HPP

#include "utils.hpp"
#include "structures.hpp"
#include "structs.h"
#include <list>
#include <unordered_map>
#include <random>
#include "MAPF_SIPP.hpp"
#include "topological_sorting.h"

namespace cvrplib {

    class MAPF_PBS:public MAPF_Prioritized{
    public:
        MAPF_PBS(int _seed = 0, bool _useTTD = false, bool _blockGoals = false, PBSversion _v = v2):MAPF_Prioritized(_seed, _useTTD, _blockGoals) {clear_tree = true; v = _v;}
        MAPF_solution computeCost(MAPF_input input) override;
        bool computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB) override;
        bool computeTwoOpt(int agentID, int startPlace, int endPlace) override;
        bool computeThreeOpt(int agentID, int placeI, int placeJ, int placeK) override;
        bool computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace) override;
        void setInitSolution(MAPF_solution solution, MAPF_input input) override;
        int getAg(std::vector<int> goals_in_path, MAPF_solution solution);

    private:
        MAPF_solution repairSolution(std::map<int, std::vector<int> > subgoals_ids);
        bool init_root(const MAPF_input &input);
        std::vector<Path> get_paths(PBS_Node *node, int agents_size);
        Conflict check_paths(Path pathA, Path pathB);
        double get_cost(PBS_Node node, int agent_id);
        std::vector<Path> repair_solution(PBS_Node &node, const std::vector<Path> &all_paths, const MAPF_input &input);
        PBS_Tree tree;
        int node_id;
        bool clear_tree;
        PBSversion v;
        std::vector<std::vector<int>> good_constraints;
    };
}

#endif //PBS_HPP
