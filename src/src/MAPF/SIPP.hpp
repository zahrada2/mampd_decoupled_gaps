#ifndef MAPF_CARDINAL_SIPP_HPP
#define MAPF_CARDINAL_SIPP_HPP

#include "utils.hpp"
#include "structures.hpp"
#include "structs.h"
#include <list>
#include <unordered_map>

namespace cvrplib {

    struct pair_hash
    {
        template <class T1, class T2>
            std::size_t operator() (const std::pair<T1, T2> &pair) const {
                return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    class SIPP
    {
    public:
        std::vector<Path> find_path(Agent agent, std::vector<Node> starts, Node goal, int wait_in_goal, boost::shared_ptr<MapData> map, const std::vector<std::vector<std::vector<int> > > &h_values);
        std::vector<Path> find_full_path(Agent agent, std::vector<Node> subgoals, boost::shared_ptr<MapData> map, std::vector<Path> obstacles, const std::vector<std::vector<std::vector<int> > > &h_values, std::set<std::pair<int, int>> _blocked_nodes = {});

    private:
        Agent agent;
        void find_successors(const Node curNode, boost::shared_ptr<MapData> map, std::list<Node> &succs, const std::vector<std::vector<std::vector<int> > > &h_values);
        void add_open(Node newNode);
        Node find_min(int size);
        int  count_h_value(int i, int j, int goal_i, int goal_j);
        Path reconstruct_path(Node curNode);
        bool checkGoal(Node goal, const std::vector<Path> &obstacles);
        void clear();
        std::vector<std::pair<int, int>> get_intervals(Node newNode);
        int get_EAT(Node newNode, Node curNode);
        void make_constraints(std::vector<Path> obstacles);

        unsigned int openSize;
        std::unordered_map<std::pair<int, int>, Node, pair_hash> close;
        std::vector<std::list<Node>> open;
        Path path;
        Node cur_goal;
        std::map<int, std::set<int>> vertex_constraints; //moments when obstacles occupy the corresponding vertex
        std::map<std::pair<int, int>, std::set<int>> edge_constraints; //moments when obstacles moves along the corresponding edge
        std::map<int, std::vector<std::pair<int, int>>> safe_intervals;
        std::map<int, int> obstacles_goals;
        std::set<std::pair<int, int>> blocked_nodes;
    };
}

#endif //MAPF_CARDINAL_SIPP_HPP
