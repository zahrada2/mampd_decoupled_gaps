#include "SIPP.hpp"

using namespace cvrplib;

void SIPP::clear()
{
    open.clear();
    close.clear();
    openSize = 0;
    path.pathlength = -1;
}

void SIPP::make_constraints(std::vector<Path> obstacles)
{
    vertex_constraints.clear();
    edge_constraints.clear();
    safe_intervals.clear();
    obstacles_goals.clear();
    for(const auto &obs:obstacles)
    {
        for(const auto &n:obs.sections)
        {
            auto vertex_cons = vertex_constraints.find(n.id);
            if(vertex_cons == vertex_constraints.end())
                vertex_constraints.insert({n.id, {n.g}});
            else
                vertex_cons->second.insert(n.g);
        }
        for(unsigned int k = 0; k < obs.sections.size()-1; k++)
        {

            auto edge_cons = edge_constraints.find({obs.sections[k].id, obs.sections[k+1].id});
            if(edge_cons == edge_constraints.end())
                edge_constraints.insert({{obs.sections[k].id, obs.sections[k+1].id}, {obs.sections[k].g}});
            else
                edge_cons->second.insert(obs.sections[k].g);
        }
        obstacles_goals.insert({obs.sections.back().id, obs.sections.back().g});
    }
}

std::vector<std::pair<int, int>> SIPP::get_intervals(Node newNode)
{
    auto has_intervals = safe_intervals.find(newNode.id);
    if(has_intervals != safe_intervals.end())
        return has_intervals->second;
    std::pair<int, int> interval = {0, CN_INFINITY};
    std::vector<std::pair<int, int>> intervals;
    auto node_cons = vertex_constraints.find(newNode.id);
    if(node_cons == vertex_constraints.end())
    {
        safe_intervals.insert({newNode.id, {interval}});
        return {interval};
    }
    else
    {
        for(auto it = node_cons->second.begin(); it != node_cons->second.end(); it++)
        {
            if(it == node_cons->second.begin())
            {
                interval = {0, *it-1};
                intervals.push_back(interval);
            }
            if(it == --node_cons->second.end())
                interval = {*it+1, CN_INFINITY};
            else
            {
                auto iter=it;
                iter++;
                interval = {*it+1, *iter-1};
            }
            auto obs_goal = obstacles_goals.find(newNode.id);
            if(obs_goal != obstacles_goals.end())
            {
                if(obs_goal->second <= interval.first)
                    continue;
                else if(obs_goal->second < interval.second)
                    interval.second = obs_goal->second - 1;
            }
            intervals.push_back(interval);
        }
    }
    safe_intervals.insert({newNode.id, intervals});
    return intervals;
}

int SIPP::get_EAT(Node newNode, Node curNode)
{
    auto edge_cons = edge_constraints.find({newNode.id, curNode.id}); //looking for moves in contrary direction
    if(edge_cons != edge_constraints.end())
    {
        auto it = edge_cons->second.begin();
        while(it != edge_cons->second.end())
        {
            if(*it+1 == newNode.g)
                newNode.g++;
            it++;
        }
        if(newNode.g > newNode.interval.second || newNode.g - 1 > curNode.interval.second)
            return CN_INFINITY;
    }
    return newNode.g;
}

int SIPP::count_h_value(int i, int j, int goal_i, int goal_j)
{
    return abs(i - goal_i) + abs(j - goal_j);
}

bool SIPP::checkGoal(Node goal, const std::vector<Path> &obstacles)
{
    for(const auto &obs:obstacles)
    {
        for(int k = goal.g; k < obs.sections.size(); k++)
            if(obs.sections[k].i == goal.i && obs.sections[k].j == goal.j)
                return false;
    }
    return true;

}
void SIPP::find_successors(const Node curNode, boost::shared_ptr<MapData> map, std::list<Node> &succs, const std::vector<std::vector<std::vector<int> > > &h_values)
{
    Node newNode;
    for(auto move : map->getValidMoves(curNode.i, curNode.j))
    {
        newNode.i = curNode.i + move.first;
        newNode.j = curNode.j + move.second;
        newNode.id = newNode.i*map->getMapWidth() + newNode.j;
        newNode.g = curNode.g + 1;
        if(blocked_nodes.find({newNode.i, newNode.j}) != blocked_nodes.end())
            continue;
        int h_value = h_values[cur_goal.id][newNode.i][newNode.j];
        auto intervals = get_intervals(newNode);
        for(auto interval:intervals)
        {
            newNode.interval = interval;
            newNode.g = std::max(curNode.g + 1, interval.first);
            if(newNode.g - 1 > curNode.interval.second || newNode.g > newNode.interval.second)
                continue;
            newNode.g = get_EAT(newNode, curNode);
            newNode.f = newNode.g + h_value;
            if(newNode.f < CN_INFINITY)
            {
                succs.push_back(newNode);
            }
        }
    }
}

Node SIPP::find_min(int size)
{
    Node min;
    min.f = CN_INFINITY;
    for(int i = 0; i < size; i++)
        if(!open[i].empty())
            if(open[i].begin()->f < min.f || (open[i].begin()->f == min.f && open[i].begin()->g > min.g))
                min = *open[i].begin();
    return min;
}

void SIPP::add_open(Node newNode)
{
    std::list<Node>::iterator iter, pos;
    bool posFound = false;
    pos = open[newNode.i].end();
    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if (iter->f >= newNode.f && !posFound)
        {
            if (iter->f == newNode.f)
            {
                if (newNode.g >= iter->g)
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }
        if (iter->j == newNode.j && iter->interval.second == newNode.interval.second)
        {
            if(iter->g <= newNode.g)
                return;
            if(pos == iter)
            {
                iter->f = newNode.f;
                iter->g = newNode.g;
                iter->interval = newNode.interval;
                iter->parent = newNode.parent;
                return;
            }
            open[newNode.i].erase(iter);
            openSize--;
            break;
        }
    }
    openSize++;
    open[newNode.i].insert(pos, newNode);
}

Path SIPP::reconstruct_path(Node curNode)
{
    Path path;
    path.pathlength = curNode.g;
    if(curNode.parent != nullptr)
    do
    {
        path.sections.insert(path.sections.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    path.sections.insert(path.sections.begin(), curNode);
    for(int i = 0; i < path.sections.size(); i++)
    {
        int j = i + 1;
        if(j == path.sections.size())
            break;
        if(path.sections[j].g - path.sections[i].g != 1)
        {
            Node add = path.sections[i];
            add.g++;
            add.f++;
            path.sections.emplace(path.sections.begin() + j, add);
            i--;
        }
    }
    return path;

}

std::vector<Path> SIPP::find_full_path(Agent agent, std::vector<Node> subgoals, boost::shared_ptr<MapData> map, std::vector<Path> obstacles, const std::vector<std::vector<std::vector<int> > > &h_values, std::set<std::pair<int, int> > _blocked_nodes)
{
    this->agent = agent;
    this->blocked_nodes = _blocked_nodes;
    make_constraints(obstacles);
    if(subgoals.empty())
    {
        Node start(agent.start_i, agent.start_j, 0, 0);
        start.id = start.i*map->getMapWidth() + start.j;
        start.interval = get_intervals(start)[0];
        Node goal(agent.goal_i, agent.goal_j, 0, 0);
        std::vector<Node> starts = {start};
        return find_path(agent, starts, goal, 0, map, h_values);
    }
    std::vector<Path> all_paths;
    for(unsigned int k=1; k < subgoals.size(); k++)
    {
        Node start, goal;
        std::vector<Node> starts;
        if(k == 1)
        {
            start = Node(agent.start_i, agent.start_j, 0, 0);
            start.id = start.i*map->getMapWidth() + start.j;
            start.interval = get_intervals(start)[0];
            starts = {start};
        }
        else
        {
            for(const Path &path:all_paths)
                if(path.sections.back().i == subgoals[k-1].i && path.sections.back().j == subgoals[k-1].j)
                {
                    start = path.sections.back();
                    start.parent = nullptr; //to correctly stop during the path reconstruction proccess
                    starts.push_back(start);
                }
        }
        goal = Node(subgoals[k].i, subgoals[k].j);
        int wait_in_goal = map->getWaitTime(subgoals[k].id);
        if(k+1 == subgoals.size())
            wait_in_goal = 0;
        cur_goal = goal;
        cur_goal.id = subgoals[k].id;
        std::vector<Path> paths = find_path(agent, starts, goal, wait_in_goal, map, h_values);
        if(paths.empty())//no paths were found
            return {};
        for(Path &path: paths)
        {
            for(int w = 0; w < wait_in_goal; w++)
            {
                Node wait = path.sections.back();
                wait.g++;
                path.sections.push_back(wait);
            }
            path.pathfound = true;
            path.pathlength = path.sections.size() - 1;
            path.goalPOI_ID = cur_goal.id;
            all_paths.push_back(path);
        }
    }
    std::vector<Path> full_path = {all_paths.back()};
    while(full_path.front().sections.front().g > 0)
    {
        Path add;
        for(const Path& p:all_paths)
            if(p.sections.back().i == full_path.front().sections.front().i && p.sections.back().j == full_path.front().sections.front().j &&
                    p.sections.back().g == full_path.front().sections.front().g && p.sections.back().interval.second == full_path.front().sections.front().interval.second)
            {
                add = p;
                break;
            }
        full_path.insert(full_path.begin(), add);
    }
    return full_path;
}

std::vector<Path> SIPP::find_path(Agent agent, std::vector<Node> starts, Node goal, int wait_in_goal, boost::shared_ptr<MapData> map, const std::vector<std::vector<std::vector<int> > > &h_values)
{
    this->clear();
    open.resize(map->getMapHeight());
    for(Node n:starts)
        add_open(n);
    std::vector<Path> paths;
    while(openSize > 0)
    {
        Node curNode = find_min(map->getMapHeight());
        open[curNode.i].pop_front();
        openSize--;
        close.insert({{curNode.id, curNode.interval.second}, curNode});
        if(curNode.i == goal.i && curNode.j == goal.j && curNode.g + wait_in_goal <= curNode.interval.second)
        {
            paths.push_back(reconstruct_path(curNode));
            paths.back().pathlength = curNode.g;
            paths.back().agent_id = agent.int_id;
            if(curNode.interval.second == CN_INFINITY)
                return paths;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find({curNode.id, curNode.interval.second})->second);
        while(it != succs.end())
        {
            it->parent = parent;
            if(close.find({it->id, it->interval.second}) == close.end())
            {
                add_open(*it);
            }
            it++;
        }
    }
    //std::cout<<"PATH NOT FOUND "<<starts.front().i<<" "<<starts.front().j<<" "<<goal.i<<" "<<goal.j<<"  "<<agent.id<<"\n";
    return {};
}
