#include "MAPF_PBS.hpp"
using namespace cvrplib;

MAPF_solution MAPF_PBS::computeCost(MAPF_input input)
{
    //std::cout<<"PBS compute cost\n";
    auto t = std::chrono::high_resolution_clock::now();
    if(good_constraints.empty())
        good_constraints.resize(input.sequences.size(), std::vector<int>(input.sequences.size(), 0));
    last_replanned_agent = -1;
    int num_of_agents = input.sequences.size();
    bool solution_found(false);
    auto POICoords = map->getPOICoords();
    MAPF_solution solution;
    solution.cost = CN_INFINITY;
    lastSolutionFound = false;

    if(clear_tree)
    {
        tree.clear();
        if(!this->init_root(input))
            return solution;
    }
    clear_tree = true;
    PBS_Node node;
    do
    {
        auto parent = tree.get_front();
        node = *parent;
        tree.pop_front();
        if(node.conflicts.empty())
        {
            solution_found = true;
            break; //i.e. no conflicts => solution found
        }
        //std::cout<<node.id<<" "<<node.colliding_agents<<" "<<node.constraints_num<<" "<<node.cost<<"\n";
        auto paths = tree.get_joined_paths(node, num_of_agents);
        Conflict conflict = node.conflicts[0];
        std::vector<int> agents = {conflict.agent1, conflict.agent2};
        for(int a:agents)
        {
            std::vector<Path> obstacles;
            for(int i=0; i<num_of_agents; i++)
                if(node.priorities.find(std::make_pair(i,a)) != node.priorities.end())
                    obstacles.push_back(paths[i]);
            auto priorities = node.priorities;
            std::pair<int, int> priority = {conflict.agent2, conflict.agent1};
            if(a == conflict.agent2)
                std::swap(priority.first, priority.second);
            obstacles.push_back(paths[priority.first]);
            priorities.insert(priority);
            TopologicalSorting order(paths.size());
            for(const auto &lp:priorities)
                order.addEdge(lp.first, lp.second);
            if(order.isCyclic())
                continue;
            Agent agent = Agent();
            agent.start_i = POICoords[input.sequences.at(a).front()].first;
            agent.start_j = POICoords[input.sequences.at(a).front()].second;
            agent.goal_i = POICoords[input.sequences.at(a).back()].first;
            agent.goal_j = POICoords[input.sequences.at(a).back()].second;
            agent.int_id = a;
            std::vector<Node> subgoals;
            for(auto poi_id:input.sequences[a])
            {
                Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
                poi.id = poi_id;
                subgoals.push_back(poi);
            }
            std::set<std::pair<int, int>> blocked_nodes;
            if(blockGoals)
                for(int i=0; i < num_of_agents; i++)
                    if(i != agent.int_id && priorities.find(std::make_pair(agent.int_id, i)) == priorities.end())
                        blocked_nodes.insert({POICoords[input.sequences.at(i).back()].first, POICoords[input.sequences.at(i).back()].second});
            std::vector<Path> sequenced_path = planner.find_full_path(agent, subgoals, map, obstacles, h_values, blocked_nodes);
            if(!sequenced_path.empty())
            {
                Path joined_path = makeJoinedPath(sequenced_path);
                std::vector<Conflict> conflicts;
                for(const Conflict &c:node.conflicts)
                    if(c.agent1 != a && c.agent2 != a)
                        conflicts.push_back(c);
                for(const auto &p:paths)
                {
                    if(p.agent_id == a)
                        continue;
                    Conflict c = check_paths(joined_path, p);
                    if(c.t > 0)
                        conflicts.push_back(c);
                }
                PBS_Node newNode = PBS_Node({joined_path}, {sequenced_path}, parent, priorities, node.cost - get_cost(node, a) + (useTTD ? joined_path.TTD : joined_path.pathlength), parent->constraints_num + 1, conflicts.size(), conflicts);
                //newNode.paths = repair_solution(newNode, paths, map, task);
                newNode.id = ++node_id;
                tree.add_node(newNode);
            }
        }
        auto time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > 60)
            return solution;
    }
    while(tree.get_open_size() > 0);


    if(solution_found)
    {
        solution.paths = tree.get_sequenced_paths(node, num_of_agents);
        solution.joined_paths = tree.get_joined_paths(node, num_of_agents);
        solution.input = input;
        solution.cost = 0;
        solution.pbs_constraints = node.priorities;
        for(const Path &p: solution.joined_paths)
            solution.cost += useTTD ? p.TTD : p.pathlength;
        lastSolutionFound = true;
        checkSolution(solution);
    }
    if((v == v3 || v == v3b) && node.parent != nullptr)
    {
        auto all_constraints = node.priorities;
        while(node.parent != nullptr)
            node = *node.parent;
        for(auto c:all_constraints)
            if(node.priorities.count(c) == 0) //if this constraint was not in root, then it is a new one, added during the search
            {
                //std::cout<<c.first<<" "<<c.second<<" "<<good_constraints.size()<<" "<<good_constraints[0].size()<<"\n";
                good_constraints[c.first][c.second]++;
            }
    }
    //std::cout<<"Called PBS computeCost "<<solution.cost<<" "<<solution.pbs_constraints.size()<<" "<<tree.get_open_size()<<"\n";
    //saveInstanceIntoXML("PBS_instance_"+std::to_string(solution.cost)+".xml", input, *map);
    //saveSolutionIntoXML("PBS_solution_"+std::to_string(solution.cost)+".xml", solution);
    found_costs.push_back(solution.cost);
    return solution;
}

void MAPF_PBS::setInitSolution(MAPF_solution solution, MAPF_input input)
{
    init_solution = solution;
    init_input = input;
    best_solution = solution;
    good_constraints.resize(input.sequences.size(), std::vector<int>(input.sequences.size(), 0));
}

bool MAPF_PBS::computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB)
{
    //std::cout<<"called exchange\n";
    if(v == v1)
        return MAPF_Prioritized::computeExchange(agentA_ID, agentB_ID, placeA, placeB);
    auto sequenceA = init_input.sequences[agentA_ID];
    auto sequenceB = init_input.sequences[agentB_ID];
    std::swap(sequenceA[placeA], sequenceB[placeB]);
    double new_costA(0), new_costB(0), old_costA(0), old_costB(0);
    old_costA = init_solution.joined_paths[agentA_ID].pathlength;
    old_costB = init_solution.joined_paths[agentB_ID].pathlength;
    auto POICoords = map->getPOICoords();

    for(int i = 0; i < placeA - 1; i++)
        new_costA += init_solution.paths[agentA_ID][i].pathlength;
    for(int i = 0; i < placeB - 1; i++)
        new_costB += init_solution.paths[agentB_ID][i].pathlength;
    double estimate_costA(new_costA), estimate_costB(new_costB);
    for(int i = placeA - 1; i < sequenceA.size() - 1; i++)
        estimate_costA += h_values[sequenceA[i]][POICoords[sequenceA[i+1]].first][POICoords[sequenceA[i+1]].second];
    for(int i = placeB - 1; i < sequenceB.size() - 1; i++)
        estimate_costB += h_values[sequenceB[i]][POICoords[sequenceB[i+1]].first][POICoords[sequenceB[i+1]].second];
    if(useTTD)
    {
        estimate_costA *= (sequenceA.size() - 2);
        estimate_costB *= (sequenceB.size() - 2);
        old_costA *= (sequenceA.size() - 2);
        old_costB *= (sequenceB.size() - 2);
    }
    if(init_solution.cost + estimate_costA + estimate_costB - old_costA - old_costB > best_solution.cost) //if estimated cost of the new sequences are worse than the best solution's cost
        return false;

    std::map<int, std::vector<int>> new_subgoals;
    new_subgoals[agentA_ID] = sequenceA;
    new_subgoals[agentB_ID] = sequenceB;
    MAPF_solution solution = repairSolution(new_subgoals);
    if(solution.cost < best_solution.cost)
        best_solution = solution;
    return true;
}

bool MAPF_PBS::computeTwoOpt(int agentID, int startPlace, int endPlace)
{
    //std::cout<<"called 2opt\n";
    if(v == v1)
        return MAPF_Prioritized::computeTwoOpt(agentID, startPlace, endPlace);
    auto POICoords = map->getPOICoords();
    std::vector<Ver> sequence;
    for(int i = 0; i < startPlace; i++)
        sequence.push_back(init_input.sequences[agentID][i]);
    for(int i = endPlace; i >= startPlace; i--)
        sequence.push_back(init_input.sequences[agentID][i]);
    for(int i = endPlace + 1; i < init_input.sequences[agentID].size(); i++)
        sequence.push_back(init_input.sequences[agentID][i]);

    MainType estimate_cost = init_solution.cost;
    if(useTTD)
    {
        estimate_cost -= init_solution.joined_paths[agentID].pathlength*(sequence.size() - 2);
        double estimate_path_cost(0);
        for(int i = 1; i < startPlace - 1; i++)
            estimate_path_cost += init_solution.paths[agentID][i].pathlength;
        for(int i = startPlace; i < sequence.size(); i++)
            estimate_path_cost += h_values[sequence[i-1]][POICoords[sequence[i]].first][POICoords[sequence[i]].second];
        estimate_cost += estimate_path_cost*(sequence.size() - 2);
    }
    else
    {
        for(int i = startPlace - 1; i < init_solution.paths[agentID].size(); i++)
            estimate_cost -= init_solution.paths[agentID][i].pathlength;
        for(int i = startPlace; i < sequence.size(); i++)
            estimate_cost += h_values[sequence[i-1]][POICoords[sequence[i]].first][POICoords[sequence[i]].second];
    }
    if(estimate_cost > best_solution.cost) //new sequence is worse than best one
        return false;
    std::map<int, std::vector<int>> new_subgoals;
    new_subgoals[agentID] = sequence;
    MAPF_solution solution = repairSolution(new_subgoals);
    if(solution.cost < best_solution.cost)
        best_solution = solution;
    return true;
}

bool MAPF_PBS::computeThreeOpt(int agentID, int placeI, int placeJ, int placeK)
{
    //std::cout<<"called 3opt\n";
    if(v == v1)
        return MAPF_Prioritized::computeThreeOpt(agentID, placeI, placeJ, placeK);
    std::vector<std::vector<Ver>> new_sequences(4, std::vector<Ver>());
    std::vector<Ver> segment_start, segment_i, segment_j, segment_k;
    segment_start.insert(segment_start.end(), init_input.sequences[agentID].begin(), init_input.sequences[agentID].begin()+placeI);
    segment_i.insert(segment_i.end(), init_input.sequences[agentID].begin()+placeI, init_input.sequences[agentID].begin()+placeJ);
    segment_j.insert(segment_j.end(), init_input.sequences[agentID].begin()+placeJ, init_input.sequences[agentID].begin()+placeK);
    segment_k.insert(segment_k.end(), init_input.sequences[agentID].begin()+placeK, init_input.sequences[agentID].end());
    std::vector<Ver> segment_i_reverse = segment_i;
    std::reverse(segment_i_reverse.begin(), segment_i_reverse.end());
    std::vector<Ver> segment_j_reverse = segment_j;
    std::reverse(segment_j_reverse.begin(), segment_j_reverse.end());

    /// AB iC jk
    new_sequences[0].insert(new_sequences[0].end(), segment_start.begin(), segment_start.end());
    new_sequences[0].insert(new_sequences[0].end(), segment_i_reverse.begin(), segment_i_reverse.end());
    new_sequences[0].insert(new_sequences[0].end(), segment_j_reverse.begin(), segment_j_reverse.end());
    new_sequences[0].insert(new_sequences[0].end(), segment_k.begin(), segment_k.end());

    /// AC ji Bk
    new_sequences[1].insert(new_sequences[1].end(), segment_start.begin(), segment_start.end());
    new_sequences[1].insert(new_sequences[1].end(), segment_j_reverse.begin(), segment_j_reverse.end());
    new_sequences[1].insert(new_sequences[1].end(), segment_i.begin(), segment_i.end());
    new_sequences[1].insert(new_sequences[1].end(), segment_k.begin(), segment_k.end());

    /// Aj CB ik
    new_sequences[2].insert(new_sequences[2].end(), segment_start.begin(), segment_start.end());
    new_sequences[2].insert(new_sequences[2].end(), segment_j.begin(), segment_j.end());
    new_sequences[2].insert(new_sequences[2].end(), segment_i_reverse.begin(), segment_i_reverse.end());
    new_sequences[2].insert(new_sequences[2].end(), segment_k.begin(), segment_k.end());

    /// Aj Ci Bk
    new_sequences[3].insert(new_sequences[3].end(), segment_start.begin(), segment_start.end());
    new_sequences[3].insert(new_sequences[3].end(), segment_j.begin(), segment_j.end());
    new_sequences[3].insert(new_sequences[3].end(), segment_i.begin(), segment_i.end());
    new_sequences[3].insert(new_sequences[3].end(), segment_k.begin(), segment_k.end());

    auto POICoords = map->getPOICoords();
    double full_cost = init_solution.cost, old_cost(0);
    std::vector<double> new_costs(4, CN_INFINITY), new_full_costs(4, 0);
    old_cost = init_solution.joined_paths[agentID].pathlength;
    for(int k = 0; k < 4; k++)
        for(int i = 1; i < new_sequences[0].size(); i++)
            new_costs[k] += h_values[new_sequences[k][i-1]][POICoords[new_sequences[k][i]].first][POICoords[new_sequences[k][i]].second];
    if(useTTD)
    {
        for(int k = 0; k < 4; k++)
            new_costs[k] *= (new_sequences[k].size() - 2);
        old_cost *= (new_sequences[0].size() - 2);
    }

    if(full_cost + *std::min_element(new_costs.begin(), new_costs.end()) - old_cost > best_solution.cost) //if none of the routes provides an estimate better than the cost of the best solution
        return false;
    for(int i = 0; i < 4; i++)
    {
        if(full_cost + new_costs[i] - old_cost > best_solution.cost)
            continue;
        std::map<int, std::vector<int>> new_subgoals;
        new_subgoals[agentID] = new_sequences[i];
        MAPF_solution solution = repairSolution(new_subgoals);
        if(solution.cost < best_solution.cost)
            best_solution = solution;
    }
    return true;
}

bool MAPF_PBS::computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace)
{
//    std::cout << "POI_ID: " << POI_ID << " newAgentID: " << newAgentID << " newPlace: " << newPlace << " oldAgentID: " << oldAgentID << " oldPlace: " << oldPlace << "\n";
    if(v == v1)
        return MAPF_Prioritized::computeRelocateCost(POI_ID, newAgentID, newPlace, oldAgentID, oldPlace);
    auto POICoords = map->getPOICoords();

    //std::cout<<"called relocate\n";
    auto sequence1 = init_input.sequences[newAgentID];
    auto sequence2 = init_input.sequences[oldAgentID];
    sequence1.insert(sequence1.begin() + newPlace, POI_ID);
    sequence2.erase(sequence2.begin() + oldPlace);

    MainType estimate_cost = init_solution.cost;
    if(useTTD)
    {
        double estimate_path_cost(0);
        for(int i = 0; i < newPlace - 1; i++)
            estimate_path_cost += init_solution.paths[newAgentID][i].pathlength;
        for(int i = newPlace; i < sequence1.size(); i++)
            estimate_path_cost += h_values[sequence1[i-1]][POICoords[sequence1[i]].first][POICoords[sequence1[i]].second];
        estimate_cost -= init_solution.joined_paths[newAgentID].pathlength*(sequence1.size() - 2);
        estimate_cost += estimate_path_cost*(sequence1.size() - 2);
        estimate_path_cost = 0;
        for(int i = 0; i < oldPlace - 1; i++)
            estimate_path_cost -= init_solution.paths[oldAgentID][i].pathlength;
        for(int i = oldPlace; i < sequence2.size(); i++)
            estimate_path_cost += h_values[sequence2[i-1]][POICoords[sequence2[i]].first][POICoords[sequence2[i]].second];
        estimate_cost -= init_solution.joined_paths[oldAgentID].pathlength*(sequence2.size() - 2);
        estimate_cost += estimate_path_cost*(sequence2.size() - 2);

    }
    else
    {
        for(int i = newPlace - 1; i < init_solution.paths[newAgentID].size(); i++)
            estimate_cost -= init_solution.paths[newAgentID][i].pathlength;
        for(int i = newPlace; i < sequence1.size(); i++)
            estimate_cost += h_values[sequence1[i-1]][POICoords[sequence1[i]].first][POICoords[sequence1[i]].second];
        for(int i = oldPlace - 1; i < init_solution.paths[oldAgentID].size(); i++)
            estimate_cost -= init_solution.paths[oldAgentID][i].pathlength;
        for(int i = oldPlace; i < sequence2.size(); i++)
            estimate_cost += h_values[sequence2[i-1]][POICoords[sequence2[i]].first][POICoords[sequence2[i]].second];

    }
    if(estimate_cost > best_solution.cost) //new sequence is worse than best one.
        return false;
    std::map<int, std::vector<int>> new_subgoals;
    new_subgoals[newAgentID] = sequence1;
    new_subgoals[oldAgentID] = sequence2;
    MAPF_solution solution = repairSolution(new_subgoals);
    if(solution.cost < best_solution.cost)
        best_solution = solution;
    return true;
}

MAPF_solution MAPF_PBS::repairSolution(std::map<int, std::vector<int>> subgoals_ids)
{
    //std::cout<<"repair solution called\n";
    int num_of_agents = init_input.sequences.size();
    tree.clear();
    auto input = init_input;
    for(auto &seq:subgoals_ids)
        input.sequences[seq.first] = seq.second;
    if(v == v0)
        return computeCost(input);
    PBS_Node start_node;
    start_node.id = 0;
    node_id = 0;
    start_node.priorities = init_solution.pbs_constraints;
    for(int i = 0; i < num_of_agents; i++)
        for(auto &seq:subgoals_ids)
        {
            start_node.priorities.erase(std::make_pair(i, seq.first));
            start_node.priorities.erase(std::make_pair(seq.first, i));
        }

    TopologicalSorting order(input.sequences.size());
    for(const auto &lp:start_node.priorities)
        order.addEdge(lp.first, lp.second);
    if(v == v3 || v == v3b)
    {
        TopologicalSorting extra_order = order;
        std::map<int, std::vector<std::pair<int, int>>, std::greater<int>> constraints;
        for(int i = 0; i < num_of_agents; i++)
            for(auto a:subgoals_ids)
            {
                if(good_constraints[i][a.first] > 0)
                    constraints[good_constraints[i][a.first]].push_back(std::make_pair(i, a.first));
                if(good_constraints[a.first][i] > 0)
                    constraints[good_constraints[a.first][i]].push_back(std::make_pair(a.first, i));
            }
        for(const std::pair<int, std::vector<std::pair<int, int>>> &c:constraints)
            for(std::pair<int, int> p:c.second)
            {
                extra_order.addEdge(p.first, p.second);
                if(!extra_order.isCyclic())
                {
                    order = extra_order;
                    start_node.priorities.insert(p);
                }
                else
                    extra_order = order;//cancel the addition of last edge(priority) that leads to cyclic ordering
            }
    }
    auto priorities = order.sort();
    std::map<int, Path> all_paths;
    auto POICoords = map->getPOICoords();
    start_node.joined_paths.resize(num_of_agents);
    start_node.sequenced_paths.resize(num_of_agents);
    if(v == v2 || v == v3)
    {
        for(int a:priorities)
        {
        std::vector<Path> obstacles;
        for(int i = 0; i < num_of_agents; i++)
            if(start_node.priorities.find(std::make_pair(i,a)) != start_node.priorities.end())
                obstacles.push_back(all_paths[i]);
        std::set<std::pair<int, int>> blocked_nodes;
        if(blockGoals)
            for(int i = 0; i < num_of_agents; i++)
                if(i != a && start_node.priorities.find(std::make_pair(a, i)) == start_node.priorities.end())
                    blocked_nodes.insert({POICoords[input.sequences.at(i).back()].first, POICoords[input.sequences.at(i).back()].second});
        Agent agent = Agent();
        agent.start_i = POICoords[input.sequences.at(a).front()].first;
        agent.start_j = POICoords[input.sequences.at(a).front()].second;
        agent.goal_i = POICoords[input.sequences.at(a).back()].first;
        agent.goal_j = POICoords[input.sequences.at(a).back()].second;
        agent.int_id = a;
        std::vector<Node> subgoals;
        for(auto poi_id:input.sequences.at(a))
        {
            Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
            poi.id = poi_id;
            subgoals.push_back(poi);
        }
        std::vector<Path> sequenced_path = planner.find_full_path(agent, subgoals, map, obstacles, h_values, blocked_nodes);
        if(sequenced_path.empty())
        {
            std::cout<<"ERROR! Cannot repair solution!\n";
            return computeCost(input);
        }
        auto joined_path = makeJoinedPath(sequenced_path);
        for(const std::pair<int, Path> &p:all_paths)
        {
            Conflict c = check_paths(joined_path, p.second);

            if(c.t > 0)
                start_node.conflicts.push_back(c);
        }
        all_paths[a] = joined_path;
        start_node.joined_paths[a] = joined_path;
        start_node.sequenced_paths[a] = sequenced_path;
        start_node.cost += useTTD ? joined_path.TTD : joined_path.pathlength;
        }
    }
    else //v2b and v3b with old paths
    {
        start_node.joined_paths = init_solution.joined_paths;
        start_node.sequenced_paths = init_solution.paths;
        for(auto a:subgoals_ids)
            start_node.joined_paths[a.first] = Path();//remove old paths of the agents that are going to be replanned
        for(auto a:subgoals_ids)
        {
            Agent agent = Agent();
            agent.start_i = POICoords[input.sequences.at(a.first).front()].first;
            agent.start_j = POICoords[input.sequences.at(a.first).front()].second;
            agent.goal_i = POICoords[input.sequences.at(a.first).back()].first;
            agent.goal_j = POICoords[input.sequences.at(a.first).back()].second;
            agent.int_id = a.first;
            std::vector<Node> subgoals;
            for(auto poi_id:input.sequences.at(a.first))
            {
                Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
                poi.id = poi_id;
                subgoals.push_back(poi);
            }
            std::vector<Path> obstacles;
            if(v == v3b)
                for(int i = 0; i < num_of_agents; i++)
                    if(start_node.priorities.count(std::make_pair(i, a.first)) > 0)
                        if(start_node.joined_paths[i].pathfound)
                            obstacles.push_back(start_node.joined_paths[i]);
            std::vector<Path> sequenced_path = planner.find_full_path(agent, subgoals, map, obstacles, h_values, {});
            if(sequenced_path.empty())
            {
                std::cout<<"ERROR! Cannot repair solution!\n";
                return computeCost(input);
            }
            auto joined_path = makeJoinedPath(sequenced_path);
            for(int i = 0; i < num_of_agents; i++)
            {
                if(!start_node.joined_paths[i].pathfound) //if path not found - its either current agent's path, or path of another agent that is not yet replanned
                    continue;
                Conflict c = check_paths(start_node.joined_paths[i], joined_path);
                if(c.t > 0)
                    start_node.conflicts.push_back(c);
            }
            start_node.joined_paths[a.first] = joined_path;
            start_node.sequenced_paths[a.first] = sequenced_path;
        }
        start_node.cost = 0;
        for(auto &p:start_node.joined_paths)
            start_node.cost += useTTD ? p.TTD : p.pathlength;
    }

    tree.add_node(start_node);
    clear_tree = false;
    return computeCost(input);
}

bool MAPF_PBS::init_root(const MAPF_input &input)
{
    PBS_Node root;
    std::vector<Path> sequenced_path;
    Path joined_path;
    auto POICoords = map->getPOICoords();
    for(int i = 0; i < input.sequences.size(); i++)
    {
        Agent agent = Agent();
        agent.start_i = POICoords[input.sequences.at(i).front()].first;
        agent.start_j = POICoords[input.sequences.at(i).front()].second;
        agent.goal_i = POICoords[input.sequences.at(i).back()].first;
        agent.goal_j = POICoords[input.sequences.at(i).back()].second;
        agent.int_id = i;
        std::vector<Node> subgoals;
        for(auto poi_id:input.sequences[i])
        {
            Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
            poi.id = poi_id;
            subgoals.push_back(poi);
        }
        sequenced_path = planner.find_full_path(agent, subgoals, map, {}, h_values);
        if(sequenced_path.empty())
            return false;
        joined_path = makeJoinedPath(sequenced_path);
        for(const Path &p:root.joined_paths)
        {
            Conflict c = check_paths(joined_path, p);

            if(c.t > 0)
                root.conflicts.push_back(c);
        }
        root.joined_paths.push_back(joined_path);
        root.sequenced_paths.push_back(sequenced_path);
        root.cost += useTTD ? joined_path.TTD : joined_path.pathlength;
    }
    root.parent = nullptr;
    root.constraints_num = 0;
    root.colliding_agents = root.conflicts.size();
    root.id = 0;
    node_id = 0;
    tree.add_node(root);
    return true;
}


/*std::vector<Path> MAPF_PBS::repair_solution(PBS_Node &node, const std::vector<Path> &all_paths, const MAPF_input &input)
{
    Path new_path = node.paths[0];
    auto lower_priorities = node.lower_priorities.find(new_path.agent_id);
    if(lower_priorities == node.lower_priorities.end())
        return {new_path};
    std::vector<Path> modified_paths, new_all_paths = all_paths;
    new_all_paths[new_path.agent_id] = new_path;
    TopologicalSorting order(all_paths.size());
    for(const auto &lp:node.lower_priorities)
        for(auto other:lp.second)
            order.addEdge(lp.first, other);
    std::vector<int> sorted_order = order.sort();
    auto POICoords = map->getPOICoords();
    for(unsigned int k = 1; k < sorted_order.size(); k++)
    {
        std::vector<Path> obstacles;
        int cur_agent_id = sorted_order[k];
        auto higher_priorities = node.higher_priorities.find(cur_agent_id);
        if(higher_priorities == node.higher_priorities.end())
            continue; //no agents with higher priority => no collisions => no need to repair the path
        for(auto hp:higher_priorities->second)
            obstacles.push_back(new_all_paths[hp]);
        Conflict conflict;
        for(const Path& obs:obstacles)
        {
            conflict = check_paths(all_paths[cur_agent_id], obs);
            if(conflict.t > 0)
                break;
        }
        if(conflict.t > 0)
        {
            Agent agent = Agent();
            agent.start_i = POICoords[input.sequences.at(i).front()].first;
            agent.start_j = POICoords[input.sequences.at(i).front()].second;
            agent.goal_i = POICoords[input.sequences.at(i).back()].first;
            agent.goal_j = POICoords[input.sequences.at(i).back()].second;
            agent.int_id = k;
            std::vector<Node> subgoals;
            for(auto poi_id:input.sequences[i])
            {
                Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
                poi.id = poi_id;
                subgoals.push_back(poi);
            }
            sequenced_path = planner.find_full_path(agent, subgoals, map, obstacles, h_values);
            if(sequenced_path.empty())
                return false;
            joined_path = makeJoinedPath(sequenced_path);
            new_all_paths[cur_agent_id] = joined_path;
            modified_paths.push_back(all_paths[cur_agent_id]);
        }
    }
    node.conflicts.clear();
    for(unsigned int i=0; i < all_paths.size(); i++)
    {
        //auto lp = node.lower_priorities.find(sorted_order[i]);
        for(unsigned int j = i + 1; j < all_paths.size(); j++)
        {

            //if(lp != node.lower_priorities.end() && lp->second.find(sorted_order[j]) != lp->second.end())
            //    continue;//this pair can't have collisions
            Conflict conflict = check_paths(new_all_paths[i], new_all_paths[j]);
            if(conflict.t > 0)
                node.conflicts.push_back(conflict);
        }
    }
    node.colliding_agents = node.conflicts.size();
    node.cost = 0;
    for(const Path& p: new_all_paths)
        node.cost += p.cost;
    return new_all_paths;
}*/


double MAPF_PBS::get_cost(PBS_Node node, int agent_id)
{
    while(node.parent != nullptr)
    {
        if(node.joined_paths.begin()->agent_id == agent_id)
            return useTTD ? node.joined_paths.begin()->TTD : node.joined_paths.begin()->pathlength;
        node = *node.parent;
    }
    return useTTD ? node.joined_paths.at(agent_id).TTD : node.joined_paths.at(agent_id).pathlength;
}

Conflict MAPF_PBS::check_paths(Path pathA, Path pathB)
{
    unsigned int i(0), j(0);
    //std::cout<<"checking paths "<<pathA.agent_id<<" "<<pathB.agent_id<<"\n";
    while(i < pathA.sections.size() - 1 || j < pathB.sections.size())
    {
        if(i == pathA.sections.size() - 1)
        {
            if(pathA.sections[i].i == pathB.sections[j].i && pathA.sections[i].j == pathB.sections[j].j)
                return Conflict(pathA.agent_id, pathB.agent_id,
                                Move(j, pathA.sections[i].i, pathA.sections[i].j, pathA.sections[i].i, pathA.sections[i].j),
                                Move(j, pathB.sections[j].i, pathB.sections[j].j, pathB.sections[j].i, pathB.sections[j].j), j);
            j++;
        }
        else if(j == pathB.sections.size() - 1)
        {
            if(pathA.sections[i].i == pathB.sections[j].i && pathA.sections[i].j == pathB.sections[j].j)
                return Conflict(pathA.agent_id, pathB.agent_id,
                                Move(i, pathA.sections[i].i, pathA.sections[i].j, pathA.sections[i].i, pathA.sections[i].j),
                                Move(i, pathB.sections[j].i, pathB.sections[j].j, pathB.sections[j].i, pathB.sections[j].j), i);
            i++;
        }
        else
        {
            if(pathA.sections[i].i == pathB.sections[j].i && pathA.sections[i].j == pathB.sections[j].j)
                return Conflict(pathA.agent_id, pathB.agent_id,
                                Move(i, pathA.sections[i].i, pathA.sections[i].j, pathA.sections[i].i, pathA.sections[i].j),
                                Move(j, pathB.sections[j].i, pathB.sections[j].j, pathB.sections[j].i, pathB.sections[j].j), i);
            if(pathA.sections[i].i == pathB.sections[j+1].i && pathA.sections[i].j == pathB.sections[j+1].j
                    && pathA.sections[i+1].i == pathB.sections[j].i && pathA.sections[i+1].j == pathB.sections[j].j)
                return Conflict(pathA.agent_id, pathB.agent_id,
                                Move(i, pathA.sections[i].i, pathA.sections[i].j, pathA.sections[i+1].i, pathA.sections[i+1].j),
                                Move(j, pathB.sections[j].i, pathB.sections[j].j, pathB.sections[j+1].i, pathB.sections[j+1].j), i);
            i++;
            j++;
        }
    }
    return Conflict();
}

int MAPF_PBS::getAg(std::vector<int> goals_in_path, MAPF_solution solution){

    int ret = 0;

    for(int k=0; k<solution.paths.size(); k++)
    {
      /* std::vector<std::pair<int, int>> single_path; */
      /* int curr_makespan = 0; */
      int route_length = 0;
      // substract 2 for starting and ending depots I think
      route_length = solution.joined_paths[k].pathlength - 2;

        /* for(int i = 0; i < solution.paths[k].size(); i++){ */
        /*     for(int j = 0; j < solution.paths[k][i].sections.size(); j++) */
        /*     { */
        /*         if(i>0 && j==0) */
        /*             continue; */
        /*       /1* single_path.push_back(std::make_pair(solution.paths[k][i].sections[j].j, solution.paths[k][i].sections[j].i)); *1/ */
        /*       /1* ++curr_makespan; *1/ */
        /*         ++route_length; */
        /*     } */
        /* } */


        ret += route_length*goals_in_path[k];
        /* if (curr_makespan > makespan){ */
        /*   makespan = curr_makespan; */
        /* } */
    }
    /* for(int k=0; k<solution.paths.size(); k++){ */

    /* } */
    /* solution. */

    return ret;

}
