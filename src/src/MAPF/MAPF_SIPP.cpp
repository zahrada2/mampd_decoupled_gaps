#include "MAPF_SIPP.hpp"

using namespace cvrplib;

bool MAPF_Prioritized::setMap(boost::shared_ptr<MapData> map_){
    map = map_;
    if (map != nullptr){
        calculateHValues();
        return true;
    } else {
        return false;
    }
}

MainType MAPF_Prioritized::estimateCost(MAPF_input input){
    auto POICoords = map->getPOICoords();
    double cost(0);

    for(auto i:input.sequences)
        for(int k = 0; k < i.size() - 1; k++)
            cost += h_values[i[k]][POICoords[i[k+1]].first][POICoords[i[k+1]].second];
    return cost;
}

void MAPF_Prioritized::setInitSolution(MAPF_solution solution, MAPF_input input)
{
    init_solution = solution;
    init_input = input;
    best_solution = solution;
}

bool MAPF_Prioritized::lastInstanceSolutionFound()
{
    return lastSolutionFound;
}

void MAPF_Prioritized::saveInstanceIntoXML(std::string fileName, MAPF_input& input, MapData& map_data)
{
    std::ofstream out(fileName.c_str());
    out<<"<?xml version=\"1.0\" ?>\n<root>\n   <agents>\n";
    auto POICoords = map_data.getPOICoords();
    for(int i=0; i<input.sequences.size(); i++)
    {
        auto s = input.sequences[i];
        out<<"      <agent id=\""<<i<<"\" start_i=\""<<POICoords[s[0]].first<<"\" start_j=\""<<POICoords[s[0]].second<<"\" goal_i=\""<<POICoords[s.back()].first<<"\" goal_j=\""<<POICoords[s.back()].second<<"\" />\n";
    }
    out<<"  </agents>\n";
    out<<"  <sequences>\n";
    for(int i=0; i<input.sequences.size(); i++)
    {
        auto s = input.sequences[i];
        out<<"      <sequence agent_id=\""<<i<<"\">\n";
        for(auto k:s)
            out<<"          <subgoal id=\""<<k<<"\" i=\""<<POICoords[k].first<<"\" j=\""<<POICoords[k].second<<"\" wait=\""<<map_data.getWaitTime(k)<<"\"/>\n";
        out<<"      </sequence>\n";
    }
    out<<"  </sequences>\n";
    out<<"</root>";
    out.close();
}

int MAPF_Prioritized::getAg(std::vector<int> goals_in_path, MAPF_solution solution){

    int ret = 0;


    for(int k=0; k<solution.paths.size(); k++)
    {
      /* std::vector<std::pair<int, int>> single_path; */
      /* int curr_makespan = 0; */
      int route_length = 0;
      // substract 2 for starting and ending depots I think
      route_length = solution.joined_paths[k].pathlength - 2;



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

void MAPF_Prioritized::saveSolutionIntoXML(std::string fileName, MAPF_solution solution)
{
    std::vector<Path> fullpaths;
    for(int i = 0; i<solution.paths.size(); i++)
    {
        Path p;
        for(int k = 0; k<solution.paths[i].size(); k++)
            for(auto n : solution.paths[i][k].sections)
                p.sections.push_back(n);
        fullpaths.push_back(p);
    }

    std::ofstream out(fileName.c_str());
    out<<"<?xml version=\"1.0\" ?>\n<root>\n   <map>\n";
    out<<"      <grid width=\""<<map->getMapWidth()<<"\" height=\""<<map->getMapHeight()<<"\">\n";
    for(int i = 0; i < map->getMapHeight(); i++)
    {
        out<<"          <row>";
        for(int j = 0; j < map->getMapWidth(); j++)
            out<<!map->cellIsTraversable(j,i)<<" ";
        out<<"</row>\n";
    }
    out<<"      </grid>\n   </map>\n";
    for(int k=0; k<solution.paths.size(); k++)
    {
        out<<"   <agent>\n       <path>\n";
        for(int i = 0; i < fullpaths[k].sections.size()-1; i++)
            {
                if(fullpaths[k].sections[i+1].g == fullpaths[k].sections[i].g)
                    continue;
                out<<"          <section start.x=\""<<fullpaths[k].sections[i].j
                                   <<"\" start.y=\""<<fullpaths[k].sections[i].i
                                   <<"\" goal.x=\""<<fullpaths[k].sections[i+1].j
                                   <<"\" goal.y=\""<<fullpaths[k].sections[i+1].i
                                   <<"\" duration=\""<<fullpaths[k].sections[i+1].g - fullpaths[k].sections[i].g<<"\"/>\n";
            }
        out<<"      </path>\n   </agent>\n";
    }
    out<<"</root>";
    out.close();
}

void MAPF_Prioritized::saveSolutionMapfIR(std::string fileName, MAPF_solution solution, std::vector<uint16_t> starting_depots, std::vector<uint16_t> ending_depots,
        int numVehicles, std::string map_name, std::vector<std::pair<int,int>> vertex_locations, int cost_lower_bound, int final_cost, std::string method){
    auto POICoords = map->getPOICoords();
    std::ofstream out(fileName.c_str());
    out << "instance=" << map_name << "\n" ;
    out << "agents=" << numVehicles << "\n";
    out << "map_file=" << "mapf_vrp/" << map_name << ".map" << "\n";
    out << "solver=" << method << "\n";
    // TODO: leave hardcoded?
    out << "solved=1\n";
    out << "soc=" << final_cost << "\n";
    out << "lb_soc=" << cost_lower_bound << "\n";
    // calculate makespan
    int makespan = 0;

    std::vector<std::vector<std::pair<int, int>>> paths;
    for(int k=0; k<solution.paths.size(); k++)
    {
      std::vector<std::pair<int, int>> single_path;
      int curr_makespan = 0;
        for(int i = 0; i < solution.paths[k].size(); i++){
            for(int j = 0; j < solution.paths[k][i].sections.size(); j++)
            {
                if(i>0 && j==0)
                    continue;
              single_path.push_back(std::make_pair(solution.paths[k][i].sections[j].j, solution.paths[k][i].sections[j].i));
              ++curr_makespan;
            }
        }
        if (curr_makespan > makespan){
          makespan = curr_makespan;
        }
        paths.push_back(single_path);
    }


    std::vector<std::vector<std::pair<int, int>>> paths_inverted;
    std::vector<std::pair<int,int>> path_makespan_point(numVehicles, std::make_pair(-1, -1));
    for (uint16_t makespan_point = 0; makespan_point < makespan; ++makespan_point){
      paths_inverted.push_back(path_makespan_point);
    }
    for (int i = 0; i < paths.size(); ++i){
      for (int j = 0; j < makespan; ++j){
        if (j >= paths[i].size()){
          paths_inverted[j][i] = paths_inverted[j-1][i];
        } else {
          paths_inverted[j][i] = paths[i][j];
        }
      }
    }


    out << "makespan=" << makespan - 1 << "\n";
    out << "lb_makespan=1\n";
    out << "comp_time=100\n";
    out << "starts=";
    for (int i = 0; i < starting_depots.size(); ++i){
      auto coord = POICoords[starting_depots[i]];
      out << "(" << coord.second << "," << coord.first << "),";
    }
    out << "\n";
    out << "goals=";
    for (int i = 0; i < starting_depots.size(); ++i){
      auto coord = POICoords[ending_depots[i]];
      out << "(" << coord.second << "," << coord.first << "),";
    }
    out << "\n";
    out << "solution=\n";
    for (int i = 0; i < makespan; ++i){
      out << i << ":";
      for (int j = 0; j < numVehicles; ++j){
        out << "(" << paths_inverted[i][j].first << "," << paths_inverted[i][j].second << "),";
      }
      out << "\n";
    }
    out.close();
}


void MAPF_Prioritized::saveSolutionMampdIR(std::string fileName, MAPF_solution solution, int numVehicles, std::string map_name, std::string instance_name, std::vector<std::pair<int,int>> vertex_locations, double vrp_cost, std::vector<std::vector<int>> assignments, MAPF_input mapf_task){
    auto POICoords = map->getPOICoords();
    std::ofstream out(fileName.c_str());
    out << "instance=" << instance_name << "\n" ;
    out << "agents=" << numVehicles << "\n";
    out << "map_file=" <<  map_name << ".map\n";
    out << "solver=" << "LNS" << "\n";
    out << "solved=1\n";
    out << "cost=" << solution.cost << "\n";
    out << "lb_cost=" << vrp_cost << "\n";
    // calculate makespan
    int makespan = 0;

    std::vector<std::vector<std::pair<int, int>>> paths;
    for(int k=0; k<solution.joined_paths.size(); k++)
    {
        std::vector<std::pair<int, int>> single_path;
        for(int i = 0; i < solution.joined_paths[k].sections.size(); i++)
            single_path.emplace_back(solution.joined_paths[k].sections[i].j, solution.joined_paths[k].sections[i].i);
        int makespan_sol = solution.joined_paths[k].pathlength;
        makespan = std::max(makespan, makespan_sol);
        paths.push_back(single_path);
    }


    std::vector<std::vector<std::pair<int, int>>> paths_inverted;
    std::vector<std::pair<int,int>> path_makespan_point(numVehicles, std::make_pair(-1, -1));
    for (uint16_t makespan_point = 0; makespan_point < makespan; ++makespan_point){
        paths_inverted.push_back(path_makespan_point);
    }
    int cost = 0;
    for (int i = 0; i < paths.size(); ++i){
        for (int j = 0; j < makespan; ++j){
            if (j >= paths[i].size()){
                paths_inverted[j][i] = paths_inverted[j-1][i];
            } else {
                paths_inverted[j][i] = paths[i][j];
                cost++;
            }
        }
    }

    out << "makespan=" << makespan << "\n";
    out << "lb_makespan=1\n";
    out << "comp_time=100\n";

    out << "starts=";
    for (int i = 0; i < assignments.size(); ++i){
        auto coord = POICoords[assignments[i].front()];
        out << "(" << coord.second << "," << coord.first << "),";
    }
    out << "\n";
    out << "goals=";
    for (int i = 0; i < assignments.size(); ++i){
        auto coord = POICoords[assignments[i].back()];
        out << "(" << coord.second << "," << coord.first << "),";
    }
    out << "\n";
    out << "actions=\n";
    for(int i = 0; i < assignments.size(); ++i){
      for(int j = 0; j < assignments[i].size(); ++j){
        int release_time = 0;
        char action = '?';
        int task_id = -1;
        if(j == 0){
          action = 'S';
          task_id = -1;
        } else if (j == (assignments[i].size() - 1)){
          action = 'E';
          task_id = -1;
        } else {
          action = 'P';
          task_id = assignments[i][j];
        }
        auto coord = POICoords[assignments[i][j]];
        out << "(" << coord.second << "," << coord.first << "," << task_id << "," << release_time << "," << action << "),";

      } // iterating over subgoals
      out << "\n";
    } // iterating over agents

    out << "solution=\n";
    for (int i = 0; i < makespan; ++i){
        out << i << ":";
        for (int j = 0; j < numVehicles; ++j){
          /* std::cout << "agent: "<< j << " inserting position " << paths_inverted[i][j].first << "," << paths_inverted[i][j].second << std::endl; */
            out << "(" << paths_inverted[i][j].first << "," << paths_inverted[i][j].second << "),";
        }
        out << "\n";
    }
    out.close();
}


MAPF_solution MAPF_Prioritized::getBestSolution()
{
    return best_solution;
}

bool MAPF_Prioritized::computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace){
//    std::cout << "POI_ID: " << POI_ID << " newAgentID: " << newAgentID << " newPlace: " << newPlace << " oldAgentID: " << oldAgentID << " oldPlace: " << oldPlace << "\n";
    auto POICoords = map->getPOICoords();

    //std::cout<<"called relocate\n";
    auto sequence1 = init_input.sequences[newAgentID];
    auto sequence2 = init_input.sequences[oldAgentID];
    if(newAgentID == oldAgentID)
    {
        sequence1.erase(sequence1.begin() + oldPlace);
        if(newPlace > oldPlace)
            sequence1.insert(sequence1.begin() + newPlace + 1, POI_ID);
        else
            sequence1.insert(sequence1.begin() + newPlace, POI_ID);
    }
    else
    {
        sequence1.insert(sequence1.begin() + newPlace, POI_ID);
        sequence2.erase(sequence2.begin() + oldPlace);
    }

    MainType estimate_cost = init_solution.cost;
    if(useTTD)
    {
        double estimate_path_cost(0);
        if(newAgentID == oldAgentID)
        {
            for(int i = 0; i < std::min(newPlace, oldPlace) - 1; i++)
                estimate_path_cost += init_solution.paths[newAgentID][i].pathlength;
            for(int i = std::min(newPlace, oldPlace); i < sequence1.size(); i++)
                estimate_path_cost += h_values[sequence1[i-1]][POICoords[sequence1[i]].first][POICoords[sequence1[i]].second];
            estimate_cost -= init_solution.joined_paths[newAgentID].pathlength*(sequence1.size() - 2);
            estimate_cost += estimate_path_cost*(sequence1.size() - 2);
        }
        else
        {
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

    }
    else
    {
        if(newAgentID == oldAgentID)
        {
            for(int i = std::min(newPlace, oldPlace) - 1; i < init_solution.paths[newAgentID].size(); i++)
                estimate_cost -= init_solution.paths[newAgentID][i].pathlength;
            for(int i = std::min(newPlace, oldPlace); i < sequence1.size(); i++)
                estimate_cost += h_values[sequence1[i-1]][POICoords[sequence1[i]].first][POICoords[sequence1[i]].second];
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
    }
    if(estimate_cost > best_solution.cost) //new sequence is worse than best one.
        return false;
    last_replanned_agent = -1;
    std::vector<Path> obstacles;
    for(int i = 0; i < init_solution.joined_paths.size(); i++)
    {
        if(i == newAgentID || i == oldAgentID)
            continue;
        obstacles.push_back(init_solution.joined_paths[i]);
    }
    double new_cost1(CN_INFINITY), new_cost2(CN_INFINITY);
    std::vector<Path> new_path1, new_path2;
    Path joined_path1, joined_path2;
    if(newAgentID == oldAgentID)
    {
        new_path1 = findPartialPath(sequence1, newAgentID, obstacles);
        if(!new_path1.empty())
        {
            joined_path1 = makeJoinedPath(new_path1);
            joined_path2 = joined_path1;
            new_cost1 = useTTD ? joined_path1.TTD : joined_path1.pathlength;
            new_cost2 = 0; //just to be sure that we will not add the cost twice
        }
    }
    else
    {

        new_path1 = findPartialPath(sequence1, newAgentID, obstacles);
        if(!new_path1.empty())
        {

            joined_path1 = makeJoinedPath(new_path1);
            new_cost1 = useTTD ? joined_path1.TTD : joined_path1.pathlength;
            obstacles.push_back(joined_path1);
            new_path2 = findPartialPath(sequence2, oldAgentID, obstacles);
            if(!new_path2.empty())
            {
                joined_path2 = makeJoinedPath(new_path2);

                new_cost2 = useTTD ? joined_path2.TTD : joined_path2.pathlength;
            }
        }
    }

    double full_cost(0);
    for(int i = 0; i < init_solution.joined_paths.size(); i++)
    {
        if(i == newAgentID || i == oldAgentID)
            continue;

        full_cost += useTTD ? init_solution.joined_paths[i].TTD : init_solution.joined_paths[i].pathlength;
    }
    full_cost += new_cost1;
    full_cost += new_cost2;

    if(full_cost < CN_INFINITY)
        found_costs.push_back(full_cost);
    if(full_cost < best_solution.cost)
    {
        best_solution = init_solution;
        best_solution.cost = full_cost;
        best_solution.paths[oldAgentID] = new_path2;
        best_solution.paths[newAgentID] = new_path1;
        best_solution.joined_paths[oldAgentID] = joined_path2;
        best_solution.joined_paths[newAgentID] = joined_path1;
        best_solution.input = init_input;
        best_solution.input.sequences[oldAgentID] = sequence2;
        best_solution.input.sequences[newAgentID] = sequence1;
    }
    else if(full_cost >= CN_INFINITY) //if partial replanning failed - launch full replaniing with rescheduling mechanism enabled
    {
        auto input = init_input;
        input.sequences[oldAgentID] = sequence2;
        input.sequences[newAgentID] = sequence1;
        MAPF_solution solution = computeCost(input);
        if(solution.cost < best_solution.cost)
            best_solution = solution;
    }
    return true;
}

bool MAPF_Prioritized::computeTwoOpt(int agentID, int startPlace, int endPlace)
{
    //std::cout<<"called 2opt\n";
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

    std::vector<Path> obstacles;
    for(int i = 0; i < init_solution.joined_paths.size(); i++)
    {
        if(i == agentID)
            continue;
        obstacles.push_back(init_solution.joined_paths[i]);

    }
    last_replanned_agent = agentID;
    double new_cost(CN_INFINITY);
    std::vector<Path> new_path;
    Path joined_path;
    new_path = findPartialPath(sequence, agentID, obstacles);
    if(!new_path.empty())
    {
        joined_path = makeJoinedPath(new_path);
        new_cost = useTTD ? joined_path.TTD : joined_path.pathlength;
    }

    MainType full_cost(init_solution.cost);
    full_cost -= useTTD ? init_solution.joined_paths[agentID].TTD : init_solution.joined_paths[agentID].pathlength;
    full_cost += new_cost;
    if(full_cost < CN_INFINITY)
        found_costs.push_back(full_cost);
    if(full_cost < best_solution.cost)
    {
        best_solution = init_solution;
        best_solution.cost = full_cost;
        best_solution.paths[agentID] = new_path;
        best_solution.joined_paths[agentID] = joined_path;
        best_solution.input = init_input;
        best_solution.input.sequences[agentID] = sequence;
    }
    else if(full_cost >= CN_INFINITY) //if partial replanning failed - launch full replaniing with rescheduling mechanism enabled
    {
        auto input = init_input;
        input.sequences[agentID] = sequence;
        last_replanned_agent = -1;
        MAPF_solution solution = computeCost(input);
        if(solution.cost < best_solution.cost)
            best_solution = solution;
    }
    return true;
}

bool MAPF_Prioritized::computeThreeOpt(int agentID, int placeI, int placeJ, int placeK)
{
    //std::cout<<"called 3opt\n";
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
    std::vector<std::vector<Path>> new_paths(4, std::vector<Path>());
    std::vector<Path> joined_paths(4, Path());
    std::vector<Path> obstacles;
    for(int i = 0; i < init_solution.joined_paths.size(); i++)
    {
        if(i == agentID)
            continue;
        obstacles.push_back(init_solution.joined_paths[i]);

    }
    last_replanned_agent = agentID;
    for(int k = 0; k < 4; k++)
    {
        new_paths[k] = findPartialPath(new_sequences[k], agentID, obstacles);
        if(!new_paths[k].empty())
        {
            joined_paths[k] = makeJoinedPath(new_paths[k]);
            new_costs[k] = useTTD ? joined_paths[k].TTD : joined_paths[k].pathlength;
        }
    }

    for(int k = 0; k < 4; k++)
    {
        if(new_costs[k] >= CN_INFINITY) //if partial replanning failed - launch full replanning with rescheduling mechanism enabled
        {
            auto input = init_input;
            input.sequences[agentID] = new_sequences[k];
            last_replanned_agent = -1;
            MAPF_solution solution = computeCost(input);
            new_full_costs[k] = solution.cost;
            if(solution.cost < best_solution.cost)
                best_solution = solution;
        }
        if(new_full_costs[k] == 0)
        {
            new_full_costs[k] = full_cost - (useTTD ? init_solution.joined_paths[agentID].TTD : init_solution.joined_paths[agentID].pathlength) + new_costs[k];
            found_costs.push_back(new_full_costs[k]);
        }
        if(new_full_costs[k] < best_solution.cost)
        {
            best_solution = init_solution;
            best_solution.cost = new_full_costs[k];
            best_solution.paths[agentID] = new_paths[k];
            best_solution.joined_paths[agentID] = joined_paths[k];
            best_solution.input = init_input;
            best_solution.input.sequences[agentID] = new_sequences[k];
        }
    }
    return true;
}

bool MAPF_Prioritized::computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB)
{
    //std::cout<<"called exchange\n";
    auto sequenceA = init_input.sequences[agentA_ID];
    auto sequenceB = init_input.sequences[agentB_ID];
    std::swap(sequenceA[placeA], sequenceB[placeB]);
    std::vector<Path> pathA, pathB;
    Path joined_pathA, joined_pathB;
    double new_costA(0), new_costB(0), old_costA(0), old_costB(0), full_cost(0), estimate_costA(0), estimate_costB(0);
    old_costA = init_solution.joined_paths[agentA_ID].pathlength;
    old_costB = init_solution.joined_paths[agentB_ID].pathlength;
    auto POICoords = map->getPOICoords();

    for(int i = 0; i < placeA - 1; i++)
        estimate_costA += init_solution.paths[agentA_ID][i].pathlength;
    for(int i = 0; i < placeB - 1; i++)
        estimate_costB += init_solution.paths[agentB_ID][i].pathlength;
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
    last_replanned_agent = -1;

    std::vector<Path> obstacles;
    for(int i = 0; i < init_solution.joined_paths.size(); i++)
    {
        if(i == agentA_ID || i == agentB_ID)
            continue;
        obstacles.push_back(init_solution.joined_paths[i]);
    }

    pathA = findPartialPath(sequenceA, agentA_ID, obstacles);
    if(!pathA.empty())
    {
        joined_pathA = makeJoinedPath(pathA);
        new_costA = useTTD ? joined_pathA.TTD : joined_pathA.pathlength;
        obstacles.push_back(joined_pathA);
    }
    pathB = findPartialPath(sequenceB, agentB_ID, obstacles);
    if(!pathB.empty())
    {
        joined_pathB = makeJoinedPath(pathB);
        new_costB = useTTD ? joined_pathB.TTD : joined_pathB.pathlength;
    }

    full_cost = init_solution.cost + new_costA + new_costB;
    full_cost -= useTTD ? init_solution.joined_paths[agentA_ID].TTD : init_solution.joined_paths[agentA_ID].pathlength;
    full_cost -= useTTD ? init_solution.joined_paths[agentB_ID].TTD : init_solution.joined_paths[agentB_ID].pathlength;

    if(full_cost < CN_INFINITY)
        found_costs.push_back(full_cost);
    if(full_cost < best_solution.cost)
    {
        best_solution = init_solution;
        best_solution.cost = full_cost;
        best_solution.paths[agentA_ID] = pathA;
        best_solution.paths[agentB_ID] = pathB;
        best_solution.joined_paths[agentA_ID] = joined_pathA;
        best_solution.joined_paths[agentB_ID] = joined_pathB;
        best_solution.input = init_input;
        best_solution.input.sequences[agentA_ID] = sequenceA;
        best_solution.input.sequences[agentB_ID] = sequenceB;
    }
    else if(full_cost >= CN_INFINITY)
    {
        auto input = init_input;
        input.sequences[agentA_ID] = sequenceA;
        input.sequences[agentB_ID] = sequenceB;
        MAPF_solution solution = computeCost(input);
        if(solution.cost < best_solution.cost)
            best_solution = solution;
    }
    return true;
}

Path MAPF_Prioritized::makeJoinedPath(std::vector<Path> sequenced_path)
{
    Path joined_path = sequenced_path.front();
    for(int k = 1; k < sequenced_path.size(); k++)
    {
        joined_path.pathlength += sequenced_path[k].pathlength;
        for(int n = 1; n < sequenced_path[k].sections.size(); n++)
            joined_path.sections.push_back(sequenced_path[k].sections[n]);
    }
    joined_path.TTD = joined_path.pathlength*(sequenced_path.size() - 1);
    for(int k = 0; k < sequenced_path.size() - 1; k++)
        joined_path.TTD -= map->getMinimalDepotDistance(sequenced_path[k].goalPOI_ID);
    return joined_path;
}

std::vector<Path> MAPF_Prioritized::findPartialPath(std::vector<int> subgoals_ids, int agent_id, std::vector<Path> obstacles)
{
    curagent = Agent();
    auto POICoords = map->getPOICoords();
    curagent.start_i = POICoords[subgoals_ids.front()].first;
    curagent.start_j = POICoords[subgoals_ids.front()].second;
    curagent.goal_i = POICoords[subgoals_ids.back()].first;
    curagent.goal_j = POICoords[subgoals_ids.back()].second;
    curagent.int_id = agent_id;
    std::vector<Node> subgoals;
    for(auto poi_id:subgoals_ids)
    {
        Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
        poi.id = poi_id;
        subgoals.push_back(poi);
    }
    auto full_path = planner.find_full_path(curagent, subgoals, map, obstacles, h_values);
    return full_path;
}

MAPF_solution MAPF_Prioritized::computeCost(MAPF_input input){
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    last_replanned_agent = -1;
    int num_of_agents = input.sequences.size();
    bool solution_found(false);
    int bad_i(0);
    double timespent(0);
    priorities.clear();
    auto POICoords = map->getPOICoords();
    MAPF_solution solution;
    solution.input = input;
    solution.paths.resize(num_of_agents);
    solution.joined_paths.resize(num_of_agents);
    solution.cost = 0;
    setPriorities(input);
    do
    {
        std::vector<Path> obstacles;
        for(unsigned int numOfCurAgent = 0; numOfCurAgent < num_of_agents; numOfCurAgent++)
        {
            int k = current_priorities[numOfCurAgent];
            curagent = Agent();
            curagent.start_i = POICoords[input.sequences.at(k).front()].first;
            curagent.start_j = POICoords[input.sequences.at(k).front()].second;
            curagent.goal_i = POICoords[input.sequences.at(k).back()].first;
            curagent.goal_j = POICoords[input.sequences.at(k).back()].second;
            curagent.int_id = k;
            std::vector<Node> subgoals;
            for(auto poi_id:input.sequences[k])
            {
                Node poi(POICoords[poi_id].first, POICoords[poi_id].second);
                poi.id = poi_id;
                subgoals.push_back(poi);
            }
            std::set<std::pair<int, int>> blocked_nodes;
            if(blockGoals)
                for(int n = numOfCurAgent + 1; n < num_of_agents; n++)
                    blocked_nodes.insert({POICoords[input.sequences.at(current_priorities[n]).back()].first, POICoords[input.sequences.at(current_priorities[n]).back()].second});
            std::vector<Path> fullpath = planner.find_full_path(curagent, subgoals, map, obstacles, h_values, blocked_nodes);

            if(fullpath.empty())
            {
                bad_i = current_priorities[numOfCurAgent];
                break;
            }
            solution.paths[k] = fullpath;
            solution.joined_paths[k] = makeJoinedPath(fullpath);
            solution.cost += useTTD ? solution.joined_paths[k].TTD : solution.joined_paths[k].pathlength;

            obstacles.push_back(solution.joined_paths[k]);
            if(numOfCurAgent + 1 == input.sequences.size())
                solution_found = true;
        }

#ifdef __linux__
    gettimeofday(&end, NULL);
    timespent = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    timespent = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        if(timespent > config.timelimit)
            break;
    } while(changePriorities(bad_i) && !solution_found);


    std::cout<<"Called Prioritized computeCost "<<solution.cost<<"\n";
    if(solution_found)
    {
        lastSolutionFound = true;
    }
    else
    {
        lastSolutionFound = false;
        solution.cost = CN_INFINITY;
    }
    checkSolution(solution);

    //saveInstanceIntoXML("prioritized_instance"+std::to_string(int(solution.cost))+".xml", input, *map);
    //saveSolutionIntoXML("prioritized_solution"+std::to_string(int(solution.cost))+".xml", solution);
    found_costs.push_back(solution.cost);
    return solution;
}

void MAPF_Prioritized::checkSolution(MAPF_solution solution)
{
    std::vector<Path> fullpaths;
    for(int i = 0; i<solution.paths.size(); i++)
    {
        Path p;
        for(int k = 0; k<solution.paths[i].size(); k++)
        {
            for(auto n : solution.paths[i][k].sections)
            {
                p.sections.push_back(n);
                //std::cout<<n.i<<" "<<n.j<<" "<<n.g<<"\n";
            }
            //std::cout<<"________\n";
        }
        fullpaths.push_back(p);
    }
    for(int i = 0; i<fullpaths.size()-1; i++)
    {
        Path pathA = fullpaths[i];
        for(int j = i+1; j<fullpaths.size(); j++)
        {
            Path pathB = fullpaths[j];
            int step = 0;
            while(step < fmax(pathA.pathlength, pathB.pathlength))
            {
                if(pathA.sections[step].i == pathB.sections[step].i && pathA.sections[step].j == pathB.sections[step].j)
                    std::cout<<"ERROR! agents "<<i<<" and "<<j<<" are colliding on step "<<step;
            }
        }
    }
}

void MAPF_Prioritized::calculateHValues()
{
    auto POI = map->getPOICoords();
    h_values.resize(POI.size());
    std::list<Node> open;
    std::set<int> visited;
    for(int p = 0; p < POI.size(); p++)
    {
        h_values[p].resize(map->getMapHeight(), std::vector<int>(map->getMapWidth(), -1));
        open.clear();
        visited.clear();
        Node curNode(POI[p].first, POI[p].second, 0, 0);
        open.push_back(curNode);
        while(!open.empty())
        {
            curNode = open.front();
            open.pop_front();
            h_values[p][curNode.i][curNode.j] = curNode.g;
            auto moves = map->getValidMoves(curNode.i, curNode.j);
            for(auto m:moves)
            {
                Node newNode(curNode.i + m.first, curNode.j + m.second, curNode.g + 1, curNode.g + 1);
                newNode.id = newNode.i * map->getMapWidth() + newNode.j;
                if(h_values[p][newNode.i][newNode.j] >= 0)
                    continue;
                if(visited.find(newNode.id) != visited.end())
                    continue;
                visited.insert(newNode.id);
                open.push_back(newNode);
            }
        }
    }
}

void MAPF_Prioritized::setPriorities(MAPF_input input)
{
    int num_of_agents = input.sequences.size();
    current_priorities.clear();
    current_priorities.resize(num_of_agents, -1);
    if(config.initialprioritization == CN_IP_FIFO)
        for(int i = 0; i < num_of_agents; i++)
            current_priorities[i] = i;
    else if(config.initialprioritization != CN_IP_RANDOM)
    {
        std::vector<double> dists(num_of_agents, -1);
        for(int i = 0; i < num_of_agents; i++)
            for(int j = 0; j < input.sequences[i].size()-1; j++)
                dists[i] += 0;//map->getCost(input.sequences[i][j], input.sequences[i][j+1]);
        int k = num_of_agents - 1;
        while(k >= 0)
        {
            double mindist = CN_INFINITY;
            int min_i = -1;
            for(unsigned int i = 0; i < dists.size(); i++)
                if(mindist > dists[i])
                {
                    min_i = i;
                    mindist = dists[i];
                }
            if(config.initialprioritization == CN_IP_LONGESTF)
                current_priorities[k] = min_i;
            else
                current_priorities[num_of_agents - k - 1] = min_i;
            dists[min_i] = CN_INFINITY;
            k--;
        }
    }
    else //random
    {
        for(int i = 0; i < num_of_agents; i++)
            current_priorities[i] = i;
        std::mt19937 g(seed);
        std::shuffle(current_priorities.begin(), current_priorities.end(), g);
    }
}

bool MAPF_Prioritized::changePriorities(int bad_i)
{
    if(config.rescheduling == CN_RE_NO)
        return false;
    priorities.push_back(current_priorities);
    if(config.rescheduling == CN_RE_RULED) //rises the piority of the agent that can't find its path
    {
        for(auto it = current_priorities.begin(); it != current_priorities.end(); it++)
            if(*it == bad_i)
            {
                current_priorities.erase(it);
                current_priorities.insert(current_priorities.begin(), bad_i);
                break;
            }
        for(unsigned int i = 0; i < priorities.size(); i++)
            for(unsigned int j = 0; j < priorities[i].size(); j++)
            {
                if(j + 1 == priorities[i].size())
                    return false;
                if(current_priorities[j] != priorities[i][j])
                    break;
            }
        return true;
    }
    else //random
    {
        std::mt19937 g(rand());
        std::shuffle(current_priorities.begin(),current_priorities.end(), g);
        bool unique = false;
        int maxtries(1000000), tries(0);
        while(!unique && tries < maxtries)
        {
            tries++;
            for(unsigned int i = 0; i < priorities.size(); i++)
            {
                for(unsigned int j = 0; j < priorities[i].size(); j++)
                {
                    if(j + 1 == priorities[i].size())
                        unique = false;
                    if(current_priorities[j] != priorities[i][j])
                        break;
                }
                if(!unique)
                {
                    std::shuffle(current_priorities.begin(),current_priorities.end(), g);
                    break;
                }
            }
            unique = true;
        }
        return unique;
    }
}

