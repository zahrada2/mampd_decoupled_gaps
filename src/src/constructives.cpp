/**
 * File:    constructives.cpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka 
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "constructives.hpp"

using namespace cvrplib;
using namespace cvrplib::solutions;
using namespace cvrplib::constructives;



/* void cvrplib::constructives::loadRoute(std::string filename, MAPF_solution &mapf_solution, solutions::Solution &pathRes, int maxIter){ */

/*   /1* filename = "/home/owl/imr_workspace/cvrp-lib-vns/src/data/PBS/"; //_iterations10000_run0_instance_log.xml"; *1/ */
/*   int numRun = pathRes.instance().numRun; */
/*   filename = "../../src/data/solutions/"; */
/*   filename = filename + pathRes.instance().problem_name + "_iterations" + std::to_string(maxIter) + "_run" + std::to_string(numRun) + ".json"; */
/*   /1* auto problemName = fs::path(filename).stem().string(); *1/ */
/*   /1* std::cout << (problemName); *1/ */
/*   CVRPLIB_LOG_INFO("Loading solution from: " + filename); */

/*   MAPF_solution parsed_solution; */
/*   int cost = -1; */
/*   std::vector<std::vector<Ver>> path; */
/*   std::vector<Ver> single_path; */

/*   std::ifstream fin; */
/*   fin.open(filename); */

/*   if (!fin){ */
/*     CVRPLIB_LOG_ERROR("could not open file " << filename); */
/*   } */

/*   std::string delimiter = " "; */
/*   std::string line; */
/*   std::vector<std::string> line_split; */
/*   std::vector<std::string> line_; */
/*   bool found_routes = false; */
/*   int agent = 0; */
/*   while (!fin.eof()){ */
/*     std::getline(fin, line); */
/*     /1* line_split = split(line, delimiter); *1/ */
/*     std::string line_without_whitespaces = line; */
/*     line_without_whitespaces.erase(std::remove_if(line_without_whitespaces.begin(), line_without_whitespaces.end(), ::isspace), line_without_whitespaces.end()); */
/*     std::string route_section_start = "routes"; */
/*     std::size_t found_route_section_start = line_without_whitespaces.find(route_section_start); */
/*     std::string route_end = "]"; */
/*     std::size_t found_route_end = line_without_whitespaces.find(route_end); */

/*     std::string vrpcost_tag = "VRPCost"; */
/*     std::size_t vrpcost_found = line_without_whitespaces.find(vrpcost_tag); */

/*     if (!found_routes && vrpcost_found!=std::string::npos){ */
/*       std::string output = std::regex_replace( */
/*         line_without_whitespaces, */
/*         std::regex("[^0-9]*([0-9]+).*"), */
/*         std::string("$1") */
/*         ); */
/*       int vrp_cost = stoi(output); */
/*       pathRes.setMinimumVrpCost(vrp_cost); */
/*       CVRPLIB_LOG_INFO("VRP Cost of loaded solution is " << vrp_cost); */

/*     } */

/*     if (found_routes){ */

/*       // if section end */
/*       if (line_without_whitespaces == "},"){ */
/*         found_routes = false; */
/*         std::cout << "found end of route section\n"; */

/*         // if route end */
/*       } else if (found_route_end!=std::string::npos){ */
/*           std::cout << "ending path\n"; */
/*           path.push_back(single_path); */
/*           single_path.clear(); */
/*         ++agent; */
/*       } else { */
/*         std::string route_start = "["; */
/*         std::size_t found_route_start = line_without_whitespaces.find(route_start); */

/*         // if new agent */
/*         if (found_routes && found_route_start!=std::string::npos){ */
/*           std::cout << "new agent\n"; */
/*         } else { */

/*           // parse goal */
/*           std::string output = std::regex_replace( */
/*             line_without_whitespaces, */
/*             std::regex("[^0-9]*([0-9]+).*"), */
/*             std::string("$1") */
/*             ); */
/*           Ver goal_id = stoi(output); */
/*           single_path.push_back(goal_id); */
/*           /1* std::cout << goal_id << "\n"; *1/ */
/*         } */

/*       } */
/*     } else if (found_route_section_start!=std::string::npos){ */
/*       found_routes = true; */
/*       std::cout << "found route section\n"; */
/*     } */
/*     // end iterating through lines */

/*   } */


/*     MAPF_input input; */
/*     input.sequences = path; */
/*     CVRPLIB_LOG_INFO("Loaded route: "); */
/*     for (uint16_t i = 0; i < path.size(); ++i){ */
/*       for (uint16_t j = 0; j < path[i].size(); ++j){ */
/*         std::cout << path[i][j] << " "; */
/*       } */
/*       std::cout << "\n"; */
/*     } */
/*     parsed_solution = pathRes.instance().secondary_mapf_solver->computeCost(input); */
/*     mapf_solution = parsed_solution; */
/*     cost = mapf_solution.cost; */
/*     pathRes.setCvrpPath(path); */
/*     pathRes.loaded_cost = mapf_solution.cost; */
/*     pathRes.rememberCost(cost); */
/*     CVRPLIB_LOG_INFO("Loaded solution with PBS cost " << cost); */

/*     /1* std::cout << "json done, goodbye\n"; *1/ */
/*     CVRPLIB_LOG_INFO("JSON parsing done"); */
/* } */

/// VNS-specific CVRP methods
void cvrplib::constructives::rand(
        Solution &pathRes,
        MAPF_solution &mapf_solution,
        MainType &cost
)
{
    /// Get the number of neighbors.
    auto &n = pathRes.n();
    /// Define candidate list.
    std::vector<bool> cl(n, true);
    /// Set vertex 0 to the beginning of each path
    
    int numVehicles = pathRes.instance().numVehicles;
    UnsignedVector starting_depots = pathRes.instance().starting_depots;
    UnsignedVector ending_depots = pathRes.instance().ending_depots;

    
    CVRPLIB_LOG_INFO("Initializing " << numVehicles << " vehicle routes.....");
    pathRes.initializePathCvrp(numVehicles);

    /* bool different_ending_depot = false; */

    for (int vehicle = 0; vehicle < numVehicles; ++vehicle){
      /* std::cout << vehicle; */
      pathRes.setCvrp(starting_depots[vehicle], 0, vehicle);
      pathRes.pushBackCvrp(vehicle, ending_depots[vehicle]);
      /// Return to depot at the end of the trip
      /* if (!different_ending_depot){ */
      /*   pathRes.setCvrp(vehicle, 0, vehicle); */
      /*   pathRes.pushBackCvrp(vehicle, vehicle); */
      /*   cl[vehicle] = false; */
      /* } else { */
      /*   pathRes.setCvrp(2*vehicle, 0, vehicle); */
      /*   /1* pathRes.pushBackCvrp(vehicle, (vehicle+(numVehicles/2))%numVehicles); *1/ */
      /*   cl[2*vehicle] = false; */
      /* } */
      CVRPLIB_LOG_DEBUG("Vehicle route " << vehicle << " initialized."); 
      /// Make depot restricted.
    }

    for (uint16_t i = 0; i < starting_depots.size(); ++i){
      cl[starting_depots[i]] = false;
      cl[ending_depots[i]] = false;
    }

    if ((pathRes.getCvrpPath().size()) != numVehicles){ 
      CVRPLIB_LOG_WARNING("Some vehicles were not initialized!");
    } else {
      CVRPLIB_LOG_INFO("All vehicles successfuly initialized.");
    }
    
    /* if (different_ending_depot){ */
    /*   for (int vehicle = 0; vehicle < numVehicles; ++vehicle){ */
    /*     pathRes.pushBackCvrp(vehicle, (2*vehicle+numVehicles+1)%(2*numVehicles)); */
    /*     cl[(2*vehicle+numVehicles+1)%(2*numVehicles)] = false; */
    /*   } */
    /* } */

    bool couldAdd = false;
    int tgt_vehicle;
    // closed list of vehicles that were attempted for vertex insert
    std::vector<bool> vehicle_cl(numVehicles, false);

    /// The main loop.
    /// TODO: think what to do if can't fit all vertices
    uint16_t verticesLeft = 0;
    for (uint16_t i = 0; i < n; ++i){
      if (cl[i]){
        ++verticesLeft;
      }
    }
    for (Pos i = 1; i <= verticesLeft; i++){
      couldAdd = false;
      std::fill(vehicle_cl.begin(), vehicle_cl.end(), false);
      auto verCandidate = random::getRandomAmongAvailable(n-1, cl);
      if (verCandidate == -1){
        while (verCandidate == -1){
          verCandidate = random::getRandomAmongAvailable(n-1, cl);
        }
      }
      cl[verCandidate] = false;
      
      while (std::any_of(vehicle_cl.begin(), vehicle_cl.end(), 
            [](bool i){return i != true;}) 
              && !couldAdd){
        tgt_vehicle = random::UniformIntDistribution<int>{0, numVehicles-1}(random::rng());
        if (!vehicle_cl[tgt_vehicle]){
          if (pathRes.canFitIntoCapacity(tgt_vehicle, verCandidate)){
            couldAdd = true;
            pathRes.pushCvrp(tgt_vehicle, verCandidate);
          } else {
            vehicle_cl[tgt_vehicle] = true;
          }
        }
      }
      if (!couldAdd){
        CVRPLIB_LOG_ERROR("Could not fit vertex " << verCandidate << " into any vehicle's path!");
      }
      /// TODO: implement overfilling a vehicle's schedule when all vehicle's schedule is full
    }

    pathRes.updateDemands();
    pathRes.precomputeTTDDiscount();
    std::vector<std::vector<Ver>> testPath;
    if (pathRes.instance().useIntegratedMapf){
      MAPF_input mapfTask;
      mapfTask.sequences = pathRes.getCvrpPath();
      mapf_solution = pathRes.instance().mapfSolver->computeCost(mapfTask);
      cost = mapf_solution.cost;
      CVRPLIB_LOG_INFO("The initial random solution has MAPF cost equal to " << cost << ".");
    } else {
      cost = pathRes.computeCost();
      CVRPLIB_LOG_INFO("The initial random solution has cost equal to " << cost << ".");
      pathRes.rememberCost(cost);
    }
    /* sleep(3); */
    /* usleep(); */
}


/// VNS-specific CVRP methods
void cvrplib::constructives::randWithOrders(
        Solution &pathRes,
        MAPF_solution &mapf_solution,
        MainType &cost
)
{
    /// Get the number of neighbors.
    auto &n = pathRes.n();
    /// Define candidate list.
    std::vector<bool> cl(n, true);
    /// Set vertex 0 to the beginning of each path
    
    int numVehicles = pathRes.instance().numVehicles;
    UnsignedVector starting_depots = pathRes.instance().starting_depots;
    UnsignedVector ending_depots = pathRes.instance().ending_depots;

    
    CVRPLIB_LOG_INFO("Initializing " << numVehicles << " vehicle routes.....");
    pathRes.initializePathCvrp(numVehicles);

    /* bool different_ending_depot = false; */

    for (int vehicle = 0; vehicle < numVehicles; ++vehicle){
      /* std::cout << vehicle; */
      pathRes.setCvrp(starting_depots[vehicle], 0, vehicle);
      pathRes.pushBackCvrp(vehicle, ending_depots[vehicle]);
      cl[starting_depots[vehicle]] = false;
      cl[ending_depots[vehicle]] = false;
      /// Return to depot at the end of the trip
      /* if (!different_ending_depot){ */
      /*   pathRes.setCvrp(vehicle, 0, vehicle); */
      /*   pathRes.pushBackCvrp(vehicle, vehicle); */
      /*   cl[vehicle] = false; */
      /* } else { */
      /*   pathRes.setCvrp(2*vehicle, 0, vehicle); */
      /*   /1* pathRes.pushBackCvrp(vehicle, (vehicle+(numVehicles/2))%numVehicles); *1/ */
      /*   cl[2*vehicle] = false; */
      /* } */
      CVRPLIB_LOG_DEBUG("Vehicle route " << vehicle << " initialized."); 
      /// Make depot restricted.
    }
    if ((pathRes.getCvrpPath().size()) != numVehicles){ 
      CVRPLIB_LOG_WARNING("Some vehicles were not initialized!");
    } else {
      CVRPLIB_LOG_INFO("All vehicles successfuly initialized.");
    }
    
    /* if (different_ending_depot){ */
    /*   for (int vehicle = 0; vehicle < numVehicles; ++vehicle){ */
    /*     pathRes.pushBackCvrp(vehicle, (2*vehicle+numVehicles+1)%(2*numVehicles)); */
    /*     cl[(2*vehicle+numVehicles+1)%(2*numVehicles)] = false; */
    /*   } */
    /* } */

    bool couldAdd = false;
    int tgt_vehicle;
    // closed list of vehicles that were attempted for vertex insert
    std::vector<bool> vehicle_cl(numVehicles, false);

    /// The main loop.
    /// TODO: think what to do if can't fit all vertices
    uint16_t verticesLeft = 0;
    for (uint16_t i = 0; i < n; ++i){
      if (cl[i]){
        ++verticesLeft;
      }
    }
    for (Pos i = 1; i <= verticesLeft; i++){
      couldAdd = false;
      std::fill(vehicle_cl.begin(), vehicle_cl.end(), false);
      auto verCandidate = random::getRandomAmongAvailable(n-1, cl);
      if (verCandidate == -1){
        while (verCandidate == -1){
          verCandidate = random::getRandomAmongAvailable(n-1, cl);
        }
      }
      cl[verCandidate] = false;
      
      while (std::any_of(vehicle_cl.begin(), vehicle_cl.end(), 
            [](bool i){return i != true;}) 
              && !couldAdd){
        tgt_vehicle = random::UniformIntDistribution<int>{0, numVehicles-1}(random::rng());
        if (!vehicle_cl[tgt_vehicle]){
          if (pathRes.canFitIntoCapacity(tgt_vehicle, verCandidate)){
            if (pathRes.instance().orders[verCandidate] == pathRes.instance().assigned_stations[ending_depots[tgt_vehicle]]){
              couldAdd = true;
              pathRes.pushCvrp(tgt_vehicle, verCandidate);
              std::cout << pathRes.instance().orders[verCandidate] << " " << pathRes.instance().assigned_stations[ending_depots[tgt_vehicle]] << "\n"; 
              /* sleep(1); */
            } else {
              vehicle_cl[tgt_vehicle] = true;
            }
          } else {
            vehicle_cl[tgt_vehicle] = true;
          }
        }
      }
      if (!couldAdd){
        std::cout << pathRes.instance().orders[verCandidate] << "\n"; 
        CVRPLIB_LOG_ERROR("Could not fit vertex " << verCandidate << " into any vehicle's path!");
      }
      /// TODO: implement overfilling a vehicle's schedule when all vehicle's schedule is full
    }

    pathRes.updateDemands();
    std::vector<std::vector<Ver>> testPath;
    if (pathRes.instance().useIntegratedMapf){
      MAPF_input mapfTask;
      mapfTask.sequences = pathRes.getCvrpPath();
      mapf_solution = pathRes.instance().mapfSolver->computeCost(mapfTask);
      cost = mapf_solution.cost;
      CVRPLIB_LOG_INFO("The initial random solution has MAPF cost equal to " << cost << ".");
    } else {
      cost = pathRes.computeCost();
      CVRPLIB_LOG_INFO("The initial random solution has cost equal to " << cost << ".");
      pathRes.rememberCost(cost);
    }
}

/// VNS-specific CVRP methods
void cvrplib::constructives::clarkeWright(
        Solution &pathRes,
        MainType &cost
)
{
    /// Get the number of neighbors.
    auto &n = pathRes.n();
    /// Define candidate list.
    std::vector<bool> cl(n, true);
    /// Set vertex 0 to the beginning of each path
    std::vector<std::vector<Ver>> pathList;
    // initialize n paths
    for (uint16_t vertex = 0; vertex < pathRes.n()/2; ++vertex){
      std::vector<Ver> single_path;
      single_path.push_back(vertex);
      single_path.push_back(vertex+pathRes.n()/2);
      pathList.push_back(single_path);
    }
   

    // use pathAux, not just a vector

    std::vector<int> single_col(n, 0);
    std::vector<std::vector<int>> improvements;
    for (uint16_t i = 0; i < n; i++){
      improvements.push_back(single_col);
    }
    
    int numVehicles = pathRes.instance().numVehicles;
    bool improvement_found = true;
    bool validPathFound = false;
    bool canStop = false;


    while (improvement_found || !canStop){
      improvement_found = false;
      validPathFound = false;
      int max_improvement = 0;
      int max_i = 0;
      int max_j = 0;
      for (int i = 0; i < pathList.size(); ++i){
        for (int j = i+1; j < pathList.size(); ++j){
          double price_i = 0;
          double price_j = 0;
          // TODO: implement mapf calls here
          // TODO: calculate distances of paths, not vertices.. resolved?
          for (int k = 1; k < pathList[i].size(); ++k){
            price_i += pathRes.instance().rho[pathList[i][k-1]][pathList[i][k]];
          }
          for (int k = 1; k < pathList[j].size(); ++k){
            price_j += pathRes.instance().rho[pathList[j][k-1]][pathList[j][k]];
          }
          improvements[i][j] = price_i + price_j - pathRes.instance().rho[pathList[i][pathList[i].size()-1]][pathList[j][pathList[j].size()-1]];
        }
      }
      bool move_found = true;
      max_improvement = 0;
      while (move_found && !validPathFound){
        move_found = false;
        for (int i = 0; i < pathList.size(); ++i){
          for (int j = i+1; j < pathList.size(); ++j){
          if (improvements[i][j] > max_improvement){
            // TODO: implement capacity checking here?
            if ((pathList[i].size() + pathList[j].size() - 2) < pathRes.instance().vehicleCapacity+2){
              if (pathList[j][0] > pathRes.instance().numVehicles){
                move_found = true;
                max_improvement = improvements[i][j];
                max_i = i;
                max_j = j;
                validPathFound = true;
                if (pathList.size() < pathRes.instance().numVehicles){
                  canStop = true;
                }
              }
            } else {
              improvements[i][j] = 0;
            }
          }
          }
        }
      }

      if (validPathFound){
        improvement_found = true;
        std::vector<Ver> new_path;
        for (int k = 0; k < pathList[max_i].size(); ++k){
          new_path.push_back(pathList[max_i][k]);
        }
        for (int k = 0; k < pathList[max_j].size(); ++k){
          new_path.push_back(pathList[max_j][k]);
        }
        pathList.erase(pathList.begin() + max_j);
        pathList.erase(pathList.begin() + max_i);
        pathList.push_back(new_path);
        /* new_path.push_back() */
      }
      if (!canStop && !improvement_found){
        printf("Could not find a valid solution!\n");
        break;
      }
    }
    for (int i = 0; i < pathList.size(); ++i){
      pathList[i].push_back(pathList[i][0]);
    }
    for (int i = 0; i < pathList.size(); ++i){
      for (int j = 0; j < pathList[i].size(); ++j){
        printf(" %d ", pathList[i][j]);
      }
      printf("\n");
    }

    pathRes.setCvrpPath(pathList);
    cost = pathRes.computeCost();
    printf("clarke-wright cost: %d \n", cost);


}

void cvrplib::constructives::mcwsa(
        Solution &pathRes,
        MainType &cost
)
{
    /// Get the number of neighbors.
    auto &n = pathRes.n();
    /// Define candidate list.
    std::vector<bool> cl(n, true);
    /// Set vertex 0 to the beginning of each path
    std::vector<std::vector<Ver>> pathList;
    std::vector<uint16_t> free_capacities;
    uint16_t vehicle_capacity = pathRes.instance().vehicleCapacity;
    uint16_t numVehicles = pathRes.instance().numVehicles;
    bool use_mapf = pathRes.instance().useIntegratedMapf;
    boost::shared_ptr<MAPF_interface> mapf = pathRes.instance().mapfSolver;
    // initialize depots
    for (uint16_t vertex = 0; vertex < numVehicles; ++vertex){
      std::vector<Ver> single_path;
      single_path.push_back(vertex);
      pathList.push_back(single_path);
      // start with full capacity
      free_capacities.push_back(vehicle_capacity);
    }

    // load goals (vertices - depots)
    std::vector<Ver> unassigned_goals;

    for (uint16_t vertex = numVehicles; vertex < n; ++vertex){
      unassigned_goals.push_back(vertex);
    }

    // iterate till we assign all goals
    uint16_t path_index = 0;
    while (unassigned_goals.size() > 0){

      // Since we know the number of vehicles,
      // we just check if the currently explored one
      // has some free capacity. If not, move to another one
      if ((free_capacities[path_index] == 0) && (path_index < numVehicles - 1)){
        path_index += 1;
      } else if ((free_capacities[path_index] == 0) && (path_index == numVehicles - 1)){
        CVRPLIB_LOG_WARNING("No route with enough capacity! Solution is incomplete.");
        break;
      }

      Ver closest = NULL;
      std::vector<Ver> Ttmp;
      // calculate distances for each goal 
      Ver next = 0;
      MainType largest_dist = -1;
      uint16_t largest_dist_index = -1;
      for (int j = 0; j < unassigned_goals.size(); ++j){
        long long dist = -1;
        // if mapf flag is set
        if (use_mapf){
          MAPF_input task;
          std::vector<std::vector<Ver>> mapf_sequences = pathList;
          mapf_sequences[path_index].push_back(unassigned_goals[j]);
          task.sequences = mapf_sequences;
          dist = mapf->computeCost(task).cost;
        } else {
          dist = pathRes.instance().rho[pathList[path_index][0]][unassigned_goals[j]];
        }
          if (dist > largest_dist){
            largest_dist = dist;
            largest_dist_index = j;
          }
      }
      next = largest_dist_index;
      
      while (free_capacities[path_index] > 0 && unassigned_goals.size() > 0){
        if (Ttmp.size() == 0 || closest == Ttmp[0]){
          Ttmp.insert(Ttmp.begin(), unassigned_goals[next]);
        } else {
          Ttmp.push_back(unassigned_goals[next]);
        }
        /* free_capacities[path_index] += -(pathRes.instance().demands[next]); */ 
        free_capacities[path_index] += -(pathRes.instance().demands[unassigned_goals[next]]); 
        unassigned_goals.erase(unassigned_goals.begin() + next);
        // find another next and closest
        MainType min_saving = INT_MAX;
        for (int i = 0; i < unassigned_goals.size(); ++i){
          for (int j = 0; j < 2; ++j){
            long long dist = -1;

            int temporary = j*Ttmp.size()-1;
            int index = std::max(0, temporary);

            if (use_mapf){
              MAPF_input task;
              std::vector<std::vector<Ver>> mapf_sequences = pathList;
              // insert currently created path
              std::vector<Ver> Ttmp_consider = Ttmp;
              if (index == 0){
                Ttmp_consider.insert(Ttmp_consider.begin(), unassigned_goals[i]);
              } else { 
                Ttmp_consider.push_back(unassigned_goals[j]);
              }
              mapf_sequences[path_index].insert(mapf_sequences[path_index].begin()+1, Ttmp.begin(), Ttmp.end());
              // insert ending depots for accurate cost calc
              for (uint16_t depot = 0; depot < numVehicles; ++depot){
                mapf_sequences[depot].push_back(mapf_sequences[depot][0]);
              }
              task.sequences = mapf_sequences;
              dist = mapf->computeCost(task).cost;
            } else {
              dist = pathRes.instance().rho[unassigned_goals[i]][Ttmp[index]];
              dist += -pathRes.instance().rho[pathList[path_index][0]][Ttmp[index]];
              dist += -pathRes.instance().rho[pathList[path_index][0]][unassigned_goals[i]];
              /* dist = pathRes.instance().rho[pathList[path_index][0]][unassigned_goals[j]]; */
            }

            /* MainType saving = pathRes.instance().rho[unassigned_goals[i]][Ttmp[j]]; */
            /* saving += -pathRes.instance().rho[pathList[path_index][0]][Ttmp[j]]; */
            /* saving += -pathRes.instance().rho[pathList[path_index][0]][unassigned_goals[i]]; */
            /* if (saving < min_saving){ */
            /*   min_saving = saving; */
            /*   closest = j; */
            /*   next = i; */
            /* } */
            if (dist < min_saving){
              min_saving = dist;
              closest = Ttmp[index];
              next = i;
            }
          }
        }
      }
      pathList[path_index].insert(pathList[path_index].begin()+1, Ttmp.begin(), Ttmp.end());
      

    }
   

    // use pathAux, not just a vector

    std::vector<int> single_col(n, 0);
    std::vector<std::vector<int>> improvements;
    for (uint16_t i = 0; i < n; i++){
      improvements.push_back(single_col);
    }
    
    bool improvement_found = true;
    bool validPathFound = false;
    bool canStop = false;


    for (int i = 0; i < pathList.size(); ++i){
      pathList[i].push_back(pathList[i][0]);
    }
    for (int i = 0; i < pathList.size(); ++i){
      for (int j = 0; j < pathList[i].size(); ++j){
        printf(" %d ", pathList[i][j]);
      }
      printf("\n");
    }

    pathRes.setCvrpPath(pathList);
    if (use_mapf){
      MAPF_input task;
      task.sequences = pathRes.getCvrpPath();
      cost = mapf->computeCost(task).cost;
    } else {
      cost = pathRes.computeCost();
    }

}

void cvrplib::constructives::mcwsa_alternative(
        Solution &pathRes,
        MainType &cost
)
{
    /// Get the number of neighbors.
    auto &n = pathRes.n();
    /// Define candidate list.
    std::vector<bool> cl(n, true);
    /// Set vertex 0 to the beginning of each path
    std::vector<std::vector<Ver>> pathList;
    std::vector<std::vector<Ver>> Ttmps;
    std::vector<int> free_capacities;
    uint16_t vehicle_capacity = pathRes.instance().vehicleCapacity;
    uint16_t numVehicles = pathRes.instance().numVehicles;


    // initialize depots
    for (uint16_t vertex = 0; vertex < numVehicles; ++vertex){
      std::vector<Ver> single_path;
      single_path.push_back(vertex);
      pathList.push_back(single_path);
      // start with full capacity
      free_capacities.push_back(vehicle_capacity);
      std::vector<Ver> Ttmp;
      Ttmps.push_back(Ttmp);
    }

    // load goals (vertices - depots)
    std::vector<Ver> unassigned_goals;

    for (uint16_t vertex = numVehicles; vertex < n; ++vertex){
      unassigned_goals.push_back(vertex);
    }

    // iterate till we assign all goals
    uint16_t path_index = 0;
    uint16_t target_vehicle = -1;
    bool no_capacity = false;


    /* while (unassigned_goals.size() > 0){ */

      /* bool no_capacity = std::all_of(v.begin(), v.end(), [](int i)) */

    Ver closest = NULL;
    // calculate distances for each goal 
    Ver next = 0;
    target_vehicle = -1;
    MainType largest_dist = -1;
    uint16_t largest_dist_index = -1;

    for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){
      for (int j = 0; j < unassigned_goals.size(); ++j){
        MainType dist = pathRes.instance().rho[pathList[vehicle][0]][unassigned_goals[j]];
        if (dist > largest_dist){
          largest_dist = dist;
          largest_dist_index = j;
          target_vehicle = vehicle;
        }
      }
    }
    next = largest_dist_index;
    
    while (!no_capacity && unassigned_goals.size() > 0){
      if (Ttmps[target_vehicle].size() == 0 || closest == Ttmps[target_vehicle][0]){
        Ttmps[target_vehicle].insert(Ttmps[target_vehicle].begin(), unassigned_goals[next]);
      } else {
        Ttmps[target_vehicle].push_back(unassigned_goals[next]);
      }

      /* free_capacities[target_vehicle] += -(pathRes.instance().demands[unassigned_goals[next]]); */ 
      free_capacities[target_vehicle] += -1; 
      unassigned_goals.erase(unassigned_goals.begin() + next);
      no_capacity = std::all_of(free_capacities.begin(), free_capacities.end(), [](int i) {return i == 0;});

      // find another next and closest
      MainType min_saving = INT_MAX;
      target_vehicle = -1;
      for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){
        // skip full vehicles
        std::cout << free_capacities[vehicle] << "\n";
        if (free_capacities[vehicle] < 1){
          std::cout << vehicle << "dat one";
          /* continue; */
        } else {
          for (int i = 0; i < unassigned_goals.size(); ++i){
            for (int j = 0; j < Ttmps[vehicle].size(); ++j){
              MainType saving = pathRes.instance().rho[unassigned_goals[i]][Ttmps[vehicle][j]];
              saving += -pathRes.instance().rho[pathList[vehicle][0]][Ttmps[vehicle][j]];
              saving += -pathRes.instance().rho[pathList[vehicle][0]][unassigned_goals[i]];
              std::cout << "min saving: " << min_saving << " saving: " << saving << "\n";
              if (saving < min_saving){
                min_saving = saving;
                closest = j;
                next = i;
                target_vehicle = vehicle;
                std::cout << "tgt vehicle: " << target_vehicle << "\n";
              }
            }
          }
        }
      }
      if (target_vehicle == -1){
        CVRPLIB_LOG_WARNING("Couldn't find a vehicle that would generate a feasible path!");
      }
    }
  /* } */
   
    for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){
      pathList[vehicle].insert(pathList[vehicle].begin(), Ttmps[vehicle].begin(), Ttmps[vehicle].end());
    }


    std::vector<int> single_col(n, 0);
    std::vector<std::vector<int>> improvements;
    for (uint16_t i = 0; i < n; i++){
      improvements.push_back(single_col);
    }
    
    bool improvement_found = true;
    bool validPathFound = false;
    bool canStop = false;


    for (int i = 0; i < pathList.size(); ++i){
      pathList[i].push_back(pathList[i][0]);
    }
    for (int i = 0; i < pathList.size(); ++i){
      for (int j = 0; j < pathList[i].size(); ++j){
        printf(" %d ", pathList[i][j]);
      }
      printf("\n");
    }

    pathRes.setCvrpPath(pathList);
    cost = pathRes.computeCost();

}
