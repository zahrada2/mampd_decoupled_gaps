/**
 * File:    local_search.cpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "local_search.hpp"

using namespace cvrplib;
using namespace cvrplib::ls;
using namespace cvrplib::solutions;

bool cvrplib::ls::exploreNeighborhoods(
        cvrplib::solutions::Solution &pathRes,
        cvrplib::MainType &cost
)
{
  bool improvement = false;
    /// Determine random number of steps and random neighborhood to explore
    /// Perform, return if improvement was found
    /// TODO: implement the move cost calculation/estimation in solutions_impl 
    /// TODO: implement the actual move (in solutions, I suppose, or solutions_impl)
    /// Swapping the sequences of vertices, or single vertices, could be done by updating either path or path_inv as is done now.
    /// however, that depends on the updateStructures method. Perhaps auxiliary arrays could work as well?
  return improvement;
}

bool cvrplib::ls::exploreVnsNeighbourhoods(
  std::vector<FunptrOperator> neighborhoods,
  solutions::Solution &pathRes,
  MAPF_solution &best_mapf,
  MainType &cost)
{

    /// Create array of neighborhoods.
    /* using FunptrOperator = bool (*)( */
    /*         solutions::Solution &, */
    /*         MAPF_solution &, */
    /*         MainType & */
    /* ); */
    
  // return value is improvement achieved
  bool ret = false;
  MAPF_input mapf_task;
  MAPF_solution initial_mapf;
  /* best_mapf = initial_mapf; */
  if (pathRes.useMapf){
    mapf_task.sequences = pathRes.getCvrpPath();

    initial_mapf = pathRes.instance().mapfSolver->computeCost(mapf_task);
    CVRPLIB_LOG_INFO("Cost at the start of iteration is " << initial_mapf.cost);
    /* ++mapf_iter; */
    /* std::cout << initial_mapf.cost; */
    cost = initial_mapf.cost;
    /* std::cout << "hello\n"; */
  }
  MAPF_solution working_mapf = initial_mapf;

    /* while (exchange(pathRes, best_mapf, cost) */
    /*        || relocate(pathRes, best_mapf, cost) */
    /*        || twoOpt(pathRes, best_mapf, cost) */
    /*        || threeOpt(pathRes, best_mapf, cost)) { */
        /// If the time limit or cost goal are met, then break the loop and set interrupt to true.
        /* if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
        /*     || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
        /*     interrupt = true; */
        /*     break; */
        /* } */
    /* } */

    /* std::vector<FunptrOperator> neighborhoods{exchange, relocate, twoOpt, threeOpt}; */
  std::vector<uint32_t> op_calls(neighborhoods.size(), 0);

  bool improvement = true;
  /* bool outputted = false; */
  while(improvement){
    if (pathRes.useMapf){
      mapf_task.sequences = pathRes.getCvrpPath();
      /* std::cout << "usemapf is " << pathRes.useMapf << "\n"; */
      MAPF_solution init_solution = pathRes.instance().mapfSolver->computeCost(mapf_task);
      std::cout << "Init solution has cost " << init_solution.cost << std::endl;
      /* initial_mapf = pathRes.instance().mapfSolver->computeCost(mapf_task); */
      initial_mapf = init_solution;
    }
    improvement = false;
    for (unsigned long i = 0; i < neighborhoods.size(); ++i){
      /* std::cout << "exploring\n"; */
      if (pathRes.instance().mapfLevelOps && pathRes.useMapf){
        std::cout << "MAPF cost before search: " << cost << "\n";
        std::cout << "MAPF cost before search from pathres: " << pathRes.recallCost() << "\n";
      }

      int operator_selector = random::UniformIntDistribution<uint16_t>{0, 100}(random::rng());
      working_mapf = initial_mapf;

      improvement = (*neighborhoods[i])(pathRes, working_mapf, cost);
      /* std::cout << "neigh was explored\n"; */
      op_calls[i]++;

      if (pathRes.instance().mapfLevelOps && pathRes.useMapf){
        std::cout << "MAPF cost after search: " << cost << "\n";
        std::cout << "MAPF cost after search from pathres: " << pathRes.recallCost() << "\n";
        /* std::string name = std::to_string(operator_selector) + "internal_solution.xml"; */
        /* std::string strMapfFileOutRunIteration = "./" + name; */

        if (improvement){
          /* pathRes.instance().mapfSolver->saveSolutionIntoXML(strMapfFileOutRunIteration, pathRes.best_sol); */
          /* std::cout << "outputted solution with cost: " << pathRes.be */
        }

      }
      // reset cycle from start if improvement was found
      if (improvement){ ret = true; break; }
    }
  }

  if (pathRes.instance().mapfLevelOps && pathRes.useMapf){
    /* mapf_task.sequences = pathRes.getCvrpPath(); */
    /* initial_mapf = pathRes.instance().mapfSolver->computeCost(mapf_task); */
    /* cost = initial_mapf.cost; */
    CVRPLIB_LOG_WARNING(cost);
  } else {
    cost = pathRes.computeCost();
  }

  pathRes.rememberCost(cost);
  /* pathRes.setVrpOpCounts(op_calls); */
  return ret;

}


bool cvrplib::ls::relocate(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
  bool improvement=false;
  uint16_t best_veh1 = 0, best_veh2 = 0, best_ver1 = 0, best_ver2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  Ver considered_vertex1 = 0;
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution, verification_mapf;

  MAPF_input mapfTask;
  /* mapfTask.sequences = pathAux.getCvrpPath(); */
  /* pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask); */

  /* CVRPLIB_LOG_DEBUG("relocate start"); */
  /* std::cout << pathAux.toString(); */
  /* usleep(500000); */

  /// explore relocate neighborhood
  /// reset the MAPF solver via init
  mapfTask.sequences = pathAux.getCvrpPath();
  pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);


  std::vector<Ver> path_1, path_2;
  for (uint16_t veh1 = 0; veh1 < numVehicles; ++veh1){
    path_1 = pathAux.getCvrpPath()[veh1];
    /* for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 1) && (!improvement); ++vtx1){ */
    for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 1); ++vtx1){
      considered_vertex1 = path_1[vtx1];
      for (uint16_t veh2 = 0; veh2 < numVehicles; ++veh2){
        if (pathAux.canFitIntoCapacity(veh2, considered_vertex1)){
          /// Maybe allow to relocate in the same path?
          /* std::cout << "relocate loop" << std::endl; */
          if (veh1 == veh2){
            continue;
          }
          for (uint16_t vtx2 = vtx1; (vtx2 < pathAux.getNumVertices(veh2)-1); ++vtx2){
            if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {

              pathAux.instance().mapfSolver->computeRelocateCost(considered_vertex1, veh2, vtx2, veh1, vtx1);

            } else {

              proposed_cost = pathAux.tryRelocate(veh1, veh2, vtx1, vtx2);

              if (proposed_cost == -1){
                CVRPLIB_LOG_DEBUG("Integrated MAPF returned invalid cost, skipping move ...");
              } else if (proposed_cost != -2) {
                if (proposed_cost < best_improved_cost){
                  best_improved_cost = proposed_cost;
                  improvement = true;
                  best_veh1 = veh1;
                  best_veh2 = veh2;
                  best_ver1 = vtx1;
                  best_ver2 = vtx2;
                  break;
                }
              }
            }
          }
        }
      }
    }
  }

  /// If we found a new best solution, store it, start from the beginning
  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    newSolution = pathAux.instance().mapfSolver->getBestSolution();
    if (newSolution.cost < cost){
      CVRPLIB_LOG_TRACE("MAPF Relocate improved cost from " << cost << " to " << newSolution.cost);
      /* std::cout << pathAux.toString(); */

      /* CVRPLIB_LOG_DEBUG("Relocate end"); */

      mapfTask = newSolution.input;
      verification_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask);
      /* if (verification_mapf.cost > 10*best_mapf.cost){ */
      if (verification_mapf.cost > 100*cost){
        /* CVRPLIB_LOG_ERROR("Verification provided invalid cost, keeping solution from iteration"); */
      } else {
        best_mapf = verification_mapf;
        cost = best_mapf.cost;
        pathAux.setCvrpPath(best_mapf.input.sequences);
        /* auto best_cost = pathAux.instance().mapfSolver->estimateCost(mapfTask); */
        /* CVRPLIB_LOG_TRACE("The new verified cost is " << best_mapf.cost<<"; Estimated cost is "<<est_cost); */
      }

      best_mapf = newSolution;
      pathAux.setCvrpPath(best_mapf.input.sequences);
      cost = best_mapf.cost;
      pathAux.rememberCost(cost);
      pathAux.best_sol = best_mapf;
      improvement = true;
    }

  } else if (improvement){
      CVRPLIB_LOG_TRACE("Relocate improved cost from " << cost << " to " << best_improved_cost);
      /* std::cout << pathAux.toString(); */
      /* CVRPLIB_LOG_DEBUG("Relocate end"); */
      pathAux.relocate(best_veh1, best_veh2, best_ver1, best_ver2);
      cost = best_improved_cost;
      pathAux.rememberCost(cost);
  }

  bool ret = improvement;
  return ret;
}

bool cvrplib::ls::exchange(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{

  bool improvement = false;
  uint16_t best_veh1 = 0, best_veh2 = 0, best_ver1 = 0, best_ver2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;

  MAPF_input mapfTask;
  mapfTask.sequences = pathAux.getCvrpPath();
  pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);
  /* CVRPLIB_LOG_DEBUG("exchange start"); */
  /* std::cout << pathAux.toString(); */
  /* usleep(500000); */

  /// Get operator starting time
  /* op_start_time = operator_clock.getTimeInSeconds(); */

  /// Iterate through all pairs of vehicles and all vertices of each
  for (uint16_t vehicle1 = 0; vehicle1 < numVehicles; ++vehicle1){
    uint16_t num_vertices1 = pathAux.getNumVertices(vehicle1);
    for (uint16_t vtx1 = 1; (vtx1 < num_vertices1 - 1); ++vtx1){
      for (uint16_t vehicle2 = 0; vehicle2 < numVehicles; ++vehicle2){
        /// Skip if exchanging vertices within the path .. maybe allow? 
        uint16_t min_vtx2 = vtx1;
        if (vehicle1 == vehicle2){
          /* min_vtx2 = vtx1 + 2; */
          continue;
        }
        /// vtx2 = vtx1 reduces computation time. 
        /// veh1 = 1, veh2 = 3, vtx1 = 5, vtx2 = 6
        /// is the same as
        /// veh1 = 3, veh2 = 1, vtx1 = 6, vtx1 = 5
        for (uint16_t vtx2 = min_vtx2; (vtx2 < pathAux.getNumVertices(vehicle2)-1); ++vtx2){
          /// Call internal MAPF operator if --mapf-ops
          if (pathAux.instance().mapfLevelOps && pathAux.useMapf){

            pathAux.instance().mapfSolver->computeExchange(vehicle1, vehicle2, vtx1, vtx2);


          } else {

            proposed_cost = pathAux.tryExchange(vehicle1, vehicle2, vtx1, vtx2);

            if (proposed_cost == -1){
              CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
            } else {
              if (proposed_cost < best_improved_cost){
                best_improved_cost = proposed_cost;
                improvement = true;
                best_veh1 = vehicle1;
                best_veh2 = vehicle2;
                best_ver1 = vtx1;
                best_ver2 = vtx2;
              }
            }
          }
        }
      }
    }
  }

  MAPF_solution newSolution, verification_mapf;

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    newSolution = pathAux.instance().mapfSolver->getBestSolution();
    if (newSolution.cost < cost){
      // MOVE TO ANOTHER FUNCTION
      CVRPLIB_LOG_TRACE("MAPF Exchange improved cost from " << cost << " to " << newSolution.cost);
      /* std::cout << pathAux.toString(); */
      /* CVRPLIB_LOG_DEBUG("Exchange end"); */

      /* mapfTask = newSolution.input; */
      /* verification_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask); */
      /* /1* if (verification_mapf.cost > 10*best_mapf.cost){ *1/ */
      /* if (verification_mapf.cost > 100*cost){ */
      /*   CVRPLIB_LOG_ERROR("Verification provided invalid cost, keeping solution from iteration"); */
      /* } else { */
      /*   best_mapf = verification_mapf; */
      /*   cost = best_mapf.cost; */
      /*   pathAux.setCvrpPath(best_mapf.input.sequences); */
      /*   auto est_cost = pathAux.instance().mapfSolver->estimateCost(mapfTask); */
      /*   CVRPLIB_LOG_TRACE("The new verified cost in iteration " << i << " is " << best_mapf.cost<<"; Estimated cost is "<<est_cost); */
      /* } */

      best_mapf = newSolution;
      pathAux.setCvrpPath(best_mapf.input.sequences);
      cost = best_mapf.cost;
      pathAux.rememberCost(cost);
      improvement = true;
      pathAux.best_sol = best_mapf;
    }

    /// otherwise if improvement, just call VRP operator with best indices.
  } else if (improvement){
      CVRPLIB_LOG_TRACE("Exchange improved cost from " << cost << " to " << best_improved_cost);
      /* std::cout << pathAux.toString(); */
      /* std::cout << best_veh1 << " " << best_veh2 << " " <<  best_ver1 <<" " << best_ver2 <<" " <<std::endl; */
      /* CVRPLIB_LOG_DEBUG("Exchange end"); */
      pathAux.exchange(best_veh1, best_veh2, best_ver1, best_ver2);
      cost = best_improved_cost;
      pathAux.rememberCost(cost);
  }

  bool ret = improvement;
  return ret;

}



bool cvrplib::ls::twoOpt(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
 
  bool improvement=false;
  uint16_t best_veh1 = 0, best_ver1 = 0, best_ver2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution, verification_mapf;

  MAPF_input mapfTask;

  mapfTask.sequences = pathAux.getCvrpPath();
  pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);

  /// Explore 2opt neighbourhood
  for (uint16_t veh1 = 0; veh1 < numVehicles; ++veh1){
    /* for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 3) && (!improvement); ++vtx1){ */
    /*   for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2 && !improvement; ++vtx2){ */
    for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 3); ++vtx1){
      for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2; ++vtx2){
        if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {

          pathAux.instance().mapfSolver->computeTwoOpt(veh1, vtx1, vtx2);

        } else {

          proposed_cost = pathAux.tryTwoOptCvrp(veh1, vtx1, vtx2);

          if (proposed_cost == -1){
            CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
          } else {
            if (proposed_cost < best_improved_cost){
              best_improved_cost = proposed_cost;
              /* perform_three_opt = false; */
              improvement = true;
              best_veh1 = veh1;
              best_ver1 = vtx1;
              best_ver2 = vtx2;
            }
          }
        }
      }
    }
  }

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    newSolution = pathAux.instance().mapfSolver->getBestSolution();
    if (newSolution.cost < cost){
      CVRPLIB_LOG_TRACE("MAPF 2opt improved cost from " << cost << " to " << newSolution.cost);

      mapfTask = newSolution.input;
      verification_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask);
      /* if (verification_mapf.cost > 100*best_mapf.cost){ */
      if (verification_mapf.cost > 100*cost){
        CVRPLIB_LOG_ERROR("Verification provided invalid cost, keeping solution from iteration");
      } else {
        best_mapf = verification_mapf;
        cost = best_mapf.cost;
        pathAux.setCvrpPath(best_mapf.input.sequences);
        auto est_cost = pathAux.instance().mapfSolver->estimateCost(mapfTask);
        CVRPLIB_LOG_TRACE("The new verified cost is " << best_mapf.cost<<"; Estimated cost is "<<est_cost);
      }

      best_mapf = newSolution;
      pathAux.setCvrpPath(best_mapf.input.sequences);
      cost = best_mapf.cost;
      pathAux.rememberCost(cost);
      improvement = true;
      pathAux.best_sol = best_mapf;
    }
  } else if (improvement){
      CVRPLIB_LOG_TRACE("2opt improved cost from " << cost << " to " << best_improved_cost);
      pathAux.twoOptCvrp(best_veh1, best_ver1, best_ver2);
      cost = best_improved_cost;
      pathAux.rememberCost(cost);
  }

  bool ret = improvement;
  return ret;
}



bool cvrplib::ls::threeOpt(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
  bool improvement=false;
  uint16_t best_veh1 = 0, best_ver1 = 0, best_ver2 = 0, best_ver3 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution, verification_mapf;
  MAPF_input mapfTask;

  mapfTask.sequences = pathAux.getCvrpPath();
  pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);

  for (uint16_t veh1 = 0; veh1 < numVehicles; ++veh1){
    /* for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 3) && (!improvement); ++vtx1){ */
    /*   for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2 && !improvement; ++vtx2){ */
    for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 3); ++vtx1){
      for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2; ++vtx2){
        for (uint16_t vtx3 = vtx2+2; vtx3 < pathAux.getNumVertices(veh1) - 2; ++vtx3){
          if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {

            pathAux.instance().mapfSolver->computeThreeOpt(veh1, vtx1, vtx2, vtx3);

          } else {

            proposed_cost = pathAux.tryThreeOptCvrp(veh1, vtx1, vtx2, vtx3);

            if (proposed_cost == -1){
              CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
            } else {
              if (proposed_cost < best_improved_cost){
                best_improved_cost = proposed_cost; 
                /* perform_three_opt = true; */
                improvement = true;
                best_veh1 = veh1;
                best_ver1 = vtx1;
                best_ver2 = vtx2;
                best_ver3 = vtx3;
              }
            }
          }
        }
      }
    }
  }

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    newSolution = pathAux.instance().mapfSolver->getBestSolution();
    if (newSolution.cost < cost){
      CVRPLIB_LOG_TRACE("MAPF 3opt improved cost from " << cost << " to " << newSolution.cost);

      mapfTask = newSolution.input;
      verification_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask);
      /* if (verification_mapf.cost > 100*best_mapf.cost){ */
      if (verification_mapf.cost > 100*cost){
        CVRPLIB_LOG_ERROR("Verification provided invalid cost, keeping solution from iteration");
      } else {
        best_mapf = verification_mapf;
        cost = best_mapf.cost;
        pathAux.setCvrpPath(best_mapf.input.sequences);
        auto est_cost = pathAux.instance().mapfSolver->estimateCost(mapfTask);
        CVRPLIB_LOG_TRACE("The new verified cost is " << best_mapf.cost<<"; Estimated cost is "<<est_cost);
      }

      best_mapf = newSolution;
      pathAux.setCvrpPath(best_mapf.input.sequences);
      cost = best_mapf.cost;
      pathAux.rememberCost(cost);
      improvement = true;
      /* pathAux.best_sol = best_mapf; */
    }
  } else if (improvement){
      CVRPLIB_LOG_TRACE("3opt improved cost from " << cost << " to " << best_improved_cost);
      pathAux.threeOptCvrp(best_veh1, best_ver1, best_ver2, best_ver3);
      cost = best_improved_cost;
      pathAux.rememberCost(cost);
  }

  bool ret = improvement;
  return ret;
}

/// FIRST ACCEPT (FA) OPERATORS


bool cvrplib::ls::relocateFA(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
  bool improvement=false;
  uint16_t best_veh1 = 0, best_veh2 = 0, best_ver1 = 0, best_ver2 = 0;
  Ver considered_vertex1 = 0, considered_vertex2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution;
  MAPF_input mapfTask;

  /* mapfTask.sequences = pathAux.getCvrpPath(); */
  /* pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask); */

  /* CVRPLIB_LOG_DEBUG("relocate start"); */
  /* std::cout << pathAux.toString(); */
  /* usleep(500000); */

  /// explore relocate neighborhood
  /// reset the MAPF solver via init
  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    mapfTask.sequences = pathAux.getCvrpPath();
    pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);
  }

  if (RECOUNT_CHECK){
    if ((cost != pathAux.computeTTD())){
      CVRPLIB_LOG_WARNING("Relocate cost at the start and verify cost are mismatched!");
    }
  }

  /* CVRPLIB_LOG_WARNING("cost_start: " << cost); */
  std::vector<Ver> path_1, path_2;
  for (uint16_t veh1 = 0; veh1 < numVehicles && !improvement; ++veh1){
    path_1 = pathAux.getCvrpPath()[veh1];
    for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 1) && (!improvement); ++vtx1){
      considered_vertex1 = path_1[vtx1];
      for (uint16_t veh2 = 0; veh2 < numVehicles && !improvement; ++veh2){
        /// Maybe allow to relocate in the same path?
        if (veh1 == veh2){
          continue;
        }
        if (pathAux.canFitIntoCapacity(veh2, considered_vertex1)){
          /* path_2 = pathAux.getCvrpPath()[veh2]; */
          for (uint16_t vtx2 = vtx1; (vtx2 < pathAux.getNumVertices(veh2)-1) && (!improvement); ++vtx2){
            if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {
              /* std::cout << "relocate loop" <<std::endl; */

              /* pathAux.instance().mapfSolver->computeRelocateCost(pathAux.getCvrpPath()[veh1][vtx1], veh2, vtx2, veh1, vtx1); */
              pathAux.instance().mapfSolver->computeRelocateCost(considered_vertex1, veh2, vtx2, veh1, vtx1);
              /* CVRPLIB_LOG_WARNING("This got called!"); */
              newSolution = pathAux.instance().mapfSolver->getBestSolution();

              if (newSolution.cost < cost){
                CVRPLIB_LOG_TRACE("MAPF Relocate improved cost from " << cost << " to " << newSolution.cost);
                mapfTask = newSolution.input;

                best_mapf = newSolution;
                pathAux.setCvrpPath(best_mapf.input.sequences);
                cost = best_mapf.cost;
                pathAux.rememberCost(cost);
                improvement = true;
              }


            } else {
              /* } else { */

              proposed_cost = pathAux.tryRelocate(veh1, veh2, vtx1, vtx2);

              if (proposed_cost == -1){
                /* CVRPLIB_LOG_DEBUG("Integrated MAPF returned invalid cost, skipping move ..."); */
              } else if (proposed_cost != -2) {
                if (proposed_cost < best_improved_cost){
                  best_improved_cost = proposed_cost;
                  improvement = true;
                  best_veh1 = veh1;
                  best_veh2 = veh2;
                  best_ver1 = vtx1;
                  best_ver2 = vtx2;
                  break;
                }
              }
            }
          }
        }
      }
    }
  }

  /// If we found a new best solution, store it, start from the beginning
  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    std::cout << "this got called.\n";
  } else if (improvement){
      CVRPLIB_LOG_TRACE("Relocate improved cost from " << cost << " to " << best_improved_cost);
      /* std::cout << pathAux.toString(); */
      /* CVRPLIB_LOG_DEBUG("Relocate end"); */
      pathAux.relocate(best_veh1, best_veh2, best_ver1, best_ver2);
      cost = best_improved_cost;
      if (RECOUNT_CHECK){
        int cost_verify = pathAux.computeTTD();
        if ((cost != cost_verify)){
          CVRPLIB_LOG_WARNING("Relocate cost and verify cost are mismatched!");
          CVRPLIB_LOG_INFO("relocate cost: " << cost << " verify: " << cost_verify);
          cost = cost_verify;
        } else {
          CVRPLIB_LOG_INFO("relocate cost: " << cost << " verify: " << cost_verify);
        }
      }
      /* CVRPLIB_LOG_WARNING("cost: " << cost); */
      /* CVRPLIB_LOG_WARNING("cost_verify: " << cost_verify); */
      pathAux.rememberCost(cost);
      /* pathAux.best_sol = best_mapf; */
  }

  bool ret = improvement;
  return ret;
}

bool cvrplib::ls::exchangeFA(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{

  bool improvement = false;
  uint16_t best_veh1 = 0, best_veh2 = 0, best_ver1 = 0, best_ver2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution;

  MAPF_input mapfTask;
  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    mapfTask.sequences = pathAux.getCvrpPath();
    pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);
  }
  /* CVRPLIB_LOG_DEBUG("exchange start"); */
  /* std::cout << pathAux.toString(); */
  /* usleep(500000); */

  /// Get operator starting time
  /* op_start_time = operator_clock.getTimeInSeconds(); */

  if (RECOUNT_CHECK){
  /* CVRPLIB_LOG_WARNING("exchange cost_start: " << cost); */
    if ((cost != pathAux.computeTTD())){
      CVRPLIB_LOG_WARNING("exchange cost at the start and verify cost are mismatched!");
    }
  }

  /// Iterate through all pairs of vehicles and all vertices of each
  for (uint16_t vehicle1 = 0; vehicle1 < numVehicles && !improvement; ++vehicle1){
    uint16_t num_vertices1 = pathAux.getNumVertices(vehicle1);
    for (uint16_t vtx1 = 1; (vtx1 < num_vertices1 - 1) && !improvement; ++vtx1){
      for (uint16_t vehicle2 = 0; vehicle2 < numVehicles && !improvement; ++vehicle2){
        /// Skip if exchanging vertices within the path .. maybe allow? 
        if (vehicle1 == vehicle2){
          continue;
        }
        /// vtx2 = vtx1 reduces computation time. 
        /// veh1 = 1, veh2 = 3, vtx1 = 5, vtx2 = 6
        /// is the same as
        /// veh1 = 3, veh2 = 1, vtx1 = 6, vtx1 = 5
        for (uint16_t vtx2 = vtx1; (vtx2 < pathAux.getNumVertices(vehicle2)-1) && !improvement; ++vtx2){
          /// Call internal MAPF operator if --mapf-ops
          if (pathAux.instance().mapfLevelOps && pathAux.useMapf){

            /* CVRPLIB_LOG_WARNING("This got called!"); */
            pathAux.instance().mapfSolver->computeExchange(vehicle1, vehicle2, vtx1, vtx2);
            newSolution = pathAux.instance().mapfSolver->getBestSolution();
            if (newSolution.cost < cost){
              // MOVE TO ANOTHER FUNCTION
              CVRPLIB_LOG_TRACE("MAPF Exchange improved cost from " << cost << " to " << newSolution.cost);
              /* CVRPLIB_LOG_DEBUG("Exchange end"); */
              best_mapf = newSolution;
              pathAux.setCvrpPath(best_mapf.input.sequences);
              cost = best_mapf.cost;
              pathAux.rememberCost(cost);
              improvement = true;
            }


          } else {

            proposed_cost = pathAux.tryExchange(vehicle1, vehicle2, vtx1, vtx2);

            if (proposed_cost == -1){
              CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
            } else {
              if (proposed_cost < best_improved_cost){
                best_improved_cost = proposed_cost;
                improvement = true;
                best_veh1 = vehicle1;
                best_veh2 = vehicle2;
                best_ver1 = vtx1;
                best_ver2 = vtx2;
              }
            }
          }
        }
      }
    }
  }

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
    /// otherwise if improvement, just call VRP operator with best indices.
  } else if (improvement){
      CVRPLIB_LOG_TRACE("Exchange improved cost from " << cost << " to " << best_improved_cost);
      /* std::cout << pathAux.toString(); */
      /* CVRPLIB_LOG_DEBUG("Exchange end"); */
      pathAux.exchange(best_veh1, best_veh2, best_ver1, best_ver2);
      cost = best_improved_cost;
      if (RECOUNT_CHECK){
        int cost_verify = pathAux.computeTTD();
        if (cost != cost_verify){
          CVRPLIB_LOG_WARNING("Exchange cost and verify cost are mismatched!");
          cost = cost_verify;
        } else {
          CVRPLIB_LOG_INFO("exchange cost: " << cost << " verify: " << cost_verify);
        }
        CVRPLIB_LOG_WARNING("exchange cost: " << cost);
        CVRPLIB_LOG_WARNING("exchange cost_verify: " << cost_verify);
      }
      pathAux.rememberCost(cost);
      /* pathAux.best_sol = best_mapf; */
  }

  bool ret = improvement;
  return ret;

}



bool cvrplib::ls::twoOptFA(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
 
  bool improvement=false;
  uint16_t best_veh1 = 0, best_ver1 = 0, best_ver2 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution;
  MAPF_input mapfTask;



  if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {
    mapfTask.sequences = pathAux.getCvrpPath();
    pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);
  }

  
  if (RECOUNT_CHECK){
    if (cost != pathAux.computeTTD()){
      CVRPLIB_LOG_WARNING("2opt cost at the start and verify cost are mismatched!");
    }
  }

  /// Explore 2opt neighbourhood
  for (uint16_t veh1 = 0; veh1 < numVehicles && !improvement; ++veh1){
    for (uint16_t vtx1 = 1; (vtx1 < pathAux.getNumVertices(veh1) - 3) && (!improvement); ++vtx1){
      for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2 && !improvement; ++vtx2){
        if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {

          pathAux.instance().mapfSolver->computeTwoOpt(veh1, vtx1, vtx2);
          newSolution = pathAux.instance().mapfSolver->getBestSolution();
          if (newSolution.cost < cost){
            CVRPLIB_LOG_TRACE("MAPF 2opt improved cost from " << cost << " to " << newSolution.cost);

            mapfTask = newSolution.input;
            best_mapf = newSolution;
            pathAux.setCvrpPath(best_mapf.input.sequences);
            cost = best_mapf.cost;
            pathAux.rememberCost(cost);
            improvement = true;
          }

        } else {

          proposed_cost = pathAux.tryTwoOptCvrp(veh1, vtx1, vtx2);

          if (proposed_cost == -1){
            CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
          } else {
            if (proposed_cost < best_improved_cost){
              best_improved_cost = proposed_cost;
              /* perform_three_opt = false; */
              improvement = true;
              best_veh1 = veh1;
              best_ver1 = vtx1;
              best_ver2 = vtx2;
            }
          }
        }
      }
    }
  }

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
  } else if (improvement){
      CVRPLIB_LOG_TRACE("2opt improved cost from " << cost << " to " << best_improved_cost);
      pathAux.twoOptCvrp(best_veh1, best_ver1, best_ver2);
      cost = best_improved_cost;
      if (RECOUNT_CHECK){
        int cost_verify = pathAux.computeTTD();
        if ((cost != cost_verify)){
          CVRPLIB_LOG_WARNING("Twoopt cost and verify cost are mismatched!");
          cost = cost_verify;
        } else {
          CVRPLIB_LOG_INFO("2opt cost: " << cost << " verify: " << cost_verify);
        }
      }
      pathAux.rememberCost(cost);
      /* pathAux.best_sol = best_mapf; */
  }

  bool ret = improvement;
  return ret;
}



bool cvrplib::ls::threeOptFA(
    solutions::Solution &pathAux,
    MAPF_solution &best_mapf,
    MainType &cost)
{
  bool improvement=false;
  uint16_t best_veh1 = 0, best_ver1 = 0, best_ver2 = 0, best_ver3 = 0;
  uint16_t numVehicles = pathAux.getNumVehicles();
  int proposed_cost = INT_MAX, best_improved_cost = cost;
  MAPF_solution newSolution;
  MAPF_input mapfTask;

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {
    mapfTask.sequences = pathAux.getCvrpPath();
    pathAux.instance().mapfSolver->setInitSolution(best_mapf, mapfTask);
  }
  
  if (RECOUNT_CHECK){
    if ((cost != pathAux.computeTTD())){
      CVRPLIB_LOG_WARNING("2opt cost at the start and verify cost are mismatched!");
    }
  }

  for (uint16_t veh1 = 0; veh1 < numVehicles && !improvement; ++veh1){
    for (uint16_t vtx1 = 1; vtx1 < pathAux.getNumVertices(veh1) - 3 && !improvement; ++vtx1){
      for (uint16_t vtx2 = vtx1+2; vtx2 < pathAux.getNumVertices(veh1) - 2 && !improvement; ++vtx2){
        for (uint16_t vtx3 = vtx2+2; vtx3 < pathAux.getNumVertices(veh1) - 2 && !improvement; ++vtx3){
          if (pathAux.instance().mapfLevelOps && pathAux.useMapf) {

            pathAux.instance().mapfSolver->computeThreeOpt(veh1, vtx1, vtx2, vtx3);
            newSolution = pathAux.instance().mapfSolver->getBestSolution();
            if (newSolution.cost < cost){
              CVRPLIB_LOG_TRACE("MAPF 3opt improved cost from " << cost << " to " << newSolution.cost);

              /* mapfTask = newSolution.input; */
              best_mapf = newSolution;
              cost = best_mapf.cost;
              pathAux.setCvrpPath(best_mapf.input.sequences);
              /* auto est_cost = pathAux.instance().mapfSolver->estimateCost(mapfTask); */

              /* /1* best_mapf = newSolution; *1/ */
              /* pathAux.setCvrpPath(best_mapf.input.sequences); */
              /* cost = best_mapf.cost; */
              /* pathAux.rememberCost(cost); */
              improvement = true;
            }

          } else {

            proposed_cost = pathAux.tryThreeOptCvrp(veh1, vtx1, vtx2, vtx3);

            if (proposed_cost == -1){
              CVRPLIB_LOG_DEBUG("MAPF returned invalid cost, skipping move ...");
            } else {
              if (proposed_cost < best_improved_cost){
                best_improved_cost = proposed_cost; 
                /* perform_three_opt = true; */
                improvement = true;
                best_veh1 = veh1;
                best_ver1 = vtx1;
                best_ver2 = vtx2;
                best_ver3 = vtx3;
              }
            }
          }
        }
      }
    }
  }

  if (pathAux.instance().mapfLevelOps && pathAux.useMapf){
  } else if (improvement){
      CVRPLIB_LOG_TRACE("3opt improved cost from " << cost << " to " << best_improved_cost);
      pathAux.threeOptCvrp(best_veh1, best_ver1, best_ver2, best_ver3);
      cost = best_improved_cost;
      if (RECOUNT_CHECK){
        int cost_verify = pathAux.computeTTD();
        if ((cost != cost_verify)){
          CVRPLIB_LOG_WARNING("3opt cost and verify cost are mismatched!");
          cost = cost_verify;
        } else {
          CVRPLIB_LOG_INFO("3opt cost: " << cost << " verify: " << cost_verify);
        }
      }
      pathAux.rememberCost(cost);
      /* pathAux.best_sol = best_mapf; */
  }

  bool ret = improvement;
  return ret;
}



void cvrplib::ls::perturbatevtwo(
        std::vector<PtbFunptrOperator> neighborhoods,
        solutions::Solution &pathRes,
        MainType &cost,
        uint16_t max_steps)
{
    int demand = 0;
    /* CVRPLIB_LOG_TRACE("Perturbation started"); */
    /* std::cout << pathRes.toString(); */
    for (uint16_t testveh = 0; testveh<pathRes.getCvrpPath().size(); ++testveh){
      demand = pathRes.computeDemands(testveh);
      if (demand > pathRes.instance().vehicleCapacity){
        CVRPLIB_LOG_WARNING("Path is wrong before perturbating " << testveh);
        CVRPLIB_LOG_WARNING("Demand: " << demand << " capacity: " << pathRes.instance().vehicleCapacity);

        for (uint16_t i = 0; i < pathRes.getCvrpPath().size(); ++i){
          for (uint16_t j = 0; j < pathRes.getCvrpPath()[i].size(); ++j){
            std::cout << pathRes.getCvrpPath()[i][j] << " ";
          }
          std::cout << "\n";
        }


        /* usleep(1); */
      }
    }
    std::vector<std::string> method_names;
    method_names.push_back("exchange");
    method_names.push_back("relocate");
    std::string selected_method_name;
    auto &n = pathRes.n();
    /* p = (p + 1 > n) ? n - 1 : p; */
    uint16_t operator_selector = 0, random_vehicle1 = 0, random_vehicle2 = 0, perturbation_steps = 0;

    uint16_t upper_v_lim = pathRes.instance().numVehicles - 1;
    uint16_t num_vehicles = pathRes.instance().numVehicles;

    auto &pathTemp = pathRes;
    Ver random_vertex1 = 0, random_vertex2 = 0;
    uint16_t num_methods = neighborhoods.size() - 1;

    operator_selector = random::UniformIntDistribution<uint16_t>{0, num_methods}(random::rng());
    selected_method_name = method_names[operator_selector];
    /* CVRPLIB_LOG_TRACE(" operator selector is: " << operator_selector << " , method name is: " << selected_method_name << "\n"); */ 
    perturbation_steps = random::UniformIntDistribution<uint16_t>{2, max_steps}(random::rng());
    
    bool perform_perturbation = false;
    std::vector<std::vector<Ver>> current_path;
    std::vector<int> open_list;

    for (uint16_t p_i = 0; p_i < perturbation_steps; p_i++){
      /* CVRPLIB_LOG_TRACE("Performing " << p_i << " p steps."); */
      /* CVRPLIB_LOG_TRACE("Operator selector is " << operator_selector); */
      // step 1: determine source vehicle
      // source vehicle must have at least 1 subgoal
      current_path = pathTemp.getCvrpPath();
      /* for (uint16_t i = 0; i < current_path.size(); ++i){ */
      /*   for (uint16_t j = 0; j < current_path[i].size(); ++j){ */
      /*     std::cout << current_path[i][j] << " "; */
      /*   } */
      /*   std::cout << "\n"; */
      /* } */

      open_list.clear();
      for (int i = 0; i < num_vehicles; ++i){
        if (pathTemp.getNumVertices(i) > 2){
          /* std::cout << "numver: " << i << " " << pathTemp.getNumVertices(i) << "\n"; */
          open_list.push_back(i);
        }
      }

      int open_list_max_value = open_list.size() - 1;
      /* std::cout << "ol max val for vehicle " << open_list_max_value << "\n"; */
      /* std::cout << "huh:" << open_list_max_value << "\n"; */
      int vehicle_1_index = random::UniformIntDistribution<int>{0, open_list_max_value}(random::rng());
      int vehicle_1 = open_list[vehicle_1_index];

      // step 2: determine source vertex
      open_list.clear();
      for(int i = 1; i < pathTemp.getNumVertices(vehicle_1)-1; ++i){
        /* std::cout << "index is " << i << "\n"; */
        open_list.push_back(i);
      }

      /* std::cout << open_list.size() << "\n"; */
      open_list_max_value = open_list.size() - 1;
      /* std::cout << "ol max val " << open_list_max_value << "\n"; */
      int vertex_1_index = random::UniformIntDistribution<int>{0, open_list_max_value}(random::rng());
      int vertex_1 = open_list[vertex_1_index];
      int vertex_1_id = current_path[vehicle_1][vertex_1];
      
      //
      // step 3: determine target vehicle
      // performing exchange
      open_list.clear();
      if (operator_selector == 0){
        for (int i = 0; i < num_vehicles; ++i){
          // if there's a vertex to exchange (demand does not change)
          if (pathTemp.getNumVertices(i) > 2){
            open_list.push_back(i);
          }
        }
      } else {
        //performing  relocate
        for (int i = 0; i < num_vehicles; ++i){
          // if we can fit the vertex inside (demand changes)
          if (pathTemp.canFitIntoCapacity(i, vertex_1_id)){
            open_list.push_back(i);
          }
        }
      }
      
      open_list_max_value = open_list.size() - 1;
      /* std::cout << "ol max val " << open_list_max_value << "\n"; */
      int vehicle_2_index = random::UniformIntDistribution<int>{0, open_list_max_value}(random::rng());
      int vehicle_2 = open_list[vehicle_2_index];

      // step 4: determine target vertex
      open_list.clear();
      //performing exchange
      if (operator_selector == 0){
        for(int i = 1; i < current_path[vehicle_2].size()-1; ++i){
          open_list.push_back(i);
        }
      } else if (operator_selector ==1){
        //performing relocate
        for(int i = 1; i < current_path[vehicle_2].size(); ++i){
          open_list.push_back(i);
        }
      }
      open_list_max_value = open_list.size() - 1;
      /* std::cout << "ol max val " << open_list_max_value << "\n"; */
      int vertex_2_index = random::UniformIntDistribution<int>{0, open_list_max_value}(random::rng());
      int vertex_2 = open_list[vertex_2_index];
      int vertex_2_id = current_path[vehicle_2][vertex_2];

      perform_perturbation = true;

      if (perform_perturbation){
        (pathTemp.*neighborhoods[operator_selector])(vehicle_1, vehicle_2, vertex_1, vertex_2);
      }
      // END PERTURBATION
    }

    pathRes = pathTemp;
    if (pathRes.useMapf){
        MAPF_input mapfTask;
        mapfTask.sequences = pathRes.getCvrpPath();
        MAPF_solution temp_mapf = pathRes.instance().mapfSolver->computeCost(mapfTask);
        cost = temp_mapf.cost;
        pathRes.resetDemands();
        /* CVRPLIB_LOG_INFO("Cost after perturbation is: " << cost); */
        return;
      
    } else {
      cost = pathRes.computeCost();
      pathRes.rememberCost(cost);
      pathRes.resetDemands();
    } 

    for (uint16_t testveh = 0; testveh<pathRes.getCvrpPath().size(); ++testveh){
      if ((demand = pathRes.computeDemands(testveh)) > pathRes.instance().vehicleCapacity){
        CVRPLIB_LOG_WARNING("Path is wrong" << testveh);
        CVRPLIB_LOG_WARNING("Demand: " << demand << " capacity: " << pathRes.instance().vehicleCapacity);
        /* sleep(1); */
      }
    }
    /* CVRPLIB_LOG_TRACE("Perturbation ended"); */
    /* std::cout << pathRes.toString(); */


}

void cvrplib::ls::perturbate(
        std::vector<PtbFunptrOperator> neighborhoods,
        solutions::Solution &pathRes,
        MainType &cost,
        uint16_t max_steps)
{
    /* return; */
    int demand = 0;
    for (uint16_t testveh = 0; testveh<pathRes.getCvrpPath().size(); ++testveh){
      demand = pathRes.computeDemands(testveh);
      if (demand > pathRes.instance().vehicleCapacity){
        CVRPLIB_LOG_WARNING("Path is wrong before perturbating " << testveh);
        CVRPLIB_LOG_WARNING("Demand: " << demand << " capacity: " << pathRes.instance().vehicleCapacity);
        /* sleep(1); */
      }
    }
    std::vector<std::string> method_names;
    method_names.push_back("exchange");
    method_names.push_back("relocate");
    std::string selected_method_name;
    auto &n = pathRes.n();
    /* p = (p + 1 > n) ? n - 1 : p; */
    uint16_t operator_selector = 0, random_vehicle1 = 0, random_vehicle2 = 0, perturbation_steps = 0;

    uint16_t upper_v_lim = pathRes.instance().numVehicles - 1;
    auto &pathTemp = pathRes;
    Ver random_vertex1 = 0, random_vertex2 = 0;
    uint16_t num_methods = neighborhoods.size() - 1;

    operator_selector = random::UniformIntDistribution<uint16_t>{0, num_methods}(random::rng());
    selected_method_name = method_names[operator_selector];
    /* CVRPLIB_LOG_TRACE(" operator selector is: " << operator_selector << " , method name is: " << selected_method_name << "\n"); */ 
    perturbation_steps = random::UniformIntDistribution<uint16_t>{2, max_steps}(random::rng());
    
    for (uint16_t p_i = 0; p_i < perturbation_steps; p_i++){

      /* std::cout << "perturbating\n"; */
                
      /// Determine the routes to perturbate
      random_vehicle1 = random::UniformIntDistribution<int>{0, upper_v_lim}(random::rng());
      random_vehicle2 = random::UniformIntDistribution<int>{0, upper_v_lim}(random::rng());
      /// Temporary fix, so that we filter out routes with only one vertex
      /// TODO: more elegant way
      // TODO: this may be broken now!
      while (!(pathRes.getNumVertices(random_vehicle1) > 2)){
        random_vehicle1 = random::UniformIntDistribution<int>{0, upper_v_lim}(random::rng());
      }
      /* veh1_max_v = std::max(pathAux.getNumVertices(rveh1) - 2, 1); */
      uint16_t veh1_max_v = pathRes.getNumVertices(random_vehicle1) - 2;
      random_vertex1 = random::UniformIntDistribution<int>{1, veh1_max_v}(random::rng());
      int v1_slots = pathRes.getNumVertices(random_vehicle1) - 2;
      // when there's only start and ending depot
      if (v1_slots == 0){
        CVRPLIB_LOG_WARNING("Bad route selected for perturbation.");
        sleep(1);
      }
      int v1_selected_slot = random::UniformIntDistribution<int>{1, v1_slots}(random::rng());
      v1_selected_slot = v1_selected_slot;
      random_vertex1 = v1_selected_slot;
      if (v1_selected_slot == 0 || v1_selected_slot == v1_slots + 1){
        CVRPLIB_LOG_WARNING("We have a problem");
        sleep(1);
      }


      uint16_t tries = 0;
      bool perform_perturbation = true;
      
      int required_min_vertex = 1;
      bool modifying_route_length = false;
      if (selected_method_name.compare("exchange") == 0){
        required_min_vertex = 2;
        modifying_route_length = false;
      } 
      if (selected_method_name.compare("relocate") == 0){
        modifying_route_length = true;
      }
      if (operator_selector == 1){
        modifying_route_length = true;
      }

      Ver random_vertex1_id = pathRes.getVerCvrp(random_vehicle1, random_vertex1);
      bool feasible_perturbation = (pathRes.getNumVertices(random_vehicle2) > required_min_vertex) && (random_vehicle2 != random_vehicle1);
      if (modifying_route_length){
        feasible_perturbation = ((pathRes.getNumVertices(random_vehicle2) > required_min_vertex) && pathRes.canFitIntoCapacity(random_vehicle2, random_vertex1_id) && !(random_vehicle2 == random_vehicle1));
      }
      /* while (!(pathRes.getNumVertices(random_vehicle2) > required_min_vertex) || pathRes.canFitIntoCapacity(random_vehicle2, random_vertex1) || (random_vehicle2 == random_vehicle1)){ */
      /* std::cout << "perturbation is feasible: " << feasible_perturbation << "with condition " << pathRes.canFitIntoCapacity(random_vehicle2, random_vertex1_id) << " are we modifying route length? " << modifying_route_length <<  "\n"; */

      if (modifying_route_length){
        /* std::cout << "rl of veh2: " << pathRes.getCvrpPath()[random_vehicle2].size() <<  "\n"; */
      }

      while (!feasible_perturbation){
        random_vehicle2 = random::UniformIntDistribution<int>{0, upper_v_lim}(random::rng());
        tries++;
        if (tries == 10){
          operator_selector = 0;
          perform_perturbation = false;
          break;
        }
        if (modifying_route_length){
          feasible_perturbation = ((pathRes.getNumVertices(random_vehicle2) > required_min_vertex) && pathRes.canFitIntoCapacity(random_vehicle2, random_vertex1_id) && !(random_vehicle2 == random_vehicle1));
        } else {
          feasible_perturbation = ((pathRes.getNumVertices(random_vehicle2) > required_min_vertex) && !(random_vehicle2 == random_vehicle1));
        }
        /* CVRPLIB_LOG_TRACE("perturbation is feasible: " << feasible_perturbation << "with condition " << pathRes.canFitIntoCapacity(random_vehicle2, random_vertex1) << "\n"); */
      }

      /// Determine the vertices for the perturbation operator
      uint16_t veh2_max_v = std::max(pathRes.getNumVertices(random_vehicle2) - 2, 1);
      int v2_slots = pathRes.getNumVertices(random_vehicle2) - 2;
      int random_slot = 1;
      if (v2_slots == 0){
        random_slot = 1;
      } else {
        random_slot = random::UniformIntDistribution<int>{1, v2_slots}(random::rng());
      }
      if (random_slot == 0 || random_slot == pathRes.getNumVertices(random_vehicle2) - 1){
        if (operator_selector == 0 && perform_perturbation){
          CVRPLIB_LOG_WARNING("We have a problem - vehicle2");
          sleep(1);
        } 
      }

      std::cout <<  "random number: " << random_slot << "\n";
      std::cout << "v2 slots " << v2_slots << "\n";
      /* sleep(1); */
      random_vertex2 = random::UniformIntDistribution<int>{1, veh2_max_v}(random::rng());
      /* int test = random::UniformIntDistribution<int>{1, 1}(random::rng()); */
      random_vertex2 = random_slot;
      if (perform_perturbation){
        pathTemp = pathRes;
        std::cout << "from path with size " << pathRes.getCvrpPath()[random_vehicle1].size() << " we chose vertex " << random_vertex1 << "\n";
        /* usleep(200000); */
        (pathTemp.*neighborhoods[operator_selector])(random_vehicle1, random_vehicle2, random_vertex1, random_vertex2);
        /* sleep(1); */
        /* if (operator_selector == 0){ */
        /*   /1* pathTemp.exchange(random_vehicle1, random_vehicle2, random_vertex1, random_vertex2); *1/ */
        /*   pathRes.exchange(random_vehicle1, random_vehicle2, random_vertex1, random_vertex2); */
        /* } else { */
        /*   /1* pathTemp.relocate(random_vehicle1, random_vehicle2, random_vertex1, random_vertex2); *1/ */
        /*   pathRes.relocate(random_vehicle1, random_vehicle2, random_vertex1, random_vertex2); */
        /* } */
      }
    }

    pathRes = pathTemp;
    if (pathRes.useMapf){
        MAPF_input mapfTask;
        mapfTask.sequences = pathRes.getCvrpPath();
        MAPF_solution temp_mapf = pathRes.instance().mapfSolver->computeCost(mapfTask);
        cost = temp_mapf.cost;
        /* CVRPLIB_LOG_INFO("Cost after perturbation is: " << cost); */
        return;
      
    } else {
      cost = pathRes.computeCost();
      pathRes.rememberCost(cost);
    }

    for (uint16_t testveh = 0; testveh<pathRes.getCvrpPath().size(); ++testveh){
      if ((demand = pathRes.computeDemands(testveh)) > pathRes.instance().vehicleCapacity){
        CVRPLIB_LOG_WARNING("Path is wrong" << testveh);
        CVRPLIB_LOG_WARNING("Demand: " << demand << " capacity: " << pathRes.instance().vehicleCapacity);
        /* usleep(50000); */
      }
    }


}



/* bool cvrplib::ls::twoOpt( */
/*         cvrplib::solutions::Solution &pathRes, */
/*         cvrplib::MainType &cost */
/* ) */
/* { */
/*     MainType impBest = 0; */
/*     Pos candI, candJ; */
/*     for (Pos i = 1; i < pathRes.n() - 1; ++i) { */
/*         for (Pos j = i + 1; j < pathRes.n(); ++j) { */
/*             auto impCurr = pathRes.improvPos2Opt(i, j); */
/*             if (impCurr < impBest) { */
/*                 impBest = impCurr; */
/*                 candI = i; */
/*                 candJ = j; */
/*             } */
/*         } */
/*     } */
/*     if (impBest < 0) { */
/*         pathRes.movePos2Opt(candI, candJ); */
/*         cost += impBest; */
/*         return true; */
/*     } else { */
/*         return false; */
/*     } */
/* } */

bool cvrplib::ls::twoString(
        cvrplib::solutions::Solution &pathRes,
        cvrplib::MainType &cost,
        Pos X,
        Pos Y
)
{
    MainType impBest = 0;
    Pos candI, candJ;
    for (Pos i = 0; i < pathRes.n() - X; ++i) {
        for (Pos j = (X == Y) ? i + X : 0; j < pathRes.n() - Y; ++j) {
            if (static_cast<int>(j) - static_cast<int>(i) >= static_cast<int>(X) ||
                static_cast<int>(i) - static_cast<int>(j) >= static_cast<int>(Y)) {
                auto impCurr = pathRes.improvPos2String(i, j, X, Y);
                if (impCurr < impBest) {
                    impBest = impCurr;
                    candI = i;
                    candJ = j;
                }
            }
        }
    }
    if (impBest < 0) {
        pathRes.movePos2String(candI, candJ, X, Y);
        cost += impBest;
        return true;
    } else {
        return false;
    }
}

bool cvrplib::ls::onePoint(
        solutions::Solution &pathRes,
        MainType &cost
)
{
    MainType impBest = 0;
    Pos candI, candJ;
    for (Pos i = 0; i < pathRes.n(); ++i) {
        for (Pos j = 0; j < pathRes.n() - 1; ++j) {
            if (i < j || j + 1 < i) {
                auto impCurr = pathRes.improvPos1Point(i, j);
                if (impCurr < impBest) {
                    impBest = impCurr;
                    candI = i;
                    candJ = j;
                }
            }
        }
    }
    if (impBest < 0) {
        pathRes.movePos1Point(candI, candJ);
        cost += impBest;
        return true;
    } else {
        return false;
    }
}

bool cvrplib::ls::twoPoint(
        solutions::Solution &pathRes,
        MainType &cost
)
{
    MainType impBest = 0;
    Pos candI, candJ;
    for (Pos i = 0; i < pathRes.n() - 1; ++i) {
        for (Pos j = i + 1; j < pathRes.n() - 1; ++j) {
            auto impCurr = pathRes.improvPos2Point(i, j);
            if (impCurr < impBest) {
                impBest = impCurr;
                candI = i;
                candJ = j;
            }
        }
    }
    if (impBest < 0) {
        pathRes.movePos2Point(candI, candJ);
        cost += impBest;
        return true;
    } else {
        return false;
    }
}



/* void cvrplib::ls::perturbate( */
/*         solutions::Solution &pathRes, */
/*         MainType &cost, */
/*         unsigned p */
/* ) */
/* { */
/*     auto &n = pathRes.n(); */
/*     p = (p + 1 > n) ? n - 1 : p; */

/*     /// First randomly choose p different positions on the path. */
/*     std::vector<bool> throwsBool(n, false); */
/*     for (unsigned i = 0; i < p; ++i) { */
/*         int throwI; */
/*         do { */
/*             throwI = random::UniformIntDistribution<unsigned>{0, n - 2}(random::rng()); */
/*         } while (throwsBool[throwI]); */
/*         throwsBool[throwI] = true; */
/*     } */

/*     /// Then split the path to (p + 1) strings : */
/*     ///     pathStrings[0]: < 0, 1, ... , throw(1) >, */
/*     ///     pathStrings[1]: < throw(1) + 1, ... , throw(2) > */
/*     ///     ... */
/*     ///     pathStrings[p - 1]: < throw(p - 1) + 1, ... , throw(p) >, */
/*     ///     pathStrings[p]: < throw(p) + 1, ... , n > */
/*     std::vector<std::vector<int>> pathStrings{}; */
/*     for (unsigned i = 0; i < n; ++i) { */
/*         std::vector<int> pathString{}; */
/*         while (!throwsBool[i] && i < n - 1) { */
/*             pathString.emplace_back(i++); */
/*         } */
/*         pathString.emplace_back(i); */
/*         pathStrings.push_back(pathString); */
/*     } */

/*     /// Create new path that starts with the first string. */
/*     std::vector<int> newPositions(pathStrings[0]); */

/*     /// Create roulette and shuffle it to get an order in which the rest of the strings will be connected. */
/*     std::vector<int> roulette = std::vector<int>(pathStrings.size()); */
/*     for (unsigned i = 0; i < roulette.size(); ++i) roulette[i] = i; */
/*     std::shuffle(roulette.begin() + 1, roulette.end(), random::rng()); */

/*     /// Connect the strings to get the new path. */
/*     for (unsigned k = 1; k < pathStrings.size(); ++k) { */

/*         /// Pick a next string to be connected to the new path. */
/*         auto &nextString = pathStrings[roulette[k]]; */

/*         /// Flip a coin. */
/*         auto side = random::UniformIntDistribution<unsigned>{0, 1}(random::rng()); */
/*         if (side == 0) { */

/*             /// If side == 0, then connect the next string in ascending order. */
/*             for (int i : nextString) { */
/*                 newPositions.push_back(i); */
/*             } */

/*         } else { */

/*             /// If side == 1, then connect the next string in descending order. */
/*             for (unsigned i = 0; i < nextString.size(); ++i) { */
/*                 newPositions.push_back(nextString[nextString.size() - i - 1]); */
/*             } */
/*         } */
/*     } */

/*     /// Now get the path's vertices. */
/*     std::vector<int> newPath(n); */
/*     for (unsigned j = 0; j < n; ++j) { */
/*         newPath[j] = pathRes.getVer(static_cast<Pos>(newPositions[j])); */
/*     } */

/*     /// Update the resulting path. */
/*     for (unsigned j = 0; j < n; ++j) { */
/*         pathRes.set(newPath[j], j); */
/*     } */
/*     pathRes.updateStructures(); */

/*     /// Finally add the new cost of the path. */
/*     cost = pathRes.computeCost(); */
/* } */

void cvrplib::ls::vnd(
        solutions::Solution &pathRes,
        MainType &cost,
        const Goals &goals,
        bool &interrupt
)
{

    /// Find best neighbor within the picked neighborhood and if it is better,
    /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence.
    /* while (twoOpt(pathRes, cost) */
    /*        || onePoint(pathRes, cost) */
    /*        || orOpt2(pathRes, cost) */
    /*        || orOpt3(pathRes, cost) */
    /*        || orOpt4(pathRes, cost)) { */
    /*     /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*     if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*         || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*         interrupt = true; */
    /*         break; */
    /*     } */
    /* } */
}

void cvrplib::ls::vndParam(
        solutions::Solution &pathRes,
        MainType &cost,
        const Goals &goals,
        bool &interrupt,
        const unsigned operatorsSeq
)
{
    /* switch (operatorsSeq) { */
    /*     case 1: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt2(pathRes, cost) */
    /*                || orOpt4(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     case 2: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt3(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     case 3: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt2(pathRes, cost) */
    /*                || orOpt4(pathRes, cost) */
    /*                || twoPoint(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     case 4: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt2(pathRes, cost) */
    /*                || orOpt3(pathRes, cost) */
    /*                || orOpt4(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     case 5: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt2(pathRes, cost) */
    /*                || orOpt3(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     case 6: { */
    /*         /// Find best neighbor within the picked neighborhood and if it is better, */
    /*         /// then assign the neighbor to pathRes and repeat, if not then try the next neighborhood in a sequence. */
    /*         while (twoOpt(pathRes, cost) */
    /*                || onePoint(pathRes, cost) */
    /*                || orOpt2(pathRes, cost) */
    /*                || orOpt3(pathRes, cost) */
    /*                || orOpt4(pathRes, cost) */
    /*                || twoPoint(pathRes, cost)) { */
    /*             /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
    /*             if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
    /*                 || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
    /*                 interrupt = true; */
    /*                 break; */
    /*             } */
    /*         } */
    /*         break; */
    /*     } */
    /*     default: { */
    /*         CVRPLIB_LOG_FATAL("Unknown operators' sequence flag!"); */
    /*         exit(CVRPLIB_EXIT_FAILURE); */
    /*     } */
    /* } */
}

void cvrplib::ls::rvnd(
        solutions::Solution &pathRes,
        MainType &cost,
        const Goals &goals,
        bool &interrupt
)
{

/*     /// Create array of neighborhoods. */
/*     using FunptrOperator = bool (*)( */
/*             solutions::Solution &, */
/*             MainType & */
/*     ); */
/*     std::vector<FunptrOperator> neighborhoods{twoPoint, twoOpt, onePoint, orOpt2, orOpt3}; */

/*     /// nlSize ~ number of available neighborhoods */
/*     auto nlSize = static_cast<unsigned>(neighborhoods.size()); */

/*     /// available[i] == true ~ neighborhood i is available */
/*     std::vector<bool> available(neighborhoods.size(), true); */

/*     /// Iterate until there are available neighborhoods. */
/*     while (nlSize > 0) { */

/*         /// Pick neighborhood index randomly among available neighborhoods. */
/*         auto neighborhoodIdx = random::getRandomAmongAvailable(nlSize, available); */

/*         /// Find best neighbor within the picked neighborhood and if it is better, then assign the neighbor to pathRes. */
/*         if ((*neighborhoods[neighborhoodIdx])(pathRes, cost)) { */

/*             /// If success, then set all neighborhoods to be available again. */
/*             nlSize = static_cast<unsigned>(neighborhoods.size()); */
/*             std::fill(available.begin(), available.end(), true); */

/*         } else { */

/*             /// If not success, then make the current neighborhood unavailable and decrease nlSize. */
/*             available[neighborhoodIdx] = false; */
/*             --nlSize; */

/*         } */

/*         /// If the time limit or cost goal are met, then break the loop and set interrupt to true. */
/*         if ((goals.isGoalTimeSet() && goals.isGoalTimeMet()) */
/*             || (goals.isGoalCostSet() && goals.isGoalCostMet(cost))) { */
/*             interrupt = true; */
/*             break; */
/*         } */
/*     } */
}



