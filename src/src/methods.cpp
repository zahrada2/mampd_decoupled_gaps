/**
 * File:    methods.cpp
 *
 * Date:    09/02/2020
 * Author:  David Zahradka 
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

/// TODO: add penalties to cost calculation
/// TODO: add interface for MAPF
/// TODO: add best solution memory
/// TODO: fix something wonky with price calculation

#include "methods.hpp"
#include "local_search.hpp"
#include "constructives.hpp"

#include <algorithm>

using namespace cvrplib;
using namespace cvrplib::solutions;
using namespace cvrplib::methods;




void cvrplib::methods::fmivns(const References &refs,
                              unsigned maxIter
                              )
{
  auto &path0 = refs.pathAux1;
  auto &pathAux = refs.pathAux2;
  auto &pathTemp = refs.pathAux3;
  auto &pathRes = refs.pathRes;
  auto &costRes = refs.cost;
  const auto &goals = refs.goals;
  
  /// Set the resulting cost to infinity.
  costRes = std::numeric_limits<MainType>::max();

  CVRPLIB_LOG_INFO("Creating path0......");
  MainType costPath0;
  MAPF_solution best_mapf;

  constructives::rand(pathAux, best_mapf, costPath0);

  pathRes = pathAux;
  // load initial cost into costPathAux work variable
  MainType costPathAux = costPath0;
  costRes = costPath0;
  MainType costIterationStart = costPath0;
  MainType costFasterIterationStart = costPath0;
  int numVehicles = pathAux.getNumVehicles();
  for (int car = 0; car < numVehicles; ++car){
    std::cout << "Initial vehicle route " << car << ":\n";
    std::cout << pathAux.toString();
    /* for (int rndm = 0; rndm < pathAux.getNumVertices(car); ++rndm){ */
    /*   std::cout << pathAux.getVerCvrp(car, rndm); */
    /*   if (rndm != pathAux.getNumVertices(car) - 1){ */
    /*     std::cout << " - "; */
    /*   } */
    /* } */
    /* std::cout << "\n"; */
    /* std::cout << "Size: " << pathAux.getNumVertices(car) << "\n"; */
  }

  pathRes.best_own_result = costPathAux;

  /// This is the main iteration counter.
  unsigned i = 0;

  /// Parameters used in the two-phase metaheuristic article
  /* uint16_t maxIter = 3000; */
  /* uint16_t maxIter = pathAux.inst->numIter; */
  /// The frequency (= num of iterations) of heartbeat and best-so-far output stdout message
  uint16_t log_freq = maxIter/10;
  uint16_t heartbeat_freq = std::min(log_freq/5, 500);
  /* uint16_t heartbeat_freq = 50; */
  uint16_t T = 10;
  uint16_t upper_v_lim = numVehicles-1;
              
  /// Pre-declare variables used in the VNS loop
  MAPF_input mapfTask;
  MAPF_solution newSolution;
  
  SimpleClock clock1{};
  double time1 = 0, time2 = 0;
    
  SimpleClock operator_clock{};
  double op_start_time = 0, op_end_time = 0;

  std::vector<double> vrp_operator_times = {0,0,0,0,0};
  std::vector<double> mapf_operator_times = {0,0,0,0,0};
  std::vector<long> mapf_op_calls = {0,0,0,0,0};
  std::vector<long> vrp_op_calls = {0,0,0,0,0};
  std::vector<MainType> cfv;
  std::vector<long> improvements;
  std::vector<long> costFunctionValues;

  // true for first accept instead of best accept
  bool first_accept = false;


  CVRPLIB_LOG_INFO("Entering main simplified VNS loop");

  /* bool logTimes = true // not used now */
  bool twoRounds = false;
  bool mapfRunning = false;
  if (pathAux.instance().useIntegratedMapf){
    twoRounds = true;
  }
  costIterationStart = costRes;
  costFasterIterationStart = costRes;



  using FunptrOperator = bool (*)(
        solutions::Solution &,
        MAPF_solution &,
        MainType &
  );

  using PtbFunptrOperator = void (Solution::*)(
      uint16_t,
      uint16_t,
      Pos,
      Pos

  );

  std::vector<PtbFunptrOperator> perturbation_neighborhoods{&Solution::exchange, &Solution::relocate};
  std::vector<FunptrOperator> local_search_neighborhoods;
  if (pathAux.instance().useFa){
    local_search_neighborhoods = std::vector<FunptrOperator>{ls::exchangeFA, ls::relocateFA, ls::twoOptFA, ls::threeOptFA};
  } else {
    local_search_neighborhoods = std::vector<FunptrOperator>{ls::exchange, ls::relocate, ls::twoOpt, ls::threeOpt};
  }
  bool load_from_file = pathRes.instance().loadSolution;

  for (uint16_t cycle = 0; cycle < 1  + (int) twoRounds; ++cycle){

    /* if (load_from_file){ */
    /* } */
    
    pathAux = pathRes;

    pathAux.useMapf = false;
    pathRes.useMapf = false;

    while (i < maxIter){

      MAPF_solution temp_mapf = best_mapf;

      /* std::cout << "iteration " << i << " before exploration \n"; */
      /* for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){ */
      /*   pathAux.computeDemands(vehicle); */
      /* } */
      /* std::cout << "starting exploration \n"; */
      MainType new_cost = costPathAux;
      cfv.push_back(new_cost);

      ls::exploreVnsNeighbourhoods(local_search_neighborhoods, pathAux, temp_mapf, costPathAux);


      /* std::cout << "exploration ended\n"; */
      
      /* std::cout << "iteration " << i << " after exploration \n"; */
      /* for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){ */
      /*   pathAux.computeDemands(vehicle); */
      /* } */

      /// Outer evaluation phase.
      if (costPathAux < pathRes.best_own_result){
        pathRes.best_own_result = costPathAux;
        pathAux.best_own_result = costPathAux;
      } 

      if (costPathAux < costRes) {

        /// pathRes <- pathAux
        /* pathRes.clone(pathAux); */
        pathRes = pathAux; 
        /* pathRes.clone(pathAux); */
        costRes = costPathAux;
        best_mapf = temp_mapf;

        if (!pathAux.useMapf){
          pathRes.setMinimumVrpCost(costRes);
          pathAux.setMinimumVrpCost(costRes);
        }


        pathRes.setIterationStamp(i);

      }

      pathRes.setVrpTimes(vrp_operator_times);
      pathRes.setMapfTimes(mapf_operator_times);
      pathRes.setVrpOpCounts(vrp_op_calls);
      pathRes.setMapfOpCounts(mapf_op_calls);
      
      /// Logging
      /// Move this to separate method
      if (!(i%heartbeat_freq) && (i != 0)){
        improvements.push_back(costFasterIterationStart - costRes);
        costFunctionValues.push_back(costRes);
        pathRes.setImprovements(improvements);
        pathRes.setCostFunctionValues(costFunctionValues);
        costFasterIterationStart = costRes;
        if (!(i%log_freq)){
          if (twoRounds){
            CVRPLIB_LOG_INFO("Running with integrated MAPF");
          }
          /* pathRes.printOperatorStats(); */
          /* printf("-----------------\n"); */
          CVRPLIB_LOG_INFO("Stats for iteration " << i);
          MainType costDifference = costRes - costIterationStart;
          costIterationStart = costRes;
          time2 = clock1.getTimeInSeconds();
          CVRPLIB_LOG_INFO("Cost reduction after " << log_freq << " iterations: " << costDifference);
          CVRPLIB_LOG_INFO("The time to perform " << log_freq <<  " iterations was " << time2 - time1 << "s.");
          CVRPLIB_LOG_INFO("The best-so-far solution is:");
          pathRes.printSolution();
          CVRPLIB_LOG_INFO("There are " << maxIter-i << " iterations remaining.");
          time1 = time2;
        }
        CVRPLIB_LOG_INFO("Current lowest cost is: " << costRes << " and current iteration is " << i << "."); 
      } 

      ls::perturbatevtwo(perturbation_neighborhoods, pathAux, costPathAux, 10);

      ++i;
    }
  }
  MainType best_vrp_cost = costRes;
  MainType found_cost;
  std::vector<uint16_t> histogram(25, 0);
  for (uint16_t i = 0; i < cfv.size(); ++i){
    found_cost = cfv[i];
    float diff = (float) (found_cost - best_vrp_cost);
    float gap = ((diff)/best_vrp_cost)*100;
    int bin = static_cast<int>(gap);
    CVRPLIB_LOG_TRACE("Best cost is: " <<  best_vrp_cost << " current cost is: " << found_cost << " diff is: " << diff << " that puts it into bin " << bin);
    ++histogram[bin];
  }
  pathRes.setHistogram(histogram);
}


void cvrplib::methods::simpleTwoPhase(const References &refs,
                                      unsigned maxIter)
{
  auto &path0 = refs.pathAux1;
  auto &pathAux = refs.pathAux2;
  auto &pathTemp = refs.pathAux3;
  auto &pathRes = refs.pathRes;
  auto &costRes = refs.cost;
  const auto &goals = refs.goals;
  
  /// Set the resulting cost to infinity.
  costRes = std::numeric_limits<MainType>::max();

  CVRPLIB_LOG_INFO("Creating path0......");
  MainType costPath0;
  MAPF_solution best_mapf;

  constructives::rand(pathAux, best_mapf, costPath0);

  pathRes = pathAux;
  // load initial cost into costPathAux work variable
  MainType costPathAux = costPath0;
  costRes = costPath0;
  MainType costIterationStart = costPath0;
  MainType costFasterIterationStart = costPath0;
  int numVehicles = pathAux.getNumVehicles();
  for (int car = 0; car < numVehicles; ++car){
    std::cout << "Initial vehicle route " << car << ":\n";
    for (int rndm = 0; rndm < pathAux.getNumVertices(car); ++rndm){
      std::cout << pathAux.getVerCvrp(car, rndm);
      if (rndm != pathAux.getNumVertices(car) - 1){
        std::cout << " - ";
      }
    }
    std::cout << "\n";
    std::cout << "Size: " << pathAux.getNumVertices(car) << "\n";
  }

  /// This is the main iteration counter.
  unsigned i = 0;

  /// Parameters used in the two-phase metaheuristic article
  /* uint16_t maxIter = 3000; */
  /* uint16_t maxIter = pathAux.inst->numIter; */
  /// The frequency (= num of iterations) of heartbeat and best-so-far output stdout message
  uint16_t log_freq = maxIter/10;
  uint16_t heartbeat_freq = std::min(log_freq/5, 250);
  uint16_t T = 10;
  uint16_t upper_v_lim = numVehicles-1;
              
  /// Pre-declare variables used in the VNS loop
  MAPF_input mapfTask;
  MAPF_solution newSolution;
  
  SimpleClock clock1{};
  double time1 = 0, time2 = 0;
    
  SimpleClock operator_clock{};
  double op_start_time = 0, op_end_time = 0;

  std::vector<double> vrp_operator_times = {0,0,0,0,0};
  std::vector<double> mapf_operator_times = {0,0,0,0,0};
  std::vector<long> mapf_op_calls = {0,0,0,0,0};
  std::vector<long> vrp_op_calls = {0,0,0,0,0};
  std::vector<long> improvements;
  std::vector<long> costFunctionValues;

  // true for first accept instead of best accept
  bool first_accept = false;


  CVRPLIB_LOG_INFO("Entering main simple Two Phase VNS loop");



  /* bool logTimes = true // not used now */
  bool twoRounds = false;
  bool mapfRunning = false;
  if (pathAux.instance().useIntegratedMapf){
    twoRounds = true;
  }
  costIterationStart = costRes;
  costFasterIterationStart = costRes;




  // wow this is black magic
  using FunptrOperator = bool (*)(
        solutions::Solution &,
        MAPF_solution &,
        MainType &
  );

  using PtbFunptrOperator = void (Solution::*)(
      uint16_t,
      uint16_t,
      Pos,
      Pos

  );

  /* std::vector<FunptrOperator> local_search_neighborhoods{ls::exchange, ls::relocate, ls::twoOpt, ls::threeOpt}; */
  /* std::vector<PtbFunptrOperator> perturbation_neighborhoods{&Solution::exchange, &Solution::relocate}; */

  std::vector<FunptrOperator> ls1_neighborhoods{ls::exchange, ls::relocate, ls::twoOpt, ls::threeOpt};
  std::vector<FunptrOperator> ls2_neighborhoods{ls::exchange, ls::relocate, ls::twoOpt, ls::threeOpt};
  std::vector<PtbFunptrOperator> perturbation1_neighborhoods{&Solution::exchange, &Solution::relocate};
  /* std::vector<PtbFunptrOperator> perturbation2_neighborhoods{Solution::twoOpt, &Solution::threeOpt}; */

  for (uint16_t cycle = 0; cycle < 1  + (int) twoRounds; ++cycle){
    pathAux = pathRes;
    pathAux.useMapf = false;







    if (cycle == 1){
      // reset for second loop, turn on mapf
      pathAux.useMapf = true;
      mapfTask.sequences = pathAux.getCvrpPath();
      best_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask);
      costPathAux = best_mapf.cost;
      costRes = costPathAux;

      i = 0;
    }


    bool internal_improvement = false;
    while (i < maxIter){

      ls::perturbate(perturbation1_neighborhoods, pathAux, costPathAux, 10);

      ls::exploreVnsNeighbourhoods(ls1_neighborhoods, pathAux, best_mapf, costPathAux);

      /* ls::perturbate(perturbation2_neighborhoods, pathAux, costPathAux, 10); */

      internal_improvement = false;
      while(ls::twoOpt(pathAux, best_mapf, costPathAux)){
        internal_improvement = true;
      }

      if (internal_improvement){
        ls::exploreVnsNeighbourhoods(ls2_neighborhoods, pathAux, best_mapf, costPathAux);
      }

      /// Outer evaluation phase.
      if (costPathAux < costRes) {

        /// pathRes <- pathAux
        /* pathRes.clone(pathAux); */
        pathRes = pathAux; 
        /* pathRes.clone(pathAux); */
        costRes = costPathAux;


        pathRes.setIterationStamp(i);

      }

      pathRes.setVrpTimes(vrp_operator_times);
      pathRes.setMapfTimes(mapf_operator_times);
      pathRes.setVrpOpCounts(vrp_op_calls);
      pathRes.setMapfOpCounts(mapf_op_calls);
      
      /// Logging
      /// Move this to separate method
      if (!(i%heartbeat_freq) && (i != 0)){
        improvements.push_back(costFasterIterationStart - costRes);
        costFunctionValues.push_back(costRes);
        pathRes.setImprovements(improvements);
        pathRes.setCostFunctionValues(costFunctionValues);
        costFasterIterationStart = costRes;
        if (!(i%log_freq)){
          if (twoRounds){
            CVRPLIB_LOG_INFO("Running with integrated MAPF");
          }
          /* pathRes.printOperatorStats(); */
          printf("-----------------");
          CVRPLIB_LOG_INFO("Stats for iteration " << i);
          MainType costDifference = costRes - costIterationStart;
          costIterationStart = costRes;
          time2 = clock1.getTimeInSeconds();
          CVRPLIB_LOG_INFO("Cost reduction after " << log_freq << " iterations: " << costDifference);
          CVRPLIB_LOG_INFO("The time to perform " << log_freq <<  " iterations was " << time2 - time1 << "s.");
          CVRPLIB_LOG_INFO("The best-so-far solution is:");
          pathRes.printSolution();
          CVRPLIB_LOG_INFO("There are " << maxIter-i << " iterations remaining.");
          time1 = time2;
        }
        CVRPLIB_LOG_INFO("Current lowest cost is: " << costRes << " and current iteration is " << i << "."); 
      } 

      ++i;
    }
  }
}

void cvrplib::methods::simplified_vns(const References &refs,
                                            unsigned maxIter
                                            )
{
  auto &path0 = refs.pathAux1;
  auto &pathAux = refs.pathAux2;
  auto &pathTemp = refs.pathAux3;
  auto &pathRes = refs.pathRes;
  auto &costRes = refs.cost;
  const auto &goals = refs.goals;
  
  /// Set the resulting cost to infinity.
  costRes = std::numeric_limits<MainType>::max();

  CVRPLIB_LOG_INFO("Creating path0......");
  MainType costPath0;
  MAPF_solution best_mapf;

  constructives::rand(pathAux, best_mapf, costPath0);

  pathRes = pathAux;
  // load initial cost into costPathAux work variable
  MainType costPathAux = costPath0;
  costRes = costPath0;
  MainType costIterationStart = costPath0;
  MainType costFasterIterationStart = costPath0;
  int numVehicles = pathAux.getNumVehicles();
  for (int car = 0; car < numVehicles; ++car){
    std::cout << "Initial vehicle route " << car << ":\n";
    for (int rndm = 0; rndm < pathAux.getNumVertices(car); ++rndm){
      std::cout << pathAux.getVerCvrp(car, rndm);
      if (rndm != pathAux.getNumVertices(car) - 1){
        std::cout << " - ";
      }
    }
    std::cout << "\n";
    std::cout << "Size: " << pathAux.getNumVertices(car) << "\n";
  }
  pathRes.best_own_result = costPathAux;

  /// This is the main iteration counter.
  unsigned i = 0;

  /// Parameters used in the two-phase metaheuristic article
  /* uint16_t maxIter = 3000; */
  /* uint16_t maxIter = pathAux.inst->numIter; */
  /// The frequency (= num of iterations) of heartbeat and best-so-far output stdout message
  uint16_t log_freq = maxIter/10;
  uint16_t heartbeat_freq = std::min(log_freq/5, 250);
  /* uint16_t heartbeat_freq = 50; */
  uint16_t T = 10;
  uint16_t upper_v_lim = numVehicles-1;
              
  /// Pre-declare variables used in the VNS loop
  MAPF_input mapfTask;
  MAPF_solution newSolution;
  
  SimpleClock clock1{};
  double time1 = 0, time2 = 0;
    
  SimpleClock operator_clock{};
  double op_start_time = 0, op_end_time = 0;

  std::vector<double> vrp_operator_times = {0,0,0,0,0};
  std::vector<double> mapf_operator_times = {0,0,0,0,0};
  std::vector<long> mapf_op_calls = {0,0,0,0,0};
  std::vector<long> vrp_op_calls = {0,0,0,0,0};
  std::vector<long> improvements;
  std::vector<long> costFunctionValues;

  // true for first accept instead of best accept
  bool first_accept = false;


  CVRPLIB_LOG_INFO("Entering main simplified VNS loop");



  /* bool logTimes = true // not used now */
  bool twoRounds = false;
  bool mapfRunning = false;
  if (pathAux.instance().useIntegratedMapf){
    twoRounds = true;
  }
  costIterationStart = costRes;
  costFasterIterationStart = costRes;



  using FunptrOperator = bool (*)(
        solutions::Solution &,
        MAPF_solution &,
        MainType &
  );

  using PtbFunptrOperator = void (Solution::*)(
      uint16_t,
      uint16_t,
      Pos,
      Pos

  );

  std::vector<PtbFunptrOperator> perturbation_neighborhoods{&Solution::exchange, &Solution::relocate};
  std::vector<FunptrOperator> local_search_neighborhoods;
  if (pathAux.instance().useFa){
    std::cout << "using FA\n";
    local_search_neighborhoods = std::vector<FunptrOperator>{ls::exchangeFA, ls::relocateFA, ls::twoOptFA, ls::threeOptFA};
  } else {
    local_search_neighborhoods = std::vector<FunptrOperator>{ls::exchange, ls::relocate, ls::twoOpt, ls::threeOpt};
  }
  bool load_from_file = pathRes.instance().loadSolution;

  for (uint16_t cycle = 0; cycle < 1  + (int) twoRounds; ++cycle){

    /* if (load_from_file){ */
    /* } */
    
    pathAux = pathRes;

    pathAux.useMapf = false;

    if (cycle == 1 || load_from_file){
      // reset for second loop, turn on mapf
      CVRPLIB_LOG_INFO("Starting MAPF-integrated cycle");
      pathAux.useMapf = true;
      /* if (!load_from_file){ */
      mapfTask.sequences = pathAux.getCvrpPath();
      best_mapf = pathAux.instance().mapfSolver->computeCost(mapfTask);
      if (!load_from_file){

        pathRes.setMinimumVrpCost(costRes);
        pathAux.setMinimumVrpCost(costRes);
        costPathAux = best_mapf.cost;
        pathRes.previous_cost = costPathAux;
        pathAux.previous_cost = costPathAux;
        maxIter = 100;
        // different number of maxiter?
        costRes = costPathAux;
        pathAux.useMapf = true;
      }
      CVRPLIB_LOG_INFO("MAPF cost is: " << best_mapf.cost);
      if (best_mapf.cost > 100000){
        CVRPLIB_LOG_WARNING("Could not solve MAPF instance!");
        costPathAux = 10000;
      }
      /* } */
      if (load_from_file){
        /* MAPF_solution discard; */
        /* /1* cvrplib::methods::loadXML("test", discard, pathRes, maxIter); *1/ */
        /* constructives::loadRoute("test", discard, pathRes, maxIter); */
        /* int original_vrp_cost = pathRes.computeCost(); */
        /* pathRes.setMinimumVrpCost(original_vrp_cost); */
        /* // this needs to be after loading! */
        /* maxIter = 500; */
        /* cycle = 1; */
        /* costPathAux = pathRes.recallCost(); */
        /* costRes = costPathAux; */
        /* CVRPLIB_LOG_WARNING("Loaded MAPF solution, skipping straight towards integrated calculation!"); */
        /* CVRPLIB_LOG_INFO("Recalled cost " << costRes << "\n"); */
        /* pathAux = pathRes; */
        /* pathAux.useMapf = true; */
      }

      i = 0;
    }

    while (i < maxIter){
      MAPF_solution temp_mapf = best_mapf;


      /* std::cout << "iteration " << i << " before exploration \n"; */
      /* for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){ */
      /*   pathAux.computeDemands(vehicle); */
      /* } */
      /* std::cout << "starting exploration \n"; */
      ls::exploreVnsNeighbourhoods(local_search_neighborhoods, pathAux, temp_mapf, costPathAux);
      /* std::cout << "exploration ended\n"; */
      
      /* std::cout << "iteration " << i << " after exploration \n"; */
      /* for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){ */
      /*   pathAux.computeDemands(vehicle); */
      /* } */

      /// Outer evaluation phase.
      if (costPathAux < pathRes.best_own_result){
        pathRes.best_own_result = costPathAux;
        pathAux.best_own_result = costPathAux;
      } 

      if (costPathAux < costRes) {
        if (pathAux.useMapf){
          CVRPLIB_LOG_WARNING("Updating cost from " << costRes << " to " << costPathAux);
        }

        /// pathRes <- pathAux
        /* pathRes.clone(pathAux); */
        pathRes = pathAux; 
        /* pathRes.clone(pathAux); */
        costRes = costPathAux;
        best_mapf = temp_mapf;

        if (!pathAux.useMapf){
          pathRes.setMinimumVrpCost(costRes);
          pathAux.setMinimumVrpCost(costRes);
        }


        pathRes.setIterationStamp(i);

      }

      pathRes.setVrpTimes(vrp_operator_times);
      pathRes.setMapfTimes(mapf_operator_times);
      pathRes.setVrpOpCounts(vrp_op_calls);
      pathRes.setMapfOpCounts(mapf_op_calls);
      
      /// Logging
      /// Move this to separate method
      if (!(i%heartbeat_freq) && (i != 0)){
        improvements.push_back(costFasterIterationStart - costRes);
        costFunctionValues.push_back(costRes);
        pathRes.setImprovements(improvements);
        pathRes.setCostFunctionValues(costFunctionValues);
        costFasterIterationStart = costRes;
        if (!(i%log_freq)){
          if (twoRounds){
            CVRPLIB_LOG_INFO("Running with integrated MAPF");
          }
          /* pathRes.printOperatorStats(); */
          /* printf("-----------------\n"); */
          CVRPLIB_LOG_INFO("Stats for iteration " << i);
          MainType costDifference = costRes - costIterationStart;
          costIterationStart = costRes;
          time2 = clock1.getTimeInSeconds();
          CVRPLIB_LOG_INFO("Cost reduction after " << log_freq << " iterations: " << costDifference);
          CVRPLIB_LOG_INFO("The time to perform " << log_freq <<  " iterations was " << time2 - time1 << "s.");
          CVRPLIB_LOG_INFO("The best-so-far solution is:");
          pathRes.printSolution();
          CVRPLIB_LOG_INFO("There are " << maxIter-i << " iterations remaining.");
          time1 = time2;
        }
        CVRPLIB_LOG_INFO("Current lowest cost is: " << costRes << " and current iteration is " << i << "."); 
      } 

      ls::perturbatevtwo(perturbation_neighborhoods, pathAux, costPathAux, 10);

      ++i;
    }
  }
}
