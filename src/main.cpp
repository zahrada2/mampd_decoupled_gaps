/**
 * File:    main.cpp
 *
 * Date:    01/02/21
 * Author:  David Zahradka
 * E-mail:  david.zahradka@cvut.cz
 *
 */

#include "Solver.hpp"

#include "vrplib/vrplib.hpp"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

using namespace cvrplib;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

#define DEFAULT_PROBLEM "warehouse24_derby/map0_64"
#define DEFAULT_METHOD "simplified_vns"
#define DEFAULT_DIR_OUT "./results/"
#define DEFAULT_FILE_OUT "out"
#define DEFAULT_MAPF_FILE_OUT "mapf"
#define DEFAULT_MAPF_MAP "warehouse24_derby/map0_64"
#define DEFAULT_WAIT_TIME 0

#include <cmath>

static std::string strProblem;
static std::string strMethod;
static std::string strDirIn;
static std::string mapfStrDirIn;
static std::string mapfMap;
static std::string mapfSolverName;
static std::string strDirOut;
static std::string strFileOut;
static std::string strMapfFileOut;
static std::string strRandomIntFile;

std::vector<std::pair<int, int>> vertexLocations;

static bool setWeightsTo1;
static bool commonWaitingTimes;
static bool beVerbose;
static bool beQuiet;
static bool beSilent;
static bool solveCvrp;
static bool sequentialMapf;
static bool mapfLevelOps;
static bool vrpOnly;
static bool vrpdebug;
static bool noExportMapf;
static bool loadSolution;

static double doubleGoalCost;
static double doubleGoalTime;
static double doubleScale;
static unsigned numVehicles;
static unsigned numGoals;
static unsigned vehicleCapacity;
static unsigned vehicleCapacityOffset;
static unsigned numIter;
static unsigned goalWaitTime;
static unsigned pbsVer;

static int numRun;
static int specificRun;

/* static unsigned itersOuter; */
static unsigned itersInner;
static bool itersInnerFixed;
static unsigned operatorsSeq;
static unsigned shakersSeq;

static int shrinkInstanceToSize;
static bool initialGreedy;
static bool usePbs;
static bool useTtd;
static bool useFa;
static bool useTrueRandom;
static bool blockGoals;

imr::vrp::VRPLib vrp_lib;
imr::vrp::Problem instance;
std::vector<uint16_t> starting_depots;
std::vector<uint16_t> ending_depots;
std::vector<int> waitings;

#define DEFAULT_ITERS_OUTER (3000)
#define DEFAULT_ITERS_INNER (30)
#define DEFAULT_ITERS_INNER_FIXED (false)
#define DEFAULT_OPERATORS_SEQ (4)
#define DEFAULT_SHAKERS_SEQ (3)
#define DEFAULT_SHRINK_INSTANCE_TO_SIZE (-1)
#define DEFAULT_MAPF_SOLVER ("cardinal_sipp")

bool parseArgs(
        int argc,
        const char *const *argv
)
{
    bool ret = true;
    po::options_description desc("General options");
    desc.add_options()
            ("help,h", "produce this help message")
            ("verbose",
             po::bool_switch(&beVerbose)->default_value(false),
             "...")
            ("quiet",
             po::bool_switch(&beQuiet)->default_value(false),
             "...")
            ("silent",
             po::bool_switch(&beSilent)->default_value(false),
             "...")
            ("solve-cvrp",
             po::bool_switch(&solveCvrp)->default_value(true),
             "...")
            ("separate-mapf",
             po::bool_switch(&sequentialMapf)->default_value(true),
             "...")
            ("vrp-only",
             po::bool_switch(&vrpOnly)->default_value(false),
             "...")
            ("no-mapf-export",
             po::bool_switch(&noExportMapf)->default_value(false),
             "...")
            ("mapf-ops",
             po::bool_switch(&mapfLevelOps)->default_value(true),
             "...")
            ("vrpdebug",
             po::bool_switch(&vrpdebug)->default_value(false),
             "...")
            ("load-solution",
             po::bool_switch(&loadSolution)->default_value(false),
             "...")
            ("goal,g",
             po::value<double>(&doubleGoalCost)->default_value(-1.0),
             "...")
            ("time-limit,t",
             po::value<double>(&doubleGoalTime)->default_value(-1.0),
             "...")
            ("scale",
             po::value<double>(&doubleScale)->default_value(1.0),
             "...")
            ("dir-in",
             po::value<std::string>(&strDirIn)->default_value(DEFAULT_DIR_IN),
             "...")
            ("mapf-dir-in",
             po::value<std::string>(&mapfStrDirIn)->default_value(DEFAULT_MAPF_DIR_IN),
             "...")
            ("mapf-map",
             po::value<std::string>(&mapfMap)->default_value(DEFAULT_MAPF_MAP),
             "...")
            ("mapf-solver",
             po::value<std::string>(&mapfSolverName)->default_value(DEFAULT_MAPF_SOLVER),
             "...")
            ("random-int-file",
             po::value<std::string>(&strRandomIntFile)->default_value(DEFAULT_RANDOM_INT_FILE),
             "...")
            ("problem,p",
             po::value<std::string>(&strProblem)->default_value(DEFAULT_PROBLEM),
             "...")
            ("shrink-instance-to-size",
             po::value<int>(&shrinkInstanceToSize)->default_value(DEFAULT_SHRINK_INSTANCE_TO_SIZE),
             "...")
            ("initial-greedy",
             po::bool_switch(&initialGreedy)->default_value(false),
             "...")
            ("method,m",
             po::value<std::string>(&strMethod)->default_value(DEFAULT_METHOD),
             "...")
            ("pbs-version",
             po::value<unsigned>(&pbsVer)->default_value(0),
             "...")
            ("itersInner,J",
             po::value<unsigned>(&itersInner)->default_value(DEFAULT_ITERS_INNER),
             "...")
            ("itersInnerFixed,4",
             po::value<bool>(&itersInnerFixed)->default_value(DEFAULT_ITERS_INNER_FIXED),
             "...")
            ("operators,O",
             po::value<unsigned>(&operatorsSeq)->default_value(DEFAULT_OPERATORS_SEQ),
             "...")
            ("shakers,S",
             po::value<unsigned>(&shakersSeq)->default_value(DEFAULT_SHAKERS_SEQ),
             "...")
            ("dir-out",
             po::value<std::string>(&strDirOut)->default_value(DEFAULT_DIR_OUT),
             "...")
            ("file-out",
             po::value<std::string>(&strFileOut)->default_value(DEFAULT_FILE_OUT),
             "...")
            ("mapf-out",
             po::value<std::string>(&strMapfFileOut)->default_value(DEFAULT_MAPF_FILE_OUT),
             "...")
            ("vehicles",
             po::value<unsigned>(&numVehicles)->default_value(5),
             "...")
            ("goals",
             po::value<unsigned>(&numGoals)->default_value(25),
             "...")
            ("capacity",
             po::value<unsigned>(&vehicleCapacity)->default_value(6),
             "...")
            ("capacity-offset",
             po::value<unsigned>(&vehicleCapacityOffset)->default_value(0),
             "...")
            ("set-weights-to-1,1",
             po::bool_switch(&setWeightsTo1)->default_value(false),
             "...")
            ("common-wt",
             po::bool_switch(&commonWaitingTimes)->default_value(false),
             "...")
            ("iters,I",
             po::value<unsigned>(&numIter)->default_value(3000),
             "...")
            ("num-run",
             po::value<int>(&numRun)->default_value(1),
             "...")
            ("specific-run",
             po::value<int>(&specificRun)->default_value(0),
             "...")
            ("pbs",
             po::bool_switch(&usePbs)->default_value(false),
             "...")
            ("ttd",
             po::bool_switch(&useTtd)->default_value(false),
             "...")
            ("block-goals",
             po::bool_switch(&blockGoals)->default_value(false),
             "...")
            ("fa",
             po::bool_switch(&useFa)->default_value(false),
             "...")
            ("true_random",
             po::bool_switch(&useTrueRandom)->default_value(false),
             "...")
            ("wait-time",
             po::value<unsigned>(&goalWaitTime)->default_value(DEFAULT_WAIT_TIME),
             "...");
    try {
        /// Continue to parse arguments.
        po::options_description cmdline_options;
        cmdline_options.add(desc);
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
        po::notify(vm);
        if (vm.count("help")) {
            CVRPLIB_LOG_INFO(cmdline_options);
            ret = false;
        }
    } catch (std::exception &e) {
        CVRPLIB_LOG_ERROR("Error in parsing arguments: " << e.what() << ".");
        ret = false;
    }
    return ret;
}

std::vector<double> getClosestDepotDistances(std::vector<uint16_t> ending_depots, std::vector<std::vector<double>> distances, std::vector<std::pair<int, int>> locations){

  int size = distances.size();
  std::vector<double> closestDepotDistances(size, -1);
  double min_distance;
  for (uint16_t i = 0; i < distances.size(); ++i){
    min_distance = INT_MAX;

    for (uint16_t j = 0; j < ending_depots.size(); ++j){
      int dist = distances[i][ending_depots[j]];// < min_distance;
      if (dist < min_distance){
        min_distance = dist;
      }
      closestDepotDistances[i] = min_distance;
    }
  }

  return closestDepotDistances;
}

void initLoggingSeverityLevel(int rng_seed)
{
    if (beSilent) {
        /// Silent ~ FATAL, ERROR
        initUtils(log::error, rng_seed, useTrueRandom);
    } else if (beQuiet) {
        /// Quet ~ FATAL, ERROR, WARNING
        initUtils(log::warning, rng_seed, useTrueRandom);
    } else if (beVerbose) {
        /// Verbose ~ FATAL, ERROR, WARNING, INFO, DEBUG, TRACE
        initUtils(log::trace, rng_seed, useTrueRandom);
    } else {
        /// Default ~ FATAL, ERROR, WARNING, INFO
        initUtils(log::info, rng_seed, useTrueRandom);
    }
}


std::vector<std::vector<double>> loadInstance(const std::string &filename, bool specified_num_goals){
    /// Check if the file exists.
    if (vrp_lib.loadProblem(filename)){
      instance = vrp_lib.getProblem();
      // load nodes
      auto lst = instance.goals;

      starting_depots = instance.starting_depots;
      ending_depots = instance.ending_depots;

      int num_goals_with_depots = numGoals + starting_depots.size()*2;
      if (specified_num_goals && (num_goals_with_depots <= lst.size())){
        lst.erase(lst.begin()+num_goals_with_depots, lst.end());
      }

      auto distances = std::vector<std::vector<double>>(lst.size(), std::vector<double>(lst.size()));
      int i = 0;

      for (auto &l1 : lst){
        int j = 0;
        for (auto &l2 : lst) distances[i][j++] = std::round(l1.distanceSqrt(l2));
        vertexLocations.push_back(l1.getLocation());
        ++i;
        /* CVRPLIB_LOG_INFO("pairtest:" << l1.getLocation().first << " " << l1.getLocation().second); */

      }

      int num_vertices = vertexLocations.size();
      int num_goals = num_vertices - starting_depots.size() - ending_depots.size();

      if (instance.waiting_times.size() != num_goals){
        CVRPLIB_LOG_WARNING("Some vertices have unspecified waiting time, filling with default ...");
        for (uint16_t i = 0; i < num_vertices; ++i){
          if (i < (starting_depots.size() + ending_depots.size() - 1)){
            waitings.push_back(0);
          } else {
            waitings.push_back(goalWaitTime);
          }
        }
      } else {
        for (uint16_t i = 0; i < starting_depots.size(); ++i){
          waitings.push_back(0);
        }

        for (uint16_t i = 0; i < ending_depots.size(); ++i){
          waitings.push_back(0);
        }

        waitings.insert(waitings.end(),instance.waiting_times.begin(), instance.waiting_times.end());
      }

      return distances;
    } else {
        CVRPLIB_LOG_FATAL("File " << filename << " cannot be opened, loaded or found.");
        exit(CVRPLIB_EXIT_FAILURE);
    }
    /// Use vrplib to load the file.
}


void loadRandomWeights(
        const std::string &filename,
        DoubleVector &weights
)
{
    std::fstream wf(filename, std::ios_base::in);
    unsigned i = 0;
    CVRPLIB_LOG_DEBUG("Loading random weights.");
    while (i < weights.size()) {
        std::string line;
        getline(wf, line);
        if (line[0] != '#') {
            double num = std::stod(line);
            CVRPLIB_LOG_DEBUG("weights[" << i << "] = " << num << " ");
            weights[i++] = num;
        }
    }
    CVRPLIB_LOG_INFO("Random weights from " << filename << " were loaded.");
}

void savePtreeAsJson(
        const std::string &dir,
        const std::string &file,
        boost::property_tree::ptree &ptree
)
{
    if (strDirOut != ".") {
        fs::create_directories(dir);
    }
    auto filePath = strDirOut + "/" + file + ".json";
    std::ofstream outputStream(filePath.c_str());
    boost::property_tree::write_json(outputStream, ptree);
}

int main(
        int argc,
        const char *const *argv
)
{

    auto clockTotal = SimpleClock();

    /// Pre-init logging.
    initLogging();

    /// Log start.
    std::stringstream call{};
    call << argv[0];
    for (int i = 1; i < argc; i++) call << " " << argv[i];
    CVRPLIB_LOG_INFO("Running: " << CVRPLIB_LOGSTYLE_COL_LIGHTBLUE << call.str() << CVRPLIB_LOGSTYLE_NORMAL << ".");

    bool specified_wait_time = false;
    bool specified_capacity = false;
    bool specified_num_vehicles = false;
    bool specified_num_goals = false;
    if (parseArgs(argc, argv)) {

        if (argc > 1){
          for (int i = 0; i < argc; ++i){
            std::string param = argv[i];
            if (param == "--wait-time"){
              std::cout << "Found wait time param, overriding ...\n";
              specified_wait_time = true;
            }
          }
          
          for (int i = 0; i < argc; ++i){
            std::string param = argv[i];
            if (param == "--capacity"){
              std::cout << "Found capacity param, overriding ...\n";
              specified_capacity = true;
            }
          }
          
          for (int i = 0; i < argc; ++i){
            std::string param = argv[i];
            if (param == "--vehicles"){
              std::cout << "Found num vehicles param, overriding ...\n";
              specified_num_vehicles= true;
            }
          }

          for (int i = 0; i < argc; ++i){
            std::string param = argv[i];
            if (param == "--goals"){
              std::cout << "Found num goals param, overriding ...\n";
              specified_num_goals = true;
            }
          }
        }

        
        std::string mapf_map = strDirIn + "/" + strProblem + ".map";

        boost::shared_ptr<MapData> map(new MapData(mapf_map));

        /// ============================================================================================================
        /// The main body of the program starts here.
        /// ============================================================================================================
        
        boost::property_tree::ptree combined_info{};
        for (uint16_t run = specificRun; run < std::max(numRun, specificRun+1); ++run){

          // This has to be here so that the randomizer is re-seeded
          /// Init logging.
          initLoggingSeverityLevel(run);


          /// Create solver.
          Solver solver{};

          /// Set method.
          if (solver.setMethod(strMethod)) {
              CVRPLIB_LOG_INFO("Method " << strMethod << " was set.");
          } else {
              CVRPLIB_LOG_WARNING("Could not set method " << strMethod << ".");
          }

          if (!vrpOnly){
            if (sequentialMapf){
              CVRPLIB_LOG_INFO("Running with sequential MAPF");
            } else {
              CVRPLIB_LOG_ERROR("Running with integrated MAPF");
            }
          } else if (vrpOnly){
            CVRPLIB_LOG_INFO("Running only VRP");
            sequentialMapf = true;
          }
          if (!noExportMapf){
            CVRPLIB_LOG_INFO("MAPF solution will be saved to " << strMapfFileOut);
          } else {
            CVRPLIB_LOG_INFO("MAPF solution WILL NOT be saved");
          }
          
          /// Set cost goal.
          if (doubleGoalCost > 0.0) {
              auto goal = static_cast<MainType>(doubleGoalCost);
              CVRPLIB_LOG_INFO("Setting cost goal: " << goal << ".");
              solver.goals.setGoalCost(goal);
          }

          /// Set time limits.
          if (doubleGoalTime > 0.0) {
              double timeLimit;
              timeLimit = doubleGoalTime;
              CVRPLIB_LOG_INFO("Setting time limit: " << timeLimit << ".");
              solver.goals.setGoalTime(timeLimit);
          }

          /// Set scale.
          solver.setScale(static_cast<long>(doubleScale));

          /// Load instance.
          std::string filename = strDirIn + "/" + strProblem + ".tsp";
          DoubleMatrix distances;
          auto problemName = fs::path(filename).stem().string();

          // Load instance file
          distances = loadInstance(filename, specified_num_goals);

          // Handle cases with no specified depots
          if (!(starting_depots.size() > 0)){
            for (uint16_t i = 0; i < numVehicles; ++i){
              starting_depots.push_back(i);
            }
          }

          if (!(ending_depots.size() > 0)){
            for (uint16_t i = 0; i < numVehicles; ++i){
              ending_depots.push_back(starting_depots[i]);
              CVRPLIB_LOG_INFO("start: " << starting_depots[i] << " end: " << ending_depots[i]);
            }
          }
        
          // Erase extra depots
          if (specified_num_vehicles){


          } else if (numVehicles != starting_depots.size() && starting_depots.size() > 0){
            // Override numVehicles if specified starting depots don't fit
            CVRPLIB_LOG_WARNING("NumVehicles automatic override! Set to the number of starting depots");
            numVehicles = starting_depots.size();
            // set waiting times for starting depots to 0
            for (int i = 0; i < starting_depots.size(); ++i){
              waitings[starting_depots[i]] = 0;
            }
            for (int i = 0; i < ending_depots.size(); ++i){
              waitings[ending_depots[i]] = 0;
            }
          }

          if (specified_wait_time){
            CVRPLIB_LOG_WARNING("Waiting time override! Set all to user-specified wait time");
            for (int i = numVehicles*2; i < waitings.size(); ++i){
              waitings[i] = goalWaitTime;
            }
          }



          /// TESTING PURPOSES
          for (uint16_t i = 0; i < starting_depots.size(); ++i){
            CVRPLIB_LOG_DEBUG("starting depot: " << starting_depots[i]  << " waiting time: " << waitings[i]);
          }
          
          for (uint16_t i = 0; i < ending_depots.size(); ++i){
            CVRPLIB_LOG_DEBUG("ending depot: " << ending_depots[i]  << " waiting time: " << waitings[i+starting_depots.size()-1]);
          }

          for (uint16_t i = starting_depots.size() + ending_depots.size(); i < vertexLocations.size(); ++i){
            CVRPLIB_LOG_DEBUG("goal: " << i << " waiting time: " << waitings[i]);
          }

          map->setPOICoords(vertexLocations);

          auto n = static_cast<int>(distances.size());

          std::vector<unsigned> waiting_times(n, goalWaitTime);
          /// Shrink instance to given size.
          if (shrinkInstanceToSize != DEFAULT_SHRINK_INSTANCE_TO_SIZE) {
              if (n <= shrinkInstanceToSize || shrinkInstanceToSize <= 0) {
                  CVRPLIB_LOG_FATAL(
                          "Cannot shrink instance of size " << n << " to size " << shrinkInstanceToSize << ".");
                  return (CVRPLIB_EXIT_FAILURE);
              }
              distances = DoubleMatrix(distances.begin(), distances.begin() + shrinkInstanceToSize);
              for (auto &dVec : distances) {
                  dVec = DoubleVector(dVec.begin(), dVec.begin() + shrinkInstanceToSize);
              }
          }

          // Initialize and set MAPF solver
          boost::shared_ptr<MAPF_interface> mapf_solver;
          boost::shared_ptr<MAPF_interface> secondary_mapf_solver;
          PBSversion version = v0;
          if (pbsVer == 0){
            version = v0;
          }
          else if (pbsVer == 1){
            version = v1;
            CVRPLIB_LOG_INFO("Running with PBS version 1."); 
          } else if(pbsVer == 2){
            version = v2;
            CVRPLIB_LOG_INFO("Running with PBS version 2."); 
          } else {
            version = v1;
            CVRPLIB_LOG_INFO("Running with default PBS version 1."); 
          }

          if (usePbs){
            mapf_solver.reset(new MAPF_PBS(run, useTtd, blockGoals, version));
          } else {
            mapf_solver.reset(new MAPF_Prioritized(run, useTtd, blockGoals));
          }



          map->setWaitTime(waitings);


          // Load grid map into distances
          MapData::DistanceMatrix distance_matrix = map->getDistanceMatrix();
          /* distance_matrix.compute(); */
          for (uint16_t i = 0; i < n; ++i){
            for (uint16_t j = 0; j < n; ++j){
              distances[i][j] = distance_matrix.get_value(vertexLocations[i].second, vertexLocations[i].first, vertexLocations[j].second, vertexLocations[j].first); // this might be the correct one, the previous gives too high values of IDs
            }
          }

          // calculate closest depot distances, save to map data
          
          std::vector<double> closestDepotDistances = getClosestDepotDistances(ending_depots, distances, vertexLocations);
          map->setMinimalDepotDistances(closestDepotDistances);

          if (mapf_solver->setMap(map)){
            CVRPLIB_LOG_INFO("Map loaded.");
          } else {
            CVRPLIB_LOG_ERROR("Map was not set properly!");
          }
          solver.setMAPFSolver(mapf_solver);
          CVRPLIB_LOG_INFO("MAPF solver set");

          secondary_mapf_solver.reset(new MAPF_PBS(run));
          if (secondary_mapf_solver->setMap(map)){
            CVRPLIB_LOG_INFO("Secondary solver - map loaded.");
            solver.setSecondaryMAPFSolver(secondary_mapf_solver);
          } else {
            CVRPLIB_LOG_ERROR("Secondary solver - map not set properly!");
          }

          /// Set data.
          if (solveCvrp) {
            /// We are solving CVRP.
            /// TODO: replace hardcoded demands by demands loaded from file
            UnsignedVector demands;
            demands = UnsignedVector(n, 1);
            /* demands[0] = 0; */
            for (uint16_t depot = 0; depot < numVehicles; ++depot){
              demands[starting_depots[depot]] = 0;
              demands[ending_depots[depot]] = 0;
            }

            if (!specified_capacity){
              vehicleCapacity = numGoals/numVehicles + 1 + vehicleCapacityOffset;
              CVRPLIB_LOG_INFO("Using automatic vehicle capacity: " << vehicleCapacity);
            } else {
              CVRPLIB_LOG_INFO("Using user-specified vehicle capacity: " << vehicleCapacity);
            }



            std::vector<uint16_t> orders(n, 9999);
            /* std::vector<bool> assigned_vector; */

            std::vector<bool> order_cl(n, true);

            //assign all vertices to some order
            uint16_t order_id = 0;

            // set depots as unavailable to assign to an order
            for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){
              order_cl[starting_depots[vehicle]] = false;
              order_cl[ending_depots[vehicle]] = false;
            }

            in::CvrpStructures cvrp{distances, starting_depots, ending_depots, demands, numVehicles, vehicleCapacity, !sequentialMapf, mapfLevelOps, waitings, strProblem, run, loadSolution, useTtd, useFa, closestDepotDistances};
            solver.setData(cvrp);
          } 

          std::string problemType = (solveCvrp) ? "CVRP" : "Other problem";
          CVRPLIB_LOG_INFO("Solving  " << problemType << " ...");

          auto *results = solver.solve(numIter, itersInner, operatorsSeq, shakersSeq, itersInnerFixed, initialGreedy);



          bool mapf_solution_successful = false;

          if (results) {
              auto &path = solver.getPath();
              MainType mapfCost;
              MainType finalCost;
              if (sequentialMapf && !vrpOnly){
                MAPF_input mapfTask;
                mapfTask.sequences = path.getCvrpPath();

                mapfCost = mapf_solver->computeCost(mapfTask).cost;
                finalCost = mapfCost;
                mapf_solution_successful = mapf_solver->lastInstanceSolutionFound();

                CVRPLIB_LOG_INFO("Final cost as computed by MAPF: " << mapfCost);
              } else {
                finalCost = results->cost;
                mapf_solution_successful = (finalCost != 1000000000);
              }
              CVRPLIB_LOG_INFO("Finished!");


              std::vector<std::vector<int>> result_path = path.getCvrpPath();

              std::stringstream ss{};
              ss << "Solution:";
              /// Print path.
              int num_nodes = 0;
              for (unsigned vehicle = 0; vehicle < numVehicles; ++vehicle){
                ss.str("");
                ss << "Route " << vehicle << ": ";
                unsigned numVertices = result_path[vehicle].size();
                for (unsigned i = 0; i < numVertices - 1; ++i){
                  ss << result_path[vehicle][i] << " - ";
                  num_nodes++;
                }
                num_nodes++;
                ss << result_path[vehicle][numVertices-1];
                CVRPLIB_LOG_INFO(ss.str());
              }

              CVRPLIB_LOG_INFO("Final route has " << num_nodes << "nodes.");
              /// Print cost and time.
              CVRPLIB_LOG_INFO("Cost: " << results->cost << ".");
              CVRPLIB_LOG_INFO("Method finished in: " << results->time << " s.");

              MainType costVerification;
              if (vrpOnly){
                costVerification = path.computeCost();
              } else {
                MAPF_input mapfTask;
                mapfTask.sequences = path.getCvrpPath();
                costVerification = mapf_solver->computeCost(mapfTask).cost;
              }

              /* // Check computed cost and path */
              bool costVerified = (finalCost == costVerification);
              if (costVerified){
                CVRPLIB_LOG_INFO("The resulting cost is correct.");
              } else {
                CVRPLIB_LOG_ERROR("The computed cost failed verification!");
                CVRPLIB_LOG_ERROR("Computed: " << finalCost << " True cost: " << costVerification);
              }

              CVRPLIB_LOG_INFO("Best cost achieved on its own was: " << path.best_own_result);

              
              std::vector<std::vector<int>> pathCvrp = path.getCvrpPath();
              MAPF_input mapftask;
              mapftask.sequences = pathCvrp;
              MAPF_solution sol = mapf_solver->computeCost(mapftask);

              std::vector<int> goals_in_path(numVehicles, 0);
              for (uint16_t i = 0; i < numVehicles; ++i){
                goals_in_path[i] = path.getNumVertices(i) - 2;
              }

              int VRP_TTD = path.computeCostSelected(true);
              int VRP_Cost = path.computeCostSelected(false);
              
              double MAPF_TTD = -1;
              double MAPF_Cost = -1;


              MAPF_input mapfOutputAssignment;
              MAPF_solution mapfOutputSolutionStandard;
              /* bool ttd_helper = false; */

              boost::shared_ptr<MAPF_interface> auxiliary_mapf_solver;
              mapfOutputAssignment.sequences = path.getCvrpPath();
              /* auxiliary_mapf_solver.reset(new MAPF_PBS(run, false)); */
              if (usePbs){
                auxiliary_mapf_solver.reset(new MAPF_PBS(run, false, blockGoals));
              } else {
                auxiliary_mapf_solver.reset(new MAPF_Prioritized(run, false, blockGoals));
              }
              if (auxiliary_mapf_solver->setMap(map)){
                mapfOutputSolutionStandard = auxiliary_mapf_solver->computeCost(mapfOutputAssignment);
                MAPF_Cost = mapfOutputSolutionStandard.cost;
              } else {
                CVRPLIB_LOG_ERROR("Standard mapf solver - map not set properly!");
              }
              /* std::cout << MAPF_Cost << "\n"; */
              
              MAPF_solution mapfOutputSolutionTTD;
              mapfOutputAssignment.sequences = path.getCvrpPath();
              if (usePbs){
                auxiliary_mapf_solver.reset(new MAPF_PBS(run, true, blockGoals));
              } else {
                auxiliary_mapf_solver.reset(new MAPF_Prioritized(run, true, blockGoals));
              }
              if (auxiliary_mapf_solver->setMap(map)){
                mapfOutputSolutionTTD = auxiliary_mapf_solver->computeCost(mapfOutputAssignment);
                MAPF_TTD = mapfOutputSolutionTTD.cost;
              } else {
                CVRPLIB_LOG_ERROR("TTD mapf solver - map not set properly!");
              }
              
              int ag = mapf_solver->getAg(goals_in_path, sol);
              std::cout << ag << "\n";
              // apply discounts
              int verification_TTD = ag;
              for (uint16_t vehicle = 0; vehicle < numVehicles; ++vehicle){
                for (uint16_t subgoal = 0; subgoal < pathCvrp[vehicle].size(); ++subgoal){
                  verification_TTD = verification_TTD - closestDepotDistances[pathCvrp[vehicle][subgoal]];
                }
              }
              /// Print all info to json.
              boost::property_tree::ptree info{};
              info.put("Problem", problemType);
              info.put("Instance", strProblem);
              info.put("MAPF map", mapf_map);
              info.put("Seed", run);
              info.put("Size", path.n());
              if (useTtd){
                info.put("Criterion", "TTD");
              } else {
                info.put("Criterion", "Length");
              }
              if (useFa){
                info.put("Acceptance", "FA");
              } else {
                info.put("Acceptance", "BA");
              }
              info.put("NumGoals", numGoals);
              info.put("Number of nodes in path", num_nodes);
              info.put("Vehicles", numVehicles);
              info.put("Capacity", vehicleCapacity);
              info.put("BlockGoals", blockGoals);
              info.put("WaitTime", goalWaitTime);
              info.put("MethodStr", strMethod);
              info.put("MethodName", solver.getMethodName());
              info.put("Iterations", numIter);
              info.put("MAPFSolved", mapf_solution_successful);

              info.put("VRPStandardCost", VRP_Cost);
              info.put("VRPTTD", VRP_TTD);
              info.put("MAPFStandardCost", MAPF_Cost);
              info.put("MAPFTTD", MAPF_TTD);
              info.put("Initial MAPF cost", path.previous_cost);
              if (sequentialMapf){
                info.put("VRPIterationCost", results->cost);
              } else {
                info.put("VRPIterationCost", path.getMinimumVrpCost());
              }
              if (vrpOnly){
                info.put("MAPFIterationOutput", -999);
              } else {
                info.put("MAPFIterationOutput", finalCost);
              }

              if (path.best_own_result != INT_MAX){
                info.put("BestOwnResult", path.best_own_result);
              }
              if (path.loaded_cost != -1){
                info.put("ExternalSolverCost", path.loaded_cost);
              }
              info.put("VerifyTTD", verification_TTD);
              std::string pbs_filename = "/home/owl/imr_workspace/cvrp-lib-vns/src/data/PBS/";
              pbs_filename = pbs_filename + path.instance().problem_name + "_iterations" + std::to_string(numIter) + "_run" + std::to_string(run) + "_instance_log.xml";
              info.put("PBSFile", pbs_filename);
              info.put("PrecomputedSolutionInput", loadSolution);
              info.put("UsePBS", usePbs);
              info.put("DeterministicRandomization", !useTrueRandom);
              info.put("RandomizationSeed", run);
              info.put("Iteration Found", path.getIterationStamp());
              /// False if does not match
              info.put("CostValid", costVerified);
              info.put("TimeExactS", results->time);
              /// Save operator counts
              info.put("VRP Exchange calls", path.getVrpOperatorCount(0));
              info.put("VRP Relocate calls", path.getVrpOperatorCount(1));
              info.put("VRP 2opt calls", path.getVrpOperatorCount(2));
              info.put("VRP 3opt calls", path.getVrpOperatorCount(3));
              info.put("VRP Doublebridge calls", path.getVrpOperatorCount(4));
              /// Save MAPF operator counts
              info.put("MAPF Exchange calls", path.getMapfOperatorCount(0));
              info.put("MAPF Relocate calls", path.getMapfOperatorCount(1));
              info.put("MAPF 2opt calls", path.getMapfOperatorCount(2));
              info.put("MAPF 3opt calls", path.getMapfOperatorCount(3));
              info.put("MAPF Doublebridge calls", path.getMapfOperatorCount(4));

              /* /// Save VRP operator times */
              info.put("VRP Exchange avg time", path.getVrpOpTime(0)/path.getVrpOperatorCount(0));
              info.put("VRP Relocate avg time", path.getVrpOpTime(1)/path.getVrpOperatorCount(1));
              info.put("VRP 2opt avg time", path.getVrpOpTime(2)/path.getVrpOperatorCount(2));
              info.put("VRP 3opt avg time", path.getVrpOpTime(3)/path.getVrpOperatorCount(3));
              if (path.getVrpOperatorCount(4) != 0){
                info.put("VRP Doublebridge avg time", path.getVrpOpTime(4)/path.getVrpOperatorCount(4));
              }
              /// Save MAPF operator times
              info.put("MAPF Exchange avg time", path.getMapfOpTime(0)/path.getMapfOperatorCount(0));
              info.put("MAPF Relocate avg time", path.getMapfOpTime(1)/path.getMapfOperatorCount(1));
              info.put("MAPF 2opt avg time", path.getMapfOpTime(2)/path.getMapfOperatorCount(2));
              info.put("MAPF 3opt avg time", path.getMapfOpTime(3)/path.getMapfOperatorCount(3));
              if (path.getMapfOperatorCount(4) != 0){
                info.put("MAPF Doublebridge avg time", path.getMapfOpTime(4)/path.getMapfOperatorCount(4));
              }
              info.put("MAPF Exchange cumul time", path.getMapfOpTime(0));
              info.put("MAPF Relocate cumul time", path.getMapfOpTime(1));
              info.put("MAPF 2opt cumul time", path.getMapfOpTime(2));
              info.put("MAPF 3opt cumul time", path.getMapfOpTime(3));
              if (path.getMapfOperatorCount(4) != 0){
                info.put("MAPF Doublebridge avg time", path.getMapfOpTime(4)/path.getMapfOperatorCount(4));
              }

              
              boost::property_tree::ptree routes;
              boost::property_tree::ptree route;
              boost::property_tree::ptree vertex;
              std::string route_title = "";
              for (uint16_t path_id = 0; path_id < numVehicles; ++path_id){
                route.clear();
                for (uint16_t vertex_id = 0; vertex_id < path.getNumVertices(path_id); ++vertex_id){
                  vertex.put("", path.getCvrpPath()[path_id][vertex_id]);
                  route.push_back(std::make_pair("", vertex));
                }
                /* route_title = "Route " + std::to_string(path_id+1); */
                routes.add_child(std::to_string(path_id), route);
              }
              info.add_child("Agent routes", routes);

              boost::property_tree::ptree improvs{};
              boost::property_tree::ptree value{};
              std::vector<long> loaded_improvements = path.getImprovements();
              for (uint16_t iteration_index = 0; iteration_index < loaded_improvements.size(); ++iteration_index){
                value.put("", loaded_improvements[iteration_index]);
                improvs.push_back(std::make_pair("", value));
              }
              info.add_child("Improvements", improvs);


              boost::property_tree::ptree hist{};
              /* boost::property_tree::ptree value{}; */
              std::vector<uint16_t> loaded_hist = path.getHistogram();
              for (uint16_t iteration_index = 0; iteration_index < loaded_hist.size(); ++iteration_index){
                value.put("", loaded_hist[iteration_index]);
                hist.push_back(std::make_pair("", value));
              }
              info.add_child("Histogram", hist);

              // export cfv = cost function values
              boost::property_tree::ptree cfvs{};
              boost::property_tree::ptree cfv_value{};
              std::vector<long> loaded_cfv = path.getCostFunctionValues();
              for (uint16_t iteration_index = 0; iteration_index < loaded_cfv.size(); ++iteration_index){
                cfv_value.put("", loaded_cfv[iteration_index]);
                cfvs.push_back(std::make_pair("", cfv_value));
              }
              info.add_child("Cost function values", cfvs);

              ///TODO: EXPORT PATH
              std::string run_id = "run_" + std::to_string(run);

              std::string json_filename = strFileOut + "_" + run_id;
              std::cout << "FILE OUT: " <<json_filename << "\n";
              savePtreeAsJson(strDirOut, json_filename, info);



              /// Export mapf solution to xml
              if (!noExportMapf && mapf_solution_successful){
                MAPF_input mapfTaskOut;
                mapfTaskOut.sequences = path.getCvrpPath();
                std::string strMapfFileOutIR = "./" + strDirOut + "/" + strMapfFileOut  + "_" + run_id + "_mapfir.txt";
                std::string strMapfFileOutMAMPDIR = "./" + strDirOut + "/" + strMapfFileOut  + "_" + run_id + ".mampd";
                std::string strMapfFileOutIRIteration = "./" + strDirOut + "/" + strMapfFileOut  + "_" + run_id + "from_iterations_mapfir.txt";
                std::string strMapfFileOutRun = "./" + strDirOut + "/" + strMapfFileOut + "_" + run_id;
                std::string strMapfFileOutRunIteration = "./" + strDirOut + "/" + strMapfFileOut + "_from_iterations" + run_id;
                std::string strInstanceOut = "./" + strDirOut + "/" + strMapfFileOut  + "_" + run_id + "_instance.xml";

                strMapfFileOutRun = strMapfFileOutRun + "_solution.xml";
                strMapfFileOutRunIteration = strMapfFileOutRunIteration + "_solution.xml";
                mapf_solver->saveSolutionIntoXML(strMapfFileOutRun, sol);

                MAPF_input instance_input;
                instance_input.sequences = path.getCvrpPath();
                MapData *map_export = map.get();
                mapf_solver->saveInstanceIntoXML(strInstanceOut, instance_input, *map_export);


                mapf_solver->saveSolutionMapfIR(strMapfFileOutIR, sol, starting_depots, ending_depots, numVehicles, mapfMap, vertexLocations, path.getMinimumVrpCost(), finalCost, strMethod);
                
        /* void saveSolutionMampdIR(std::string fileName, MAPF_solution solution, int numVehicles, std::string map_name, std::string instance_name, std::vector<std::pair<int,int>> vertex_locations, double vrp_cost, std::vector<std::vector<int>> assignments, MAPF_input mapf_task) override; */
                /* mapf_solver->saveSolutionMampdIR(strMapfFileOutMAMPDIR, sol, numVehicles, mapfMap, problemName, vertexLocations, path.getMinimumVrpCost(), finalCost, strMethod); */
                mapf_solver->saveSolutionMampdIR(strMapfFileOutMAMPDIR, sol, numVehicles, mapfMap, problemName, vertexLocations, path.getMinimumVrpCost(), path.getCvrpPath(),  instance_input);

                mapf_solver->saveSolutionIntoXML(strMapfFileOutRunIteration, path.best_sol);

                mapf_solver->saveSolutionMapfIR(strMapfFileOutIRIteration, path.best_sol, starting_depots, ending_depots, numVehicles, mapfMap, vertexLocations, path.getMinimumVrpCost(), finalCost, strMethod);

              }

          } else {
              CVRPLIB_LOG_ERROR("FAILURE!");
          }


          /// ============================================================================================================
          /// The main body of the program ends here.
          /// ============================================================================================================

        }
      /// Log end.
      CVRPLIB_LOG_INFO("Finished: " << CVRPLIB_LOGSTYLE_COL_LIGHTGREEN << call.str() <<
                                    CVRPLIB_LOGSTYLE_NORMAL << " in " << clockTotal.getTimeInSeconds() << " s.");
      map->getDistanceMatrix().freeData();
    }

    return (CVRPLIB_EXIT_SUCCESS);
}
