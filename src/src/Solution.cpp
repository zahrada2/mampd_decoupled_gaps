/**
 * File:    Solution.cpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "Solution.hpp"

#include <limits>

using namespace cvrplib;
using namespace cvrplib::solutions;

/// --------------------------------------------------------------------------------------------------------------------
/// Getters / setters
/// --------------------------------------------------------------------------------------------------------------------

const uint16_t Solution::getNumVehicles() const
{
    return inst->numVehicles;
}

const std::string &Solution::getName() const
{
    return name;
}

void Solution::setHistogram(std::vector<uint16_t> histogram){
  _histogram = histogram;
}

std::vector<uint16_t> Solution::getHistogram() const{
  return _histogram;
}


void Solution::printSolution(){
  
    std::stringstream ss{};
    /// Print path.
    for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
      ss.str("");
      ss << "Route " << vehicle << ": ";
      unsigned numVertices = pathCvrp[vehicle].size();
      for (unsigned i = 0; i < numVertices - 1; ++i){
        ss << pathCvrp[vehicle][i] << " - ";
      }
      ss << pathCvrp[vehicle][numVertices-1];
      CVRPLIB_LOG_INFO(ss.str());
    }
}

std::string Solution::toString(){
  std::string out;
  for (uint16_t i = 0; i < pathCvrp.size(); ++i){
    for (uint16_t j = 0; j < pathCvrp[i].size()-1; ++j){
      out += std::to_string(pathCvrp[i][j]) + " - ";
    }
    out += std::to_string(pathCvrp[i][pathCvrp[i].size()-1]) + "\n";
  }
  return out;
}

bool Solution::switchMapf(){
  useMapf = !useMapf; 
  return getMapfMode();
}

bool Solution::getMapfMode(){
  return useMapf;
}

void Solution::printOperatorStats(){
  CVRPLIB_LOG_INFO("OPERATOR STATS");
  CVRPLIB_LOG_INFO("Exchange: " << vrp_operator_calls[0] << " relocate: " << vrp_operator_calls[1] << " 2opt: " << vrp_operator_calls[2] << " 3opt: " << vrp_operator_calls[3] << " doublebridge: " << vrp_operator_calls[4]);
}

const unsigned Solution::getVrpOperatorCount(unsigned operator_id) const{
  return vrp_operator_calls[operator_id];
}

const unsigned Solution::getMapfOperatorCount(unsigned operator_id) const{
  return mapf_operator_calls[operator_id];
}

const double Solution::getVrpOpTime(unsigned operator_id) const{
  return vrp_operator_times[operator_id];
}

const double Solution::getMapfOpTime(unsigned operator_id) const{
  return mapf_operator_times[operator_id];
}

const unsigned Solution::getIterationStamp() const{
  return iterationStamp;
}

const MainType Solution::getMinimumVrpCost() const{
  return minimum_vrp_cost;
}

const std::vector<long> Solution::getImprovements() const{
  return improvements;
}

const std::vector<long> Solution::getCostFunctionValues() const{
  return costFunctionValues;
}

void Solution::updateDemands(){
  for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    current_demands.insert(current_demands.begin()+vehicle, computeDemands(vehicle));
  }
  /* for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   std::cout << current_demands[vehicle] << "\n"; */
  /* } */
}

void Solution::resetDemands(){
  for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    current_demands[vehicle] = computeDemands(vehicle);
  }
  /* for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   std::cout << current_demands[vehicle] << "\n"; */
  /* } */
}

/* void Solution::setDemand(uint16_t vehicle, uint16_t ){ */
/*   for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
/*     current_demands.insert(current_demands.begin()+vehicle, computeDemands(vehicle)); */
/*   } */
/*   /1* for (unsigned vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ *1/ */
/*   /1*   std::cout << current_demands[vehicle] << "\n"; *1/ */
/*   /1* } *1/ */
/* } */

void Solution::setVrpTimes(std::vector<double> times){
  vrp_operator_times = times; 
}

void Solution::setMapfTimes(std::vector<double> times){
  mapf_operator_times = times; 
}

void Solution::setVrpOpCounts(std::vector<long> op_count){
  vrp_operator_calls = op_count;
}

void Solution::setMapfOpCounts(std::vector<long> op_count){
  mapf_operator_calls = op_count; 
}

void Solution::setIterationStamp(unsigned iteration_when_found){
  iterationStamp = iteration_when_found;
}

void Solution::setImprovements(std::vector<long> improvements_to_set){
  improvements = improvements_to_set;
}

void Solution::setCostFunctionValues(std::vector<long> cfv_to_set){
  costFunctionValues = cfv_to_set;
}

void Solution::setMinimumVrpCost(MainType minimum_cost){
  minimum_vrp_cost = minimum_cost;
}

const cvrplib::Instance &Solution::instance() const
{
    return *inst;
}

const unsigned &Solution::n() const
{
    return inst->n;
}

const MainType &Solution::sumWeights() const
{
    return inst->sumWeights;
}

Ver Solution::getVer(Pos pos) const
{
    return path[pos];
}

Ver Solution::getVerCvrp(uint16_t vehicle, Pos pos) const
{
    return pathCvrp[vehicle][pos];
}

Pos Solution::getPos(Ver ver) const
{
    return pathInv[ver];
}
            
const std::vector<std::vector<Ver>> Solution::getCvrpPath() const
{
  return pathCvrp;
}

void Solution::setCvrpPath(std::vector<std::vector<Ver>> new_path)
{
  pathCvrp = new_path;
}

void Solution::set(
        Ver ver,
        Pos pos
)
{
    path[pos] = ver;
    pathInv[ver] = pos;
}

void Solution::initializePathCvrp(uint16_t numVehicles)
{
    std::vector<Ver> empty_vector_ver;
    Ver placeholder_ver = 0;
    empty_vector_ver.push_back(placeholder_ver);
    std::vector<Pos> empty_vector_pos;
    uint16_t placeholder_vehicle = -1;
    Pos placeholder_pos = -1;
    empty_vector_pos.push_back(placeholder_vehicle);
    empty_vector_pos.push_back(placeholder_pos);
    for (uint16_t route = 0; route < numVehicles; ++route){
      pathCvrp.push_back(empty_vector_ver);
    } 
}

void Solution::setCvrp(
        Ver ver,
        Pos pos,
        uint16_t path_id
)
{
    pathCvrp[path_id][pos] = ver;
    /* for (uint16_t i = 1; i < pathCvrp[path_id].size(); ++i){ */
    /*   printf("- %d ", pathCvrp[path_id][pos]); */
    /* } */
    /* printf("\n"); */
    /* pathInvCvrp[ver][0] = path_id; */
    /* pathInvCvrp[ver][1] = pos; */
}

void Solution::pushCvrp(
        uint16_t path_id,
        Ver ver
)
{
    /* pathCvrp[path_id].push_back(ver); */
    pathCvrp[path_id].insert((pathCvrp[path_id].end()-1), ver);
}

void Solution::pushBackCvrp(
        uint16_t path_id,
        Ver ver
)
{
    /* pathCvrp[path_id].push_back(ver); */
    pathCvrp[path_id].push_back(ver);
}
            
int Solution::getNumVertices(uint16_t vehicle) const
{
  return pathCvrp[vehicle].size();
}

void Solution::setInstance(const Instance &inst)
{
    Solution::inst = &inst;
    path = std::vector<Ver>(inst.n, -1);
    pathInv = std::vector<Pos>(inst.n, std::numeric_limits<Pos>::max());
    reinitStructures(inst.n);
}

/// --------------------------------------------------------------------------------------------------------------------
/// Utils
/// --------------------------------------------------------------------------------------------------------------------

void Solution::clear()
{
    for (unsigned i = 0; i < inst->n; ++i) {
        path[i] = -1;
        pathInv[i] = std::numeric_limits<Pos>::max();
    }
    reinitStructures(inst->n);
}

void Solution::clone(
        Solution &path,
        bool name
)
{
    Solution::inst = path.inst;
    Solution::path = path.path;
    Solution::pathInv = path.pathInv;
    if (name) {
        Solution::name = path.name;
    }
    reinitStructures(path.n());
    updateStructures();
}

std::string Solution::str() const
{
    std::stringstream ss{"[" + name + "] "};
    for (unsigned i = 0; i < inst->n; i++) {
        ss << path[i] << ((i == inst->n - 1) ? "" : " ");
    }
    return ss.str();
}

bool Solution::isValid() const
{
    return isValid(false);
}


bool Solution::isValid(bool checkInverse) const
{
    /// Check if the path is empty or contains invalid flag.
    if (path.empty() || std::count(path.begin(), path.end(), INVALID_PATH_FLAG)) {
        return false;
    }

    /// Check if vertex 0 is on the first position.
    if (path[0] != 0) {
        /// Return false if no.
        return false;
    }
    const auto &n = inst->n;
    /// Check if no vertex repeats in the sequence.
    std::vector<bool> bools(n, false);
    for (Pos i = 0; i < n; ++i) {
        if (bools[path[i]]) {
            /// Return false if yes.
            return false;
        }
        bools[path[i]] = true;
    }
    /// Check if all vertices are present.
    auto all = true;
    for (Pos i = 0; i < n; ++i) {
        all *= bools[i];
        if (!all) {
            /// Return false if no.
            return false;
        }
    }
    if (checkInverse) {
        /// Check if pathInv is inverse of path.
        for (Pos i = 0; i < n; ++i) {
            if (i != pathInv[path[i]]) {
                /// Return false if no.
                return false;
            }
        }
    }
    /// Return true.
    return true;
}

const std::vector<int> &Solution::getSequence() const
{
    return path;
}

bool Solution::setSequence(
        const std::vector<Ver> &path,
        bool checkIfValid
)
{

    if (path.size() != inst->n) {
        return false;
    }

    for (unsigned i = 0; i < path.size(); ++i) {
        set(path[i], i);
    }

    updateStructures();

    if (checkIfValid) {
        return isValid(true);
    } else {
        return true;
    }
}



/// --------------------------------------------------------------------------------------------------------
/// CVRP VND Operator - Neighborhood 1: sequence insert
/// --------------------------------------------------------------------------------------------------------

MainType Solution::trySequenceInsert(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos)
{
  MainType ret = 0;
  float pen = 0;
  float c = 0;
  for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    Pos prev_pos = 0;
    /// If the src_vehicle and tgt_vehicle are different (to remove one if from the for loop)
    if (src_vehicle != tgt_vehicle){
      for (uint16_t pos = 1; pos < pathCvrp[vehicle].size(); ++pos){
        /// If we are operating on the path we removed the vertices from
        if ((vehicle == src_vehicle) && (pos >= src_pos) && (pos <= src_pos + amount)){
          c += 0;
        /// If we are operating on the target path
        } else if ((vehicle == tgt_vehicle) && (pos == tgt_pos)){
          Pos internal_prev_pos = prev_pos;
          uint16_t internal_prev_vehicle = vehicle;
          /// Iterate through vertices from the sequence to insert
          for (uint16_t vertex_to_insert = 0; vertex_to_insert < amount; ++vertex_to_insert){
            c+= inst->rho[pathCvrp[src_vehicle][src_pos+vertex_to_insert]][pathCvrp[internal_prev_vehicle][internal_prev_pos]];
            internal_prev_pos = src_pos+vertex_to_insert;
            internal_prev_vehicle = src_vehicle;
          }
        /// Otherwise, just calculate distance as usual
        } else {
          c += inst->rho[pathCvrp[vehicle][pos]][pathCvrp[vehicle][prev_pos]];
        }

        prev_pos = pos;
      }
    }
  }
  ret = c + inst->alpha*pen;
  return ret;
}

void Solution::sequenceInsert(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos)
{
  /// TODO: optimize, beautify
  printf("src path before:\n");
  for (uint16_t i = 0; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path before:\n");
  for (uint16_t i = 0; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");

  /* uint16_t to_remove_src = pathCvrp[src_vehicle].size() - src_pos + amount; */
  /* uint16_t to_remove_tgt = pathCvrp[tgt_vehicle].size() - tgt_pos; */
  std::vector<Ver> src_temporary_removal;
  std::vector<Ver> tgt_temporary_removal;
  std::vector<Ver> sequence;
  /// Temporarily remove vertices from src path
  while (pathCvrp[src_vehicle].size() > src_pos + amount){
    src_temporary_removal.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Extract sequence to insert
  for (uint16_t i = 0; i < amount; ++i){
    sequence.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Return temporarily removed vertices to src path
  uint16_t num_of_removed_vertices = src_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[src_vehicle].push_back(src_temporary_removal.back());
    src_temporary_removal.pop_back();
  }


  /// Temporarily remove vertices from target vehicle
  while (pathCvrp[tgt_vehicle].size() > tgt_pos){
  /* for (uint16_t i = 0; i < to_remove_tgt; ++i){ */
    tgt_temporary_removal.push_back(pathCvrp[tgt_vehicle].back());
    pathCvrp[tgt_vehicle].pop_back();
  }
  /// Insert sequence
  for (uint16_t i = 0; i < amount; ++i){
    pathCvrp[tgt_vehicle].push_back(sequence.back());
    sequence.pop_back();
  }
  /// Return temporarily removed vertices to tgt path
  num_of_removed_vertices = tgt_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[tgt_vehicle].push_back(tgt_temporary_removal.back());
    tgt_temporary_removal.pop_back();
  }
  printf("src path after:\n");
  for (uint16_t i = 1; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path after:\n");
  for (uint16_t i = 1; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");
}

/// --------------------------------------------------------------------------------------------------------
/// CVRP VND Operator - Neighborhood 2: reverse sequence insert
/// --------------------------------------------------------------------------------------------------------

MainType Solution::trySequenceInsertReverse()
{
  MainType ret = 0;

  return ret;
}

void Solution::sequenceInsertReverse(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos)
{
  /// TODO: optimize, beautify
  printf("src path before:\n");
  for (uint16_t i = 0; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path before:\n");
  for (uint16_t i = 0; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");

  /* uint16_t to_remove_src = pathCvrp[src_vehicle].size() - src_pos + amount; */
  /* uint16_t to_remove_tgt = pathCvrp[tgt_vehicle].size() - tgt_pos; */
  std::vector<Ver> src_temporary_removal;
  std::vector<Ver> tgt_temporary_removal;
  std::vector<Ver> sequence;
  /// Temporarily remove vertices from src path
  while (pathCvrp[src_vehicle].size() > src_pos + amount){
    src_temporary_removal.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Extract sequence to insert
  for (uint16_t i = 0; i < amount; ++i){
    sequence.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Reverse the sequence to insert
  std::reverse(sequence.begin(), sequence.end());

  /// Return temporarily removed vertices to src path
  uint16_t num_of_removed_vertices = src_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[src_vehicle].push_back(src_temporary_removal.back());
    src_temporary_removal.pop_back();
  }


  /// Temporarily remove vertices from target vehicle
  while (pathCvrp[tgt_vehicle].size() > tgt_pos){
  /* for (uint16_t i = 0; i < to_remove_tgt; ++i){ */
    tgt_temporary_removal.push_back(pathCvrp[tgt_vehicle].back());
    pathCvrp[tgt_vehicle].pop_back();
  }

  /// Insert sequence
  for (uint16_t i = 0; i < amount; ++i){
    pathCvrp[tgt_vehicle].push_back(sequence.back());
    sequence.pop_back();
  }

  /// Return temporarily removed vertices to tgt path
  num_of_removed_vertices = tgt_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[tgt_vehicle].push_back(tgt_temporary_removal.back());
    tgt_temporary_removal.pop_back();
  }
  printf("src path after:\n");
  for (uint16_t i = 1; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path after:\n");
  for (uint16_t i = 1; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");
}

/// --------------------------------------------------------------------------------------------------------
/// CVRP VND Operator - Neighborhood 3: single customer (vertex) swap
/// --------------------------------------------------------------------------------------------------------

MainType Solution::tryVertexSwap()
{
  MainType ret = 0;

  return ret;
}

void Solution::vertexSwap(uint16_t src_vehicle, Pos src_pos, uint16_t tgt_vehicle, Pos tgt_pos)
{
  Ver tmp_ver;
  tmp_ver = pathCvrp[tgt_vehicle][tgt_pos];
  pathCvrp[tgt_vehicle][tgt_pos] = pathCvrp[src_vehicle][src_pos];
  pathCvrp[src_vehicle][src_pos] = tmp_ver;
}

/// --------------------------------------------------------------------------------------------------------
/// CVRP VND Operator - Neighborhood 4: sequence swap
/// --------------------------------------------------------------------------------------------------------

MainType Solution::trySequenceSwap()
{
  MainType ret = 0;

  return ret;
}

void Solution::sequenceSwap(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos)
{
  /// TODO: optimize, beautify
  printf("src path before:\n");
  for (uint16_t i = 0; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path before:\n");
  for (uint16_t i = 0; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");

  /* uint16_t to_remove_src = pathCvrp[src_vehicle].size() - src_pos + amount; */
  /* uint16_t to_remove_tgt = pathCvrp[tgt_vehicle].size() - tgt_pos; */
  std::vector<Ver> src_temporary_removal;
  std::vector<Ver> tgt_temporary_removal;
  std::vector<Ver> sequence;
  /// Temporarily remove vertices from src path
  while (pathCvrp[src_vehicle].size() > src_pos + amount){
    src_temporary_removal.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Extract sequence to insert
  for (uint16_t i = 0; i < amount; ++i){
    sequence.push_back(pathCvrp[src_vehicle].back());
    pathCvrp[src_vehicle].pop_back();
  }

  /// Return temporarily removed vertices to src path
  uint16_t num_of_removed_vertices = src_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[src_vehicle].push_back(src_temporary_removal.back());
    src_temporary_removal.pop_back();
  }


  /// Temporarily remove vertices from target vehicle
  while (pathCvrp[tgt_vehicle].size() > tgt_pos){
  /* for (uint16_t i = 0; i < to_remove_tgt; ++i){ */
    tgt_temporary_removal.push_back(pathCvrp[tgt_vehicle].back());
    pathCvrp[tgt_vehicle].pop_back();
  }
  /// Insert sequence
  for (uint16_t i = 0; i < amount; ++i){
    pathCvrp[tgt_vehicle].push_back(sequence.back());
    sequence.pop_back();
  }
  /// Return temporarily removed vertices to tgt path
  num_of_removed_vertices = tgt_temporary_removal.size();
  for (uint16_t i = 0; i < num_of_removed_vertices; ++i){
    pathCvrp[tgt_vehicle].push_back(tgt_temporary_removal.back());
    tgt_temporary_removal.pop_back();
  }
  printf("src path after:\n");
  for (uint16_t i = 1; i < pathCvrp[src_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[src_vehicle][i]);
  }
  printf("\n");
  printf("tgt path after:\n");
  for (uint16_t i = 1; i < pathCvrp[tgt_vehicle].size(); ++i){
    printf(" - %d", pathCvrp[tgt_vehicle][i]);
  }
  printf("\n");
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator CVRP - Exchange
/// --------------------------------------------------------------------------------------------------------------------



///TODO: add consideration of feasibility violation (= calculate penalty for violating capacity restrictions)

MainType Solution::tryExchange(uint16_t vehicle1,
        uint16_t vehicle2,
        Pos i,
        Pos j
)
{
    MainType ret = -1;
    MainType cost = -1;
    /* if (!isExchangeFeasible(vehicle1, vehicle2, i, j)){ */
    /*   ret = INT_MAX; */
    /*   return ret; */
    /* } else { */
    /*   /1* std::cout << "YEAH BOII"; *1/ */
    /*   /1* sleep(1); *1/ */
    /* } */

    /* if (inst->mapfLevelOps && useMapf){ */
    /*   mapf_operator_calls[0] += 1; */
    /*   /// TODO: Implement call to computeExchangeCost here */
    /*   /// vehicle1 is the source agent, vehicle2 the target agent */
    /*   /// i is the vertex in vehicle1 to be exhanged for vertex j from vehicle2 (and vice versa) */
    /*   /// Do not forget to flip the boolean exchangeInMapf to true */
 
    /*   inst->mapfSolver->computeExchange(vehicle1, vehicle2, i, j); */

    /*   if (ret != -1){ */
    /*     return ret; */
    /*   } else { */
    /*     CVRPLIB_LOG_ERROR("MAPF exchange returned invalid cost."); */
    /*     return INT_MAX; */
    /*   } */
    /* } else if (useMapf){ */
    if (useMapf) {
      /* mapf_operator_calls[0] += 1; */
      MAPF_input mapfTask;
      mapfTask.sequences = pathCvrp;
      mapfTask.sequences[vehicle1][i] = pathCvrp[vehicle2][j];
      mapfTask.sequences[vehicle2][j] = pathCvrp[vehicle1][i];
      ret = inst->mapfSolver->computeCost(mapfTask).cost;
      if (ret != -1){
        return ret;
      } else {
        CVRPLIB_LOG_ERROR("MAPF returned invalid cost.");
        return INT_MAX;
      }
    } else if (!useMapf && inst->useTtd){
      Ver vertex_1 = pathCvrp[vehicle1][i];
      Ver vertex_2 = pathCvrp[vehicle2][j];
      Ver vertex_v1_prev = pathCvrp[vehicle1][i-1];
      Ver vertex_v1_next = pathCvrp[vehicle1][i+1];
      Ver vertex_v2_prev = pathCvrp[vehicle2][j-1];
      Ver vertex_v2_next = pathCvrp[vehicle2][j+1];
      cost = current_cost;
      int delta_l_1 = 0; 
      int delta_l_2 = 0; 

      /// Remove edges from the two exchanged vertices
      delta_l_1 += -(inst->rho[vertex_v1_prev][vertex_1]);
      delta_l_1 += -(inst->rho[vertex_1][vertex_v1_next]);
      delta_l_2 += -(inst->rho[vertex_v2_prev][vertex_2]);
      delta_l_2 += -(inst->rho[vertex_2][vertex_v2_next]);
 
      /// Reconnect in the new routes
      delta_l_1 += inst->rho[vertex_v1_prev][vertex_2];
      delta_l_1 += inst->rho[vertex_2][vertex_v1_next];
      delta_l_2 += inst->rho[vertex_v2_prev][vertex_1];
      delta_l_2 += inst->rho[vertex_1][vertex_v2_next];

      /// Remove edges from the two exchanged vertices
      /* delta_l_1 += -(inst->rho[pathCvrp[vehicle1][i-1]][pathCvrp[vehicle1][i]]); */
      /* delta_l_1 += -(inst->rho[pathCvrp[vehicle1][i]][pathCvrp[vehicle1][i+1]]); */
      /* delta_l_2 += -(inst->rho[pathCvrp[vehicle2][j-1]][pathCvrp[vehicle2][j]]); */
      /* delta_l_2 += -(inst->rho[pathCvrp[vehicle2][j]][pathCvrp[vehicle2][j+1]]); */
 
      /* /// Reconnect in the new routes */
      /* delta_l_1 += inst->rho[pathCvrp[vehicle1][i-1]][pathCvrp[vehicle2][j]]; */
      /* delta_l_2 += inst->rho[pathCvrp[vehicle2][j]][pathCvrp[vehicle1][i+1]]; */
      /* delta_l += inst->rho[pathCvrp[vehicle2][j-1]][pathCvrp[vehicle1][i]]; */
      /* delta_l += inst->rho[pathCvrp[vehicle1][i]][pathCvrp[vehicle2][j+1]]; */

      delta_l_1 = delta_l_1*(pathCvrp[vehicle1].size()-2);
      delta_l_2 = delta_l_2*(pathCvrp[vehicle2].size()-2);
      cost = cost + delta_l_1 + delta_l_2;

      /* CVRPLIB_LOG_INFO("cost: " << cost); */
      /* int test_cost = computeTTD(); */
      /* CVRPLIB_LOG_INFO("cost verification:" << test_cost); */

      return cost;

      //TODO
      /* cost = computeTTD(); */
      /* return cost; */

    } else if (!useMapf){
      /* operator_calls[0] += 1; */
      Ver vertex_1 = pathCvrp[vehicle1][i];
      Ver vertex_2 = pathCvrp[vehicle2][j];
      Ver vertex_v1_prev = pathCvrp[vehicle1][i-1];
      Ver vertex_v1_next = pathCvrp[vehicle1][i+1];
      Ver vertex_v2_prev = pathCvrp[vehicle2][j-1];
      Ver vertex_v2_next = pathCvrp[vehicle2][j+1];
      cost = current_cost;

      /// Remove edges from the two exchanged vertices
      cost += -(inst->rho[vertex_v1_prev][vertex_1]);
      cost += -(inst->rho[vertex_1][vertex_v1_next]);
      cost += -(inst->rho[vertex_v2_prev][vertex_2]);
      cost += -(inst->rho[vertex_2][vertex_v2_next]);
 
      /// Reconnect in the new routes
      cost += inst->rho[vertex_v1_prev][vertex_2];
      cost += inst->rho[vertex_2][vertex_v1_next];
      cost += inst->rho[vertex_v2_prev][vertex_1];
      cost += inst->rho[vertex_1][vertex_v2_next];

      /// Fix bug when exchanging vtx2=vtx1+1 from the same path
      /* cost += (vertex_2 == vertex_v1_next) * inst->rho[vertex_v2_prev][vertex_2]; */

      /// Remove edges from the two exchanged vertices
      /* cost += -(inst->rho[pathCvrp[vehicle1][i-1]][pathCvrp[vehicle1][i]]); */
      /* cost += -(inst->rho[pathCvrp[vehicle1][i]][pathCvrp[vehicle1][i+1]]); */
      /* cost += -(inst->rho[pathCvrp[vehicle2][j-1]][pathCvrp[vehicle2][j]]); */
      /* cost += -(inst->rho[pathCvrp[vehicle2][j]][pathCvrp[vehicle2][j+1]]); */
 
      /* /// Reconnect in the new routes */
      /* cost += inst->rho[pathCvrp[vehicle1][i-1]][pathCvrp[vehicle2][j]]; */
      /* cost += inst->rho[pathCvrp[vehicle2][j]][pathCvrp[vehicle1][i+1]]; */
      /* cost += inst->rho[pathCvrp[vehicle2][j-1]][pathCvrp[vehicle1][i]]; */
      /* cost += inst->rho[pathCvrp[vehicle1][i]][pathCvrp[vehicle2][j+1]]; */

      return cost;
    } else {
      CVRPLIB_LOG_ERROR("useMapf flag has invalid value!");
      return INT_MAX;
    }
}


void Solution::exchange(uint16_t vehicle1,
        uint16_t vehicle2,
        Pos i,
        Pos j
)
{
    /* std::cout << "exchanging " << pathCvrp[vehicle1][i] << " from position " << i << " with " << pathCvrp[vehicle2][j] <<  " from pos " << j << "\n"; */
    /* sleep(1); */
    Ver temp = pathCvrp[vehicle1][i];
    pathCvrp[vehicle1][i] = pathCvrp[vehicle2][j];
    pathCvrp[vehicle2][j] = temp;

    /// check for feasibility?


}


/// --------------------------------------------------------------------------------------------------------------------
/// Operator CVRP - Relocate
/// --------------------------------------------------------------------------------------------------------------------

MainType Solution::tryRelocate(uint16_t src_vehicle,
        uint16_t tgt_vehicle,
        Pos i,
        Pos j)
{
    MainType ret = -1;
    MainType cost = -1;

    if (current_demands[tgt_vehicle] == inst->vehicleCapacity){
      ret = INT_MAX;
      return ret;
    }
    if (current_demands[tgt_vehicle] > inst->vehicleCapacity){
      CVRPLIB_LOG_WARNING("Capacity overreached!");
      ret = INT_MAX;
      return ret;
    }
    /* if (!isRelocateFeasible(src_vehicle, tgt_vehicle, i)){ */
    /*   ret = INT_MAX; */
    /*   return ret; */
    /* } else { */
    /*   /1* std::cout << "YEAH BOII"; *1/ */
    /*   /1* sleep(1); *1/ */
    /* } */

    /* if (inst->mapfLevelOps && useMapf){ */

    /*   mapf_operator_calls[1] += 1; */

    /*   inst->mapfSolver->computeRelocateCost(pathCvrp[src_vehicle][i], tgt_vehicle, j, src_vehicle, i); */

    /*   if (ret != -1){ */
    /*     return ret; */
    /*   } else { */
    /*     CVRPLIB_LOG_ERROR("MAPF relocate returned invalid cost."); */
    /*     return INT_MAX; */
    /*   } */

    /* } else if (useMapf){ */
    if (useMapf) {
      /* mapf_operator_calls[1] += 1; */
      MAPF_input mapfTask;
      mapfTask.sequences = pathCvrp;
      mapfTask.sequences[src_vehicle].erase(mapfTask.sequences[src_vehicle].begin() + i);
      mapfTask.sequences[tgt_vehicle].insert(mapfTask.sequences[tgt_vehicle].begin() + j, pathCvrp[src_vehicle][i]);
      ret = inst->mapfSolver->computeCost(mapfTask).cost;
      if (ret != -1){
        return ret;
      } else {
        CVRPLIB_LOG_ERROR("MAPF returned invalid cost.");
        return INT_MAX;
      }
    } else if (!useMapf && inst->useTtd) {

      //TODO
      /* cost = computeTTD(); */
      /* return cost; */
      Ver vertex = pathCvrp[src_vehicle][i];
      Ver v1_prev = pathCvrp[src_vehicle][i-1];
      Ver v1_next = pathCvrp[src_vehicle][i+1];
      Ver v2_prev = pathCvrp[tgt_vehicle][j-1];
      Ver v2_next = pathCvrp[tgt_vehicle][j];
      
      cost = current_cost;
      int delta_l_1 = 0;
      int delta_l_2 = 0;
      
      /// Remove the costs of the edges of the vertex to be relocated

      delta_l_1 += -(inst->rho[v1_prev][vertex]);
      delta_l_1 += -(inst->rho[vertex][v1_next]);
      /// Reconnect them
      delta_l_1 += inst->rho[v1_prev][v1_next];

      /// Prepare space for new node
      delta_l_2 += -(inst->rho[v2_prev][v2_next]);
      /// Add the costs of two new edges
      delta_l_2 += inst->rho[v2_prev][vertex];
      delta_l_2 += inst->rho[vertex][v2_next];

      /// Get TTD
      delta_l_1 = delta_l_1*(pathCvrp[src_vehicle].size() -1 - 2) - computeSingleRouteCost(src_vehicle);
      delta_l_2 = delta_l_2*(pathCvrp[tgt_vehicle].size() +1 - 2) + computeSingleRouteCost(tgt_vehicle);
      
      ret = cost + delta_l_1 + delta_l_2;
      return ret;

    } else if (!useMapf){
      /* operator_calls[1] += 1; */

      Ver vertex = pathCvrp[src_vehicle][i];
      Ver v1_prev = pathCvrp[src_vehicle][i-1];
      Ver v1_next = pathCvrp[src_vehicle][i+1];
      Ver v2_prev = pathCvrp[tgt_vehicle][j-1];
      Ver v2_next = pathCvrp[tgt_vehicle][j];
      
      cost = current_cost;

      /// Remove the costs of the edges of the vertex to be relocated

      cost += -(inst->rho[v1_prev][vertex]);
      cost += -(inst->rho[vertex][v1_next]);
      /// Reconnect them
      cost += inst->rho[v1_prev][v1_next];

      /// Prepare space for new node
      cost += -(inst->rho[v2_prev][v2_next]);
      /// Add the costs of two new edges
      cost += inst->rho[v2_prev][vertex];
      cost += inst->rho[vertex][v2_next];

      /* /// Remove the costs of the edges of the vertex to be relocated */

      /* cost += -(inst->rho[pathCvrp[src_vehicle][i-1]][pathCvrp[src_vehicle][i]]); */
      /* cost += -(inst->rho[pathCvrp[src_vehicle][i]][pathCvrp[src_vehicle][i+1]]); */
      /* /// Reconnect them */
      /* cost += inst->rho[pathCvrp[src_vehicle][i-1]][pathCvrp[src_vehicle][i+1]]; */

      /* /// Prepare space for new node */
      /* cost += -(inst->rho[pathCvrp[tgt_vehicle][j-1]][pathCvrp[tgt_vehicle][j]]); */
      /* /// Add the costs of two new edges */
      /* cost += inst->rho[pathCvrp[tgt_vehicle][j-1]][pathCvrp[src_vehicle][i]]; */
      /* cost += inst->rho[pathCvrp[src_vehicle][i]][pathCvrp[tgt_vehicle][j]]; */

      ret = cost;
      return ret;
    } else {
      CVRPLIB_LOG_ERROR("useMapf flag has invalid value!");
      return ret;
    }

}

void Solution::relocate(uint16_t src_vehicle,
        uint16_t tgt_vehicle,
        Pos i,
        Pos j
)
{
  Ver vertex = pathCvrp[src_vehicle][i];

  /* std::cout << "inserting " << pathCvrp[src_vehicle][i] << " from position " << i << " and route " << src_vehicle << " to position " << j << " in route " << tgt_vehicle << "\n"; */
  /* sleep(1); */
    // update vehicle load
  current_demands[src_vehicle] += -(inst->demands[vertex]);
  current_demands[tgt_vehicle] += inst->demands[vertex];

  if (src_vehicle == tgt_vehicle){
    if (i < j){
      pathCvrp[tgt_vehicle].insert(pathCvrp[tgt_vehicle].begin() + j, vertex);
      pathCvrp[src_vehicle].erase(pathCvrp[src_vehicle].begin() + i);
    } else {
      pathCvrp[src_vehicle].erase(pathCvrp[src_vehicle].begin() + i);
      pathCvrp[tgt_vehicle].insert(pathCvrp[tgt_vehicle].begin() + j, vertex);
    }

  } else {
    pathCvrp[tgt_vehicle].insert(pathCvrp[tgt_vehicle].begin() + j, vertex);
    pathCvrp[src_vehicle].erase(pathCvrp[src_vehicle].begin() + i);
  }



}


/// --------------------------------------------------------------------------------------------------------------------
/// Operator CVRP - Two-opt move
/// --------------------------------------------------------------------------------------------------------------------

MainType Solution::tryTwoOptCvrp(uint16_t vehicle,
        Pos i,
        Pos j
)
{

    if (i > j){
      Pos temp = i;
      i = j;
      j = temp;
    }
 
    MainType ret = -1;


    /* if (inst->mapfLevelOps && useMapf){ */
    /*   /// TODO: Implement call to computeTwoOptCost here */
    /*   /// vehicle is the agent we are performing the operation on, */
    /*   /// i and j denote the edges we are reconnecting (edge between i-1 and i gets reconencted to i-1 and j) */
    /*   /// meaning the segment starting from i and ending with j is reversed */
    /*   /// Do not forget to flip the boolean twoOptInMapf to true */

    /*   mapf_operator_calls[2] += 1; */

    /*   inst->mapfSolver->computeTwoOpt(vehicle, i, j); */

    /*   if (ret != -1){ */
    /*     return ret; */
    /*   } else { */
    /*     CVRPLIB_LOG_ERROR("MAPF twoOpt returned invalid cost."); */
    /*     return INT_MAX; */
    /*   } */
    /* } else if (useMapf){ */
    if (useMapf) {
      /* mapf_operator_calls[2] += 1; */
      MAPF_input mapfTask;
      mapfTask.sequences = pathCvrp;
      std::reverse(mapfTask.sequences[vehicle].begin() + i, mapfTask.sequences[vehicle].begin() + j + 1);
      ret = inst->mapfSolver->computeCost(mapfTask).cost;
      if (ret){
        return ret;
      } else {
        CVRPLIB_LOG_ERROR("MAPF returned invalid cost.");
        return INT_MAX;
      }
    } else if (inst->useTtd) { 

      uint32_t cost = current_cost;
      uint32_t delta_cost = 0;
      /// Remove the costs of the two original edges
      delta_cost += -(inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][i]]);
      delta_cost += -(inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][j+1]]);
      /// Add the costs of two new edges
      delta_cost += inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][j]];
      delta_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][j+1]];

      delta_cost = delta_cost*(pathCvrp[vehicle].size() - 2);
      cost = cost + delta_cost;

      /* ret = cost; */
      return cost;


    } else {
      /* operator_calls[2] += 1; */
      /// Use simple cost calculation
      uint32_t cost = current_cost;
      /// Remove the costs of the two original edges
      cost += -(inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][i]]);
      cost += -(inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][j+1]]);
      /// Add the costs of two new edges
      cost += inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][j]];
      cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][j+1]];

      /* ret = cost; */
      return cost;
    }
}

void Solution::twoOptCvrp(uint16_t vehicle,
        Pos i,
        Pos j
)
{
    /// Reverse the order of the elements in the range [first, last), i.e., [i + 1, j + 1), i.e., [i + 1, j].
    std::reverse(pathCvrp[vehicle].begin() + i, pathCvrp[vehicle].begin() + j + 1);
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator CVRP - Three-opt move
/// --------------------------------------------------------------------------------------------------------------------

MainType Solution::tryThreeOptCvrp(uint16_t vehicle,
        Pos i_in,
        Pos j_in,
        Pos k_in
)
{
    MainType ret = -1;
    uint32_t cost = 0;
 
    /// To simplify, order i j k so that they are in ascending order. May not be necessary in the future
    Pos i = std::min(i_in, std::min(j_in, k_in));
    Pos k = std::max(i_in, std::max(j_in, k_in));
    Pos j;

    if (i == j_in)
    {
      j = std::min(i_in, k_in);
    } else if (k == j_in)
    {
      j = std::max(i_in, k_in);
    } else {
      j = j_in;
    }

    Pos a = i-1;
    Pos b = j-1;
    Pos c = k-1;
    /* std::cout << "These should be in ascending order: " << a << " " << i << " " << b << " " << j << " " << c << " " << k << "\n"; */

    /* if (inst->mapfLevelOps && useMapf){ */
    /*   /// TODO: Implement call to computeThreeOptCost here */
    /*   /// i, j and k denote the vertices which have their incoming edges removed, and those are then reconnected. */
    /*   /// Do not forget to flip the boolean threeOptInMapf to true */

    /*   mapf_operator_calls[3] += 1; */

    /*   inst->mapfSolver->computeThreeOpt(vehicle, i_in, j_in, k_in); */

    /*   if (ret != -1){ */
    /*     return ret; */
    /*   } else { */
    /*     CVRPLIB_LOG_ERROR("MAPF threeOpt returned invalid cost."); */
    /*     return INT_MAX; */
    /*   } */
    /* } else if (useMapf){ */
    if (useMapf) {
      /* mapf_operator_calls[3] += 1; */
      float costA = -1, costB = -1, costC = -1, costD = -1;
      /// Prepare 4 different mapf tasks
      MAPF_input mapfTaskA;
      MAPF_input mapfTaskB;
      MAPF_input mapfTaskC;
      MAPF_input mapfTaskD;

      mapfTaskA.sequences = pathCvrp;
      mapfTaskB.sequences = pathCvrp;
      mapfTaskC.sequences = pathCvrp;
      mapfTaskD.sequences = pathCvrp;
 
      std::vector<Ver> pathA;
      std::vector<Ver> pathB;
      std::vector<Ver> pathC;
      std::vector<Ver> pathD;

      /// extract segments from routes
      /// 1 - a
      std::vector<Ver> segment_start;
      /// i - b
      std::vector<Ver> segment_i;
      /// j - c
      std::vector<Ver> segment_j;
      /// k - 1
      std::vector<Ver> segment_k;
      segment_start.insert(segment_start.end(), pathCvrp[vehicle].begin(), pathCvrp[vehicle].begin()+i);
      segment_i.insert(segment_i.end(), pathCvrp[vehicle].begin()+i, pathCvrp[vehicle].begin()+j);
      segment_j.insert(segment_j.end(), pathCvrp[vehicle].begin()+j, pathCvrp[vehicle].begin()+k);
      segment_k.insert(segment_k.end(), pathCvrp[vehicle].begin()+k, pathCvrp[vehicle].end());

      /// c - j
      std::vector<Ver> segment_i_reverse = segment_i;
      std::reverse(segment_i_reverse.begin(), segment_i_reverse.end());
 
      /// b - i
      std::vector<Ver> segment_j_reverse = segment_j;
      std::reverse(segment_j_reverse.begin(), segment_j_reverse.end());

      /// perform move1
      /// AB iC jk
      pathA.insert(pathA.end(), segment_start.begin(), segment_start.end());
      pathA.insert(pathA.end(), segment_i_reverse.begin(), segment_i_reverse.end());
      pathA.insert(pathA.end(), segment_j_reverse.begin(), segment_j_reverse.end());
      pathA.insert(pathA.end(), segment_k.begin(), segment_k.end());
 
      mapfTaskA.sequences[vehicle] = pathA;

      /// perform move2
      /// AC ji Bk
      pathB.insert(pathB.end(), segment_start.begin(), segment_start.end());
      pathB.insert(pathB.end(), segment_j_reverse.begin(), segment_j_reverse.end());
      pathB.insert(pathB.end(), segment_i.begin(), segment_i.end());
      pathB.insert(pathB.end(), segment_k.begin(), segment_k.end());
      
      mapfTaskB.sequences[vehicle] = pathB;

      /// perform move3
      /// Aj CB ik
      pathC.insert(pathC.end(), segment_start.begin(), segment_start.end());
      pathC.insert(pathC.end(), segment_j.begin(), segment_j.end());
      pathC.insert(pathC.end(), segment_i_reverse.begin(), segment_i_reverse.end());
      pathC.insert(pathC.end(), segment_k.begin(), segment_k.end());

      mapfTaskC.sequences[vehicle] = pathC;

      /// perform move4
      /// Aj Ci Bk
      pathD.insert(pathD.end(), segment_start.begin(), segment_start.end());
      pathD.insert(pathD.end(), segment_j.begin(), segment_j.end());
      pathD.insert(pathD.end(), segment_i.begin(), segment_i.end());
      pathD.insert(pathD.end(), segment_k.begin(), segment_k.end());
      
      mapfTaskD.sequences[vehicle] = pathD;
      
      costA = inst->mapfSolver->computeCost(mapfTaskA).cost;
      costB = inst->mapfSolver->computeCost(mapfTaskB).cost;
      costC = inst->mapfSolver->computeCost(mapfTaskC).cost;
      costD = inst->mapfSolver->computeCost(mapfTaskD).cost;
      ret = std::min({costA, costB, costC, costD});

      // implement selection that filters -1s

      if (ret != -1){
        return ret;
      } else {
        CVRPLIB_LOG_ERROR("MAPF returned invalid cost.");
        return INT_MAX;
      }
    } else if (inst->useTtd) {

      /* operator_calls[3] += 1; */
      /* uint32_t seg_k_cost = 0; */
      uint32_t fixed_seg_cost = 0;
      std::vector<Ver> seg_vertices;
      /// the unchanged segment cost
      /* fixed_seg_cost = current_cost; */
      fixed_seg_cost = 0;
      /// remove the costs of three original edges

      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][i]];
      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][j]];
      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][k]];

      /// Calculate the costs of all 4 possible moves
      uint32_t move1_cost = fixed_seg_cost;
      uint32_t move2_cost = fixed_seg_cost;
      uint32_t move3_cost = fixed_seg_cost;
      uint32_t move4_cost = fixed_seg_cost;

      /// move 1
      /// AB iC jk
      move1_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][b]];
      move1_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][c]];
      move1_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][k]];
      /// AC ji Bk
      move2_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][c]];
      move2_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][i]];
      move2_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];
      /// Aj CB ik
      move3_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
      move3_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][b]];
      move3_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][k]];
      /// Aj Ci Bk
      move4_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
      move4_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][i]];
      move4_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];

      /// Select the best move and return its cost
      int delta_cost = std::min({move1_cost, move2_cost, move3_cost, move4_cost}) * (pathCvrp[vehicle].size() - 2);
      cost = current_cost + delta_cost;

      return cost;

    } else {
    
      /* operator_calls[3] += 1; */
      /* uint32_t seg_k_cost = 0; */
      uint32_t fixed_seg_cost = 0;
      std::vector<Ver> seg_vertices;
      /// the unchanged segment cost
      fixed_seg_cost = current_cost;
      /// remove the costs of three original edges
      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][i]];
      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][j]];
      fixed_seg_cost += -inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][k]];

      /// Calculate the costs of all 4 possible moves
      uint32_t move1_cost = fixed_seg_cost;
      uint32_t move2_cost = fixed_seg_cost;
      uint32_t move3_cost = fixed_seg_cost;
      uint32_t move4_cost = fixed_seg_cost;

      /// move 1
      /// AB iC jk
      move1_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][b]];
      move1_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][c]];
      move1_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][k]];
      /// AC ji Bk
      move2_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][c]];
      move2_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][i]];
      move2_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];
      /// Aj CB ik
      move3_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
      move3_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][b]];
      move3_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][k]];
      /// Aj Ci Bk
      move4_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
      move4_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][i]];
      move4_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];

      /// Select the best move and return its cost
      cost = std::min({move1_cost, move2_cost, move3_cost, move4_cost});

      return cost;
    }
}

void Solution::threeOptCvrp(uint16_t vehicle,
        Pos i_in,
        Pos j_in,
        Pos k_in
)
{

    /// order ijk just in case they're not in ascending order
    Pos i = std::min(i_in, std::min(j_in, k_in));
    Pos k = std::max(i_in, std::max(j_in, k_in));
    Pos j;
    if (i == j_in)
    {
      j = std::min(i_in, k_in);
    } else if (k == j_in)
    {
      j = std::max(i_in, k_in);
    } else {
      j = j_in;
    }
    Pos a = i-1;
    Pos b = j-1;
    Pos c = k-1;

    /* std::cout << "These should be in ascending order: " << a << " " << i << " " << b << " " << j << " " << c << " " << k << "\n"; */

    /// calculate costs of segments
    uint16_t fixed_seg_cost = 0;
    std::vector<Ver> seg_vertices;
    /// calculate the unchanged segment cost
    fixed_seg_cost = current_cost;
    fixed_seg_cost += -inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][i]];
    fixed_seg_cost += -inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][j]];
    fixed_seg_cost += -inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][k]];

    /// Calculate the costs of all 4 possible moves
    uint32_t move1_cost = fixed_seg_cost;
    uint32_t move2_cost = fixed_seg_cost;
    uint32_t move3_cost = fixed_seg_cost;
    uint32_t move4_cost = fixed_seg_cost;

    /// move 1
    /// AB iC jk
    move1_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][b]];
    move1_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][c]];
    move1_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][k]];
    /// AC ji Bk
    move2_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][c]];
    move2_cost += inst->rho[pathCvrp[vehicle][j]][pathCvrp[vehicle][i]];
    move2_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];
    /// Aj CB ik
    move3_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
    move3_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][b]];
    move3_cost += inst->rho[pathCvrp[vehicle][i]][pathCvrp[vehicle][k]];
    /// Aj Ci Bk
    move4_cost += inst->rho[pathCvrp[vehicle][a]][pathCvrp[vehicle][j]];
    move4_cost += inst->rho[pathCvrp[vehicle][c]][pathCvrp[vehicle][i]];
    move4_cost += inst->rho[pathCvrp[vehicle][b]][pathCvrp[vehicle][k]];

    /// Select the best move and apply
    float cost = 0;
    cost = std::min({move1_cost, move2_cost, move3_cost, move4_cost});
    std::vector<uint32_t> costs = {move1_cost, move2_cost, move3_cost, move4_cost};
    uint16_t index_cheapest = 0;

    for (index_cheapest = 0; index_cheapest < costs.size(); ++index_cheapest){
      if (costs[index_cheapest] == cost){
        break;
      }
    }

    /// extract segments from routes
    /// 1 - a
    std::vector<Ver> segment_start;
    /// i - b
    std::vector<Ver> segment_i;
    /// j - c
    std::vector<Ver> segment_j;
    /// k - 1
    std::vector<Ver> segment_k;

    /* std::cout << "size is: " << pathCvrp[vehicle].size() << "\n"; */
    /* std::cout << "a " << a << " b " << b << " c " << c << " i " << i << " j " << j << " k " << k << "\n"; */

    /* std::cout << "whole path is : "; */
    /* for (uint16_t iter = 0; iter < pathCvrp[vehicle].size(); iter++){ */
    /*   std::cout << pathCvrp[vehicle][iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */

    segment_start.insert(segment_start.end(), pathCvrp[vehicle].begin(), pathCvrp[vehicle].begin()+i);
    segment_i.insert(segment_i.end(), pathCvrp[vehicle].begin()+i, pathCvrp[vehicle].begin()+j);
    segment_j.insert(segment_j.end(), pathCvrp[vehicle].begin()+j, pathCvrp[vehicle].begin()+k);
    segment_k.insert(segment_k.end(), pathCvrp[vehicle].begin()+k, pathCvrp[vehicle].end());
    
    /* std::cout << "segment start is: "; */
    /* for (uint16_t iter = 0; iter < segment_start.size(); iter++){ */
    /*   std::cout << segment_start[iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */

    /* std::cout << "segment i is: "; */
    /* for (uint16_t iter = 0; iter < segment_i.size(); iter++){ */
    /*   std::cout << segment_i[iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */

    /* std::cout << "segment j is: "; */
    /* for (uint16_t iter = 0; iter < segment_j.size(); iter++){ */
    /*   std::cout << segment_j[iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */

    /* std::cout << "segment k is: "; */
    /* for (uint16_t iter = 0; iter < segment_k.size(); iter++){ */
    /*   std::cout << segment_k[iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */
    
    /* std::cout << "4\n"; */
    std::vector<Ver> segment_i_reverse = segment_i;
    std::reverse(segment_i_reverse.begin(), segment_i_reverse.end());
    
    /// c - j
    std::vector<Ver> segment_j_reverse = segment_j;
    std::reverse(segment_j_reverse.begin(), segment_j_reverse.end());


    std::vector<Ver> pathTemp;

    switch(index_cheapest) {
      case 0:
        /// perform move1
        /// AB iC jk
        pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
        pathTemp.insert(pathTemp.end(), segment_i_reverse.begin(), segment_i_reverse.end());
        pathTemp.insert(pathTemp.end(), segment_j_reverse.begin(), segment_j_reverse.end());
        pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
        pathCvrp[vehicle] = pathTemp;
        break;

      case 1:
        /// perform move2
        /// AC ji Bk
        pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
        pathTemp.insert(pathTemp.end(), segment_j_reverse.begin(), segment_j_reverse.end());
        pathTemp.insert(pathTemp.end(), segment_i.begin(), segment_i.end());
        pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
        pathCvrp[vehicle] = pathTemp;
        break;

      case 2:
        /// perform move3
        /// Aj CB ik
        pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
        pathTemp.insert(pathTemp.end(), segment_j.begin(), segment_j.end());
        pathTemp.insert(pathTemp.end(), segment_i_reverse.begin(), segment_i_reverse.end());
        pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
        pathCvrp[vehicle] = pathTemp;
        break;

      case 3:
        /// perform move4
        /// Aj Ci Bk
        pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
        pathTemp.insert(pathTemp.end(), segment_j.begin(), segment_j.end());
        pathTemp.insert(pathTemp.end(), segment_i.begin(), segment_i.end());
        pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
        pathCvrp[vehicle] = pathTemp;
        break;
      default:
        CVRPLIB_LOG_WARNING("Something broke in 3opt - best possible move index has unexpected value. No move performed.");
    }
    
    /* std::cout << "path after 3opt is:"; */
    /* for (uint16_t iter = 0; iter < pathCvrp[vehicle].size(); iter++){ */
    /*   std::cout << pathCvrp[vehicle][iter] << " - "; */
    /* } */
    /* std::cout << "\n"; */

}


/// --------------------------------------------------------------------------------------------------------------------
/// Operator CVRP - Double-bridge move
/// --------------------------------------------------------------------------------------------------------------------

MainType Solution::tryDoubleBridgeCvrp(uint16_t vehicle,
        Pos i,
        Pos j,
        Pos k,
        Pos l
)
{

    /* if (i > j){ */
    /*   Pos temp = i; */
    /*   i = j; */
    /*   j = temp; */
    /* } */
    
    /* if (k > l){ */
    /*   Pos temp = i; */
    /*   i = j; */
    /*   j = temp; */
    /* } */

    MainType ret = -1;
    
    if (useMapf){
      /* mapf_operator_calls[4] += 1; */
      
      /// extract segments from routes
      /// 1 - a
      std::vector<Ver> segment_start;
      /// i - b
      std::vector<Ver> segment_i;
      /// j - c
      std::vector<Ver> segment_j;
      /// k - d
      std::vector<Ver> segment_k;
      /// l - 1
      std::vector<Ver> segment_l;

      segment_start.insert(segment_start.end(), pathCvrp[vehicle].begin(), pathCvrp[vehicle].begin()+i);
      segment_i.insert(segment_i.end(), pathCvrp[vehicle].begin()+i, pathCvrp[vehicle].begin()+j);
      segment_j.insert(segment_j.end(), pathCvrp[vehicle].begin()+j, pathCvrp[vehicle].begin()+k);
      segment_k.insert(segment_k.end(), pathCvrp[vehicle].begin()+k, pathCvrp[vehicle].begin()+l);
      segment_l.insert(segment_l.end(), pathCvrp[vehicle].begin()+l, pathCvrp[vehicle].end());
      
      std::vector<Ver> pathTemp;

      pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
      pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
      pathTemp.insert(pathTemp.end(), segment_j.begin(), segment_j.end());
      pathTemp.insert(pathTemp.end(), segment_i.begin(), segment_i.end());
      pathTemp.insert(pathTemp.end(), segment_l.begin(), segment_l.end());
      
      MAPF_input mapfTask;
      mapfTask.sequences = pathCvrp;
      mapfTask.sequences[vehicle] = pathTemp;
      ret = inst->mapfSolver->computeCost(mapfTask).cost;
      if (ret){
        return ret;
      } else {
        CVRPLIB_LOG_ERROR("MAPF returned invalid cost. (double bridge)");
        return INT_MAX;
      }
    } else {

      /* operator_calls[4] += 1; */
      /// Use simple cost calculation
      uint32_t cost = current_cost;
      /// Remove the costs of the two original edges
      cost += -(inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][i]]);
      cost += -(inst->rho[pathCvrp[vehicle][j-1]][pathCvrp[vehicle][j]]);
      cost += -(inst->rho[pathCvrp[vehicle][k-1]][pathCvrp[vehicle][k]]);
      cost += -(inst->rho[pathCvrp[vehicle][l-1]][pathCvrp[vehicle][l]]);
      /// Add the costs of two new edges
      cost += inst->rho[pathCvrp[vehicle][i-1]][pathCvrp[vehicle][k]];
      cost += inst->rho[pathCvrp[vehicle][l-1]][pathCvrp[vehicle][j]];
      cost += inst->rho[pathCvrp[vehicle][k-1]][pathCvrp[vehicle][i]];
      cost += inst->rho[pathCvrp[vehicle][j-1]][pathCvrp[vehicle][l]];

      /* ret = cost; */
      std::cout << "doublebridge cost successfuly computed\n";
      return cost;
    }
}

void Solution::doubleBridgeCvrp(uint16_t vehicle,
        Pos i,
        Pos j,
        Pos k,
        Pos l
)
{
    /// extract segments from routes
    /// 1 - a
    std::vector<Ver> segment_start;
    /// i - b
    std::vector<Ver> segment_i;
    /// j - c
    std::vector<Ver> segment_j;
    /// k - d
    std::vector<Ver> segment_k;
    /// l - 1
    std::vector<Ver> segment_l;

    segment_start.insert(segment_start.end(), pathCvrp[vehicle].begin(), pathCvrp[vehicle].begin()+i);
    segment_i.insert(segment_i.end(), pathCvrp[vehicle].begin()+i, pathCvrp[vehicle].begin()+j);
    segment_j.insert(segment_j.end(), pathCvrp[vehicle].begin()+j, pathCvrp[vehicle].begin()+k);
    segment_k.insert(segment_k.end(), pathCvrp[vehicle].begin()+k, pathCvrp[vehicle].begin()+l);
    segment_l.insert(segment_l.end(), pathCvrp[vehicle].begin()+l, pathCvrp[vehicle].end());
    
    std::vector<Ver> pathTemp;
    
    pathTemp.insert(pathTemp.end(), segment_start.begin(), segment_start.end());
    pathTemp.insert(pathTemp.end(), segment_k.begin(), segment_k.end());
    pathTemp.insert(pathTemp.end(), segment_j.begin(), segment_j.end());
    pathTemp.insert(pathTemp.end(), segment_i.begin(), segment_i.end());
    pathTemp.insert(pathTemp.end(), segment_l.begin(), segment_l.end());
    
    pathCvrp[vehicle] = pathTemp;
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator - Two-opt move
/// --------------------------------------------------------------------------------------------------------------------

void Solution::moveVer2Opt(
        Ver u,
        Ver v
)
{
    movePos2Opt(pathInv[u], pathInv[v]);
}

MainType Solution::improvVer2Opt(
        Ver u,
        Ver v
) const
{
    return improvPos2Opt(pathInv[u], pathInv[v]);
}

void Solution::movePos2Opt(
        Pos i,
        Pos j
)
{
    /// Reverse the order of the elements in the range [first, last), i.e., [i + 1, j + 1), i.e., [i + 1, j].
    std::reverse(path.begin() + i, path.begin() + j + 1);
    /// Update inverse.
    for (unsigned k = i; k <= j; ++k) {
        pathInv[path[k]] = k;
    }
    updateStructures();
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator - Two-string move
/// --------------------------------------------------------------------------------------------------------------------

void Solution::moveVer2String(
        Ver u,
        Ver v,
        Pos X,
        Pos Y
)
{
    movePos2String(pathInv[u], pathInv[v], X, Y);
}

MainType Solution::improvVer2String(
        Ver u,
        Ver v,
        Pos X,
        Pos Y
) const
{
    return improvPos2String(pathInv[u], pathInv[v], X, Y);
}

void Solution::movePos2String(
        Pos i,
        Pos j,
        Pos X,
        Pos Y
)
{
    if ((X == 0 || Y == 0) && i == j) return;
    /// If (j < i) -> swap (i and j) and (X and Y).
    if (j < i) {
        movePos2String(j, i, Y, X);
    } else {
        /// This will be string (i + 1, ..., i + X) of the original path.
        std::vector<int> auxIPlus1ToIPlusX(static_cast<unsigned>(X));
        /// This will be string (i + X + 1, ..., j) of the original path.
        std::vector<int> auxIPlusXPlus1ToJ(static_cast<unsigned>(j - i - X));
        /// This will be string (j + 1, ..., j + Y) of the original path.
        std::vector<int> auxJPlus1ToJPlusY(static_cast<unsigned>(Y));
        /// Save auxiliary substrings of the original path.
        for (Pos k = 0; k < auxIPlus1ToIPlusX.size(); ++k)
            auxIPlus1ToIPlusX[k] = path[i + 1 + k];
        for (Pos k = 0; k < auxIPlusXPlus1ToJ.size(); ++k)
            auxIPlusXPlus1ToJ[k] = path[i + X + 1 + k];
        for (Pos k = 0; k < auxJPlus1ToJPlusY.size(); ++k)
            auxJPlus1ToJPlusY[k] = path[j + 1 + k];
        /// Update path.
        for (Pos k = 0; k < auxJPlus1ToJPlusY.size(); ++k)
            path[i + 1 + k] = auxJPlus1ToJPlusY[k];
        for (Pos k = 0; k < auxIPlusXPlus1ToJ.size(); ++k)
            path[i + Y + 1 + k] = auxIPlusXPlus1ToJ[k];
        for (Pos k = 0; k < auxIPlus1ToIPlusX.size(); ++k)
            path[j + Y - X + 1 + k] = auxIPlus1ToIPlusX[k];
        /// Update inverse.
        for (Pos k = i + 1; k <= j + Y; k++) pathInv[path[k]] = k;
        /// Update structures.
        updateStructures();
    }
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator - Two-point move
/// --------------------------------------------------------------------------------------------------------------------

void Solution::moveVer2Point(
        Ver u,
        Ver v
)
{
    movePos2Point(pathInv[u], pathInv[v]);
}

MainType Solution::improvVer2Point(
        Ver u,
        Ver v
) const
{
    return improvPos2Point(pathInv[u], pathInv[v]);
}

void Solution::movePos2Point(
        Pos i,
        Pos j
)
{
    auto &u = path[i + 1];
    auto &v = path[j + 1];
    std::swap(u, v);
    std::swap(pathInv[u], pathInv[v]);
    updateStructures();
}

/// --------------------------------------------------------------------------------------------------------------------
/// Operator - One-point move
/// --------------------------------------------------------------------------------------------------------------------

void Solution::moveVer1Point(
        Ver u,
        Ver v
)
{
    movePos1Point(pathInv[u], pathInv[v]);
}

MainType Solution::improvVer1Point(
        Ver u,
        Ver v
) const
{
    return improvPos1Point(pathInv[u], pathInv[v]);
}

void Solution::movePos1Point(
        Pos i,
        Pos j
)
{
    if (i == j) return;
    if (i < j) {
        /// This will be string (i + 1, ..., j) of the original path.
        std::vector<int> auxIPlus1ToJ(static_cast<unsigned>(j - i));
        /// Save auxiliary substrings of the original path.
        for (Pos k = 0; k < auxIPlus1ToJ.size(); ++k)
            auxIPlus1ToJ[k] = path[i + 1 + k];
        /// Update path.
        path[i + 1] = path[j + 1];
        for (Pos k = 0; k < auxIPlus1ToJ.size(); ++k)
            path[i + 2 + k] = auxIPlus1ToJ[k];
        /// Update inverse.
        for (Pos k = i + 1; k <= j + 1; k++) pathInv[path[k]] = k;
    } else {
        int auxJPlus1 = path[j + 1];
        /// This will be string (j + Y + 1, ..., i) of the original path.
        std::vector<int> auxJPlusYPlus1ToI(static_cast<unsigned>(i - j - 1));
        /// Save auxiliary substrings of the original path.
        for (Pos k = 0; k < auxJPlusYPlus1ToI.size(); ++k)
            auxJPlusYPlus1ToI[k] = path[j + 2 + k];
        /// Update path.
        for (Pos k = 0; k < auxJPlusYPlus1ToI.size(); ++k)
            path[j + 1 + k] = auxJPlusYPlus1ToI[k];
        path[i] = auxJPlus1;
        /// Update inverse.
        for (Pos k = j; k <= i; k++) pathInv[path[k]] = k;
    }
    /// Update structures.
    updateStructures();
}

/// --------------------------------------------------------------------------------------------------------------------
/// Protected constructor
/// --------------------------------------------------------------------------------------------------------------------


Solution::Solution(
        const cvrplib::Instance &inst,
        std::string name
)
        : inst(&inst),
          path(inst.n, INVALID_PATH_FLAG),
          pathInv(inst.n, std::numeric_limits<Pos>::max()),
          name(std::move(name))
{

}
