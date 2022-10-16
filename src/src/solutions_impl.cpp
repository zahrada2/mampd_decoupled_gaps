/**
 * File:    solutions_impl.cpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "solutions_impl.hpp"

using namespace cvrplib;
using namespace cvrplib::solutions;
using namespace cvrplib::solutions::impl;


/// --------------------------------------------------------------------------------------------------------------------
/// CVRP-Solution-naive
/// --------------------------------------------------------------------------------------------------------------------

CvrpPathNaive::CvrpPathNaive(
        const Instance &inst,
        std::string name
)
        : Solution(inst, name)
{

}

void CvrpPathNaive::reinitStructures(unsigned n)
{

}

void CvrpPathNaive::updateStructures()
{

}

std::unique_ptr<Solution> CvrpPathNaive::makeNewNaive() const
{
    return nullptr;
}

MainType CvrpPathNaive::computeDemands(uint16_t vehicle) const
{
    MainType ret = 0;
    for (Pos i = 1; i < pathCvrp[vehicle].size() - 1; ++i) {
        ret += inst->demands[pathCvrp[vehicle][i]];
    }
    return ret;
}

MainType CvrpPathNaive::computeDemandsIfAdded(uint16_t vehicle, Ver vertex) const
{
  MainType ret = 0;
  ret = computeDemands(vehicle) + inst->demands[vertex];
  return ret;

}

bool CvrpPathNaive::canFitIntoCapacity(uint16_t vehicle, Ver vertex) const
{
  bool ret = 0;
  MainType demand = 0;
  demand = computeDemands(vehicle) + inst->demands[vertex];
  ret = (demand <= inst->vehicleCapacity);
  return ret;

}


MainType CvrpPathNaive::computeTSPCost() const
{
    MainType ret = 0;
    ret = computeCost();
    /* for (Pos i = 0; i < inst->n - 1; ++i) { */
    /*     ret += inst->rho[path[i]][path[i + 1]]; */
    /* } */
    /* ret += inst->rho[path[inst->n - 1]][path[0]]; */
    return ret;
}

MainType CvrpPathNaive::computeTDPCost() const
{
    MainType ret = 0;
    for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
      uint32_t n = getNumVertices(vehicle);
      for (Pos i = 1; i < n; ++i) {
          ret += (n - 1 - i + 1) * inst->rho[pathCvrp[vehicle][i - 1]][pathCvrp[vehicle][i]];
      }
    }
    return ret;
}

bool CvrpPathNaive::isFeasible() const{

  bool ret = true;
  for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    uint32_t num_vertices = pathCvrp[vehicle].size();
    for (uint32_t carried_vertex = 1; carried_vertex < num_vertices; ++carried_vertex){
      //TODO: rename orders to target_depot
      if (inst->orders[pathCvrp[vehicle][carried_vertex]] != (int) (pathCvrp[vehicle][num_vertices-1]/5)){
        ret = false;
      }
    }

  }

  return ret;
}

bool CvrpPathNaive::isExchangeFeasible(uint16_t vehicle1, uint16_t vehicle2, uint16_t ver1, uint16_t ver2) const{

  bool ret = true;
  uint32_t num_vertices1 = pathCvrp[vehicle1].size();
  if (inst->orders[pathCvrp[vehicle2][ver2]] != (int) (pathCvrp[vehicle2][num_vertices1-1]/5)){
    ret = false;
  }
  uint32_t num_vertices2 = pathCvrp[vehicle2].size();
  if (inst->orders[pathCvrp[vehicle1][ver1]] != (int) (pathCvrp[vehicle2][num_vertices2-1]/5)){
    ret = false;
  }
  /* for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   uint32_t num_vertices = pathCvrp[vehicle].size(); */
  /*   for (uint32_t carried_vertex = 1; carried_vertex < num_vertices; ++carried_vertex){ */
  /*     //TODO: rename orders to target_depot */
  /*     if (inst->orders[pathCvrp[vehicle][carried_vertex]] != (int) (pathCvrp[vehicle][num_vertices-1]/5)){ */
  /*       ret = false; */
  /*     } */
  /*   } */

  /* } */

  return ret;
}

bool CvrpPathNaive::isRelocateFeasible(uint16_t vehicle_from, uint16_t vehicle_to, Ver vertex) const{

  bool ret = true;
  uint32_t depot_id = pathCvrp[vehicle_to].size() - 1;
  if (inst->orders[pathCvrp[vehicle_from][vertex]] != (int) (pathCvrp[vehicle_to][depot_id]/5)){
    ret = false;
  }
  /* for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   uint32_t num_vertices = pathCvrp[vehicle].size(); */
  /*   for (uint32_t carried_vertex = 1; carried_vertex < num_vertices; ++carried_vertex){ */
  /*     //TODO: rename orders to target_depot */
  /*     if (inst->orders[pathCvrp[vehicle][carried_vertex]] != (int) (pathCvrp[vehicle][num_vertices-1]/5)){ */
  /*       ret = false; */
  /*     } */
  /*   } */

  /* } */

  return ret;
}

void CvrpPathNaive::precomputeTTDDiscount(){
  MainType ttd_discount = 0;
  for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    if (pathCvrp[vehicle].size() > 2){
      /* int depot = pathCvrp[vehicle][pathCvrp[vehicle].size()-1]; */
      for (uint16_t subgoal = 1; subgoal < pathCvrp[vehicle].size()-1; ++subgoal){
        ttd_discount += inst->closestDepotDistances[pathCvrp[vehicle][subgoal]];
        /* std::cout << inst->closestDepotDistances[subgoal]; */
        /* ret = ret - inst->rho[pathCvrp[vehicle][subgoal]][depot]; */
      }
    } else {
    }
  }
  precomputedTTD = ttd_discount;

}

MainType CvrpPathNaive::computeTTD() const{
  MainType ret = 0;
  for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
    if (pathCvrp[vehicle].size() > 2){
      ret += computeSingleRouteCost(vehicle)*(pathCvrp[vehicle].size()-2);
      /* int depot = pathCvrp[vehicle][pathCvrp[vehicle].size()-1]; */
      /* for (uint16_t subgoal = 1; subgoal < pathCvrp[vehicle].size()-1; ++subgoal){ */
      /*   ret = ret - inst->closestDepotDistances[pathCvrp[vehicle][subgoal]]; */
        /* std::cout << inst->closestDepotDistances[subgoal]; */
        /* ret = ret - inst->rho[pathCvrp[vehicle][subgoal]][depot]; */
      /* } */
    } else {
      // no goals, zero cost
      ret += 0;
    }
  }
  // apply discounts ^^
  ret = ret - precomputedTTD;
  /* for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /* } */

  return ret;
}

MainType CvrpPathNaive::computeCostSelected(bool use_ttd) const
{
    MainType ret = 0;
    /* if (!isFeasible()){ */
    /*   ret = 10000; */
    /*   return ret; */
    /* } */
    if (use_ttd){
      ret = computeTTD();
    } else {
      for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
        ret += computeSingleRouteCost(vehicle);
      }
    }
    /* MainType ret = 0; */
    /* for (Pos i = 1; i < inst->n; ++i) { */
    /*     ret += (inst->n - 1 - i + 1) * inst->rho[path[i - 1]][path[i]]; */
    /* } */
    return ret;
}

MainType CvrpPathNaive::computeCost() const
{
    MainType ret = 0;
    /* if (!isFeasible()){ */
    /*   ret = 10000; */
    /*   return ret; */
    /* } */
    if (inst->useTtd){
      ret = computeTTD();
    } else {
      for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
        ret += computeSingleRouteCost(vehicle);
      }
    }
    /* MainType ret = 0; */
    /* for (Pos i = 1; i < inst->n; ++i) { */
    /*     ret += (inst->n - 1 - i + 1) * inst->rho[path[i - 1]][path[i]]; */
    /* } */
    return ret;
}

MainType CvrpPathNaive::computeSingleRouteCost(uint16_t vehicle) const
{
  MainType ret = 0;

  /// The distance travelled by the vehicle (0 for now)
  float c = 0;
  /// Penalization for overloading demand
  float pen = 0.0;
  /* alpha = 1; */

  for (uint16_t vtx = 0; vtx < pathCvrp[vehicle].size()-1; ++vtx){
    c+=inst->rho[pathCvrp[vehicle][vtx]][pathCvrp[vehicle][vtx+1]];
  }


  for (uint16_t vtx = 0; vtx < pathCvrp[vehicle].size(); ++vtx){
    /* CVRPLIB_LOG_INFO("the waiting time for vertex " << pathCvrp[vehicle][vtx] << " is " << inst->waitTimes[pathCvrp[vehicle][vtx]]); */
    c+=inst->waitTimes[pathCvrp[vehicle][vtx]];
  }

  float fulfilled_demand = computeDemands(vehicle);
  if (fulfilled_demand > inst->vehicleCapacity){
    pen = (0<fulfilled_demand)?fulfilled_demand:0;
  }
  ret = c + inst->alpha*pen;
  /* printf("Single route cost is %lld\n", ret); */

  return ret;

}

MainType CvrpPathNaive::computeCycleCost() const
{
    MainType ret = 0, wAcum = 1;
    uint32_t n = 0;
    for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){
      n = getNumVertices(vehicle);
      ret += wAcum * inst->rho[pathCvrp[vehicle][n - 1]][pathCvrp[vehicle][0]];
      for (Pos i = n - 1; i > 0; --i) {
          ret += ++wAcum * inst->rho[pathCvrp[vehicle][i - 1]][pathCvrp[vehicle][i]];
      }
    }
    return ret;
}


MainType CvrpPathNaive::improvPos2Opt(
        Pos i,
        Pos j
) const
{
    i = i - 1;
    /// Make things simpler.
    const auto &n = inst->n;
    const auto &d = inst->rho;
    /// Declare auxiliary variables.
    MainType costOrig, costNew, wAcum;
    /// Define auxiliary variables.
    costOrig = computeCost(), costNew = 0, wAcum = 0;
    /// Calculate new cost.
    if (j != n - 1) {
        /// (j + 1) --(...)--> (n - 1)
        for (Pos k = n - 1; k > j + 1; --k)
            costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
        /// (i + 1) ----> (j + 1)
        costNew += (wAcum += 1) * d[path[i + 1]][path[j + 1]];
    }
    /// j --(...)--> (i + 1)
    for (Pos k = i + 1; k < j; ++k)
        costNew += (wAcum += 1) * d[path[k + 1]][path[k]];
    /// i ----> j
    costNew += (wAcum += 1) * d[path[i]][path[j]];
    /// 0 --(...)--> i
    for (Pos k = i; k > 0; --k)
        costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
    /// Return a difference between the new cost and the original cost.
    return (costNew - costOrig);
}

MainType CvrpPathNaive::improvPos2String(
        Pos i,
        Pos j,
        Pos X,
        Pos Y
) const
{
    /// If (j < i) -> swap (i and j) and (X and Y).
    if (j < i) return improvPos2String(j, i, Y, X);
    /// When to return zero.
    if ((!X && (!Y || i == j)) || (!Y && (i == j - X || i == j))) return 0;
    /// Make things simpler.
    auto &n = inst->n;
    auto &d = inst->rho;
    /// Declare auxiliary variables.
    MainType costOrig, costNew, wAcum;
    /// Define auxiliary variables.
    costOrig = computeCost(), costNew = 0, wAcum = 0;
    /// Calculate new cost.
    if (j + Y < n - 1) {
        /// (j + Y + 1) --(...)--> (n - 1)
        for (Pos k = n - 1; k > j + Y + 1; --k)
            costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
        if (X)
            /// (i + X) ----> (j + Y + 1)
            costNew += (wAcum += 1) * d[path[i + X]][path[j + Y + 1]];
    }
    if (X) {
        /// (i + 1) --(...)--> (i + X)
        for (Pos k = i + X; k > i + 1; --k)
            costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
        if (i + X != j)
            /// j ----> (i + 1)
            costNew += (wAcum += 1) * d[path[j]][path[i + 1]];
    } else if (j + Y < n - 1)
        /// j ----> (j + Y + 1)
        costNew += (wAcum += 1) * d[path[j]][path[j + Y + 1]];
    /// (i + X + 1) --(...)--> j
    for (Pos k = j; k > i + X + 1; --k)
        costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
    if (Y) {
        if (i + X != j)
            /// (j + Y) ----> (i + X + 1)
            costNew += (wAcum += 1) * d[path[j + Y]][path[i + X + 1]];
        else
            /// (j + Y) ----> (i + 1)
            costNew += (wAcum += 1) * d[path[j + Y]][path[i + 1]];
        /// (j + 1) --(...)--> (j + Y)
        for (Pos k = j + Y; k > j + 1; --k)
            costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
        /// i ----> (j + 1)
        costNew += (wAcum += 1) * d[path[i]][path[j + 1]];
    } else
        /// i ----> (i + X + 1)
        costNew += (wAcum += 1) * d[path[i]][path[i + X + 1]];
    /// 0 --(...)--> i
    for (Pos k = i; k > 0; --k)
        costNew += (wAcum += 1) * d[path[k - 1]][path[k]];
    /// Return a difference between the new cost and the original cost.
    return (costNew - costOrig);
}

MainType CvrpPathNaive::improvPos2Point(
        Pos i,
        Pos j
) const
{
    return improvPos2String(i, j, 1, 1);
}

MainType CvrpPathNaive::improvPos1Point(
        Pos i,
        Pos j
) const
{
    return improvPos2String(i, j, 0, 1);
}


/// --------------------------------------------------------------------------------------------------------------------
/// CVRP-Solution
/// --------------------------------------------------------------------------------------------------------------------

CvrpPath::CvrpPath(
        const Instance &inst,
        std::string name
)
        : CvrpPathNaive(inst, std::move(name)),
          f(inst.n, 0),
          l(inst.n, 0),
          delta(inst.n, 0)
{
}

bool CvrpPath::isExchangeFeasible(uint16_t vehicle1, uint16_t vehicle2, uint16_t ver1, uint16_t ver2) const{

  bool ret = true;
  uint32_t num_vertices1 = pathCvrp[vehicle1].size();
  if (inst->orders[pathCvrp[vehicle2][ver2]] != inst->assigned_stations[inst->ending_depots[vehicle2]]){
    ret = false;
  }
  uint32_t num_vertices2 = pathCvrp[vehicle2].size();
  if (inst->orders[pathCvrp[vehicle1][ver1]] != inst->assigned_stations[inst->ending_depots[vehicle1]]){
    ret = false;
  }
  /* for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   uint32_t num_vertices = pathCvrp[vehicle].size(); */
  /*   for (uint32_t carried_vertex = 1; carried_vertex < num_vertices; ++carried_vertex){ */
  /*     //TODO: rename orders to target_depot */
  /*     if (inst->orders[pathCvrp[vehicle][carried_vertex]] != (int) (pathCvrp[vehicle][num_vertices-1]/5)){ */
  /*       ret = false; */
  /*     } */
  /*   } */

  /* } */

  return ret;
}

bool CvrpPath::isRelocateFeasible(uint16_t vehicle_from, uint16_t vehicle_to, Ver vertex) const{

  bool ret = true;
  uint32_t depot_id = pathCvrp[vehicle_to].size() - 1;
  if (inst->orders[pathCvrp[vehicle_from][vertex]] != (int) (pathCvrp[vehicle_to][depot_id]/5)){
    ret = false;
  }
  /* for (uint16_t vehicle = 0; vehicle < inst->numVehicles; ++vehicle){ */
  /*   uint32_t num_vertices = pathCvrp[vehicle].size(); */
  /*   for (uint32_t carried_vertex = 1; carried_vertex < num_vertices; ++carried_vertex){ */
  /*     //TODO: rename orders to target_depot */
  /*     if (inst->orders[pathCvrp[vehicle][carried_vertex]] != (int) (pathCvrp[vehicle][num_vertices-1]/5)){ */
  /*       ret = false; */
  /*     } */
  /*   } */

  /* } */

  return ret;
}

void CvrpPath::reinitStructures(unsigned n)
{
    if (n == f.size()) {
        for (unsigned i = 0; i < n; ++i) {
            f[i] = 0;
            l[i] = 0;
            delta[i] = 0;
        }
    } else {
        f = TdppVector(n, 0);
        l = TdppVector(n, 0);
        delta = TdppVector(n, 0);
    }
}

void CvrpPath::updateStructures()
{
    /* const auto &n = inst->n; */
    /* const auto &d = inst->rho; */
    /* /// Update delta, gamma and f. */
    /* for (Pos i = 1; i < n; ++i) { */
    /*     delta[i] = delta[i - 1] + d[path[i - 1]][path[i]]; */
    /*     f[i] = f[i - 1] + delta[i]; */
    /* } */
    /* /// Update l. */
    /* l[n - 1] = delta[n - 1]; */
    /* for (Pos i = n - 2; i > 0; i--) { */
    /*     l[i] = l[i + 1] + delta[i]; */
    /* } */
}

std::unique_ptr<Solution> CvrpPath::makeNewNaive() const
{
    return std::make_unique<CvrpPathNaive>(*inst);
}

bool CvrpPath::canFitIntoCapacity(uint16_t vehicle, Ver vertex) const
{
  bool ret = 0;
  MainType demand = 0;
  demand = computeDemands(vehicle) + inst->demands[vertex];
  ret = (demand <= inst->vehicleCapacity);
  return ret;

}

MainType CvrpPath::computeDemands(uint16_t vehicle) const
{
    MainType ret = 0;
    for (Pos i = 1; i < pathCvrp[vehicle].size(); ++i) {
        ret += inst->demands[pathCvrp[vehicle][i]];
    }
    /* std::cout << "calculating demands\n"; */
    /* if (ret != pathCvrp[vehicle].size() - 2){ */
    /*   CVRPLIB_LOG_WARNING("This does not correspond!"); */
    /*   sleep(1); */
      /* for (int j = 0; j < pathCvrp[vehicle].size(); ++j){ */
      /*   std::cout << "demand for vertex " <<  pathCvrp[vehicle][j] << " is " << inst->demands[pathCvrp[vehicle][j]] << "\n"; */
      /* } */
      /* sleep(5); */
    /* } */

    /* std::cout << "\n"; */
    return ret;
}

MainType CvrpPath::computeDemandsIfAdded(uint16_t vehicle, Ver vertex) const
{
  MainType ret = 0;
  ret = computeDemands(vehicle) + inst->demands[vertex];
  return ret;

}

MainType CvrpPath::improvPos2Opt(
        Pos i,
        Pos j
) const
{
    i = i - 1;
    /// Make things simpler.
    const auto &n = inst->n;
    const auto &d = inst->rho;
    /// Init improvement to 0.
    MainType improvement = 0;
    /// Calculate improvement.
    improvement += 2 * f[i] + (delta[i] + delta[j] + d[path[i]][path[j]]) * (j - i) - 2 * f[n - 1];
    if (j < n - 1) {
        improvement += (d[path[i]][path[j]] + d[path[i + 1]][path[j + 1]] - d[path[i]][path[i + 1]] -
                        d[path[j]][path[j + 1]]) * (n - j - 1) + 2 * l[j + 1];
    }
    /// Return total improvement.
    return improvement;
}

MainType CvrpPath::improvPos2String(
        Pos i,
        Pos j,
        Pos X,
        Pos Y
) const
{
    /// If (j < i) -> swap (i and j) and (X and Y).
    if (j < i) return improvPos2String(j, i, Y, X);
    /// When to return zero.
    if ((!X && (!Y || i == j)) || (!Y && (i == j - X || i == j))) return 0;
    /// Make things simpler.
    const auto &n = inst->n;
    const auto &d = inst->rho;
    /// Declare variables.
    MainType improvement, lambda1, lambda2, lambda3, lambda4;
    /// Init improvement to 0.
    improvement = 0;
    /// The line bellow prevents segmentation fault.
    int j1 = (j + 1 == n) ? 0 : j + 1, jY1 = (j + Y + 1 == n) ? 0 : j + Y + 1;
    /// Define lambdas.
    lambda1 = delta[i] - delta[j1] + d[path[i]][path[j1]];
    lambda2 = delta[j + Y] - delta[i + X + 1] + d[path[j + Y]][path[i + X + 1]];
    lambda3 = delta[j] - delta[i + 1] + d[path[j]][path[i + 1]];
    lambda4 = delta[i + X] - delta[jY1] + d[path[i + X]][path[jY1]];
    /// Calculate improvement.
    if (Y == 0) lambda1 = 0, lambda2 = delta[i] - delta[i + X + 1] + d[path[i]][path[i + X + 1]];
    else improvement += lambda1 * Y;
    if (j == i + X) lambda2 = 0, lambda3 = delta[j + Y] - delta[i + 1] + d[path[j + Y]][path[i + 1]];
    else improvement += (lambda1 + lambda2) * (j - i - X);
    if (X == 0) lambda3 = 0, lambda4 = delta[j] - delta[jY1] + d[path[j]][path[jY1]];
    else improvement += (lambda1 + lambda2 + lambda3) * (i + X - i);
    improvement += (!jY1) ? 0 : (lambda1 + lambda2 + lambda3 + lambda4) * (n - j - Y - 1);
    /// Return total improvement.
    return improvement;
}

MainType CvrpPath::improvPos2Point(
        Pos i,
        Pos j
) const
{
    /// If (j < i) -> swap (i and j).
    if (j < i) return improvPos2Point(j, i);
    /// Make things simpler.
    const auto &n = inst->n;
    const auto &d = inst->rho;
    /// Declare variables.
    MainType lambda1, lambda2, lambda3, lambda4;
    /// Define lambdas.
    lambda1 = delta[i] - delta[j + 1] + d[path[i]][path[j + 1]];
    if (j == i + 1) {
        lambda2 = 0;
        lambda3 = lambda1 + delta[j + 1] - delta[i + 1] + d[path[j + 1]][path[i + 1]];
    } else {
        lambda2 = lambda1 + delta[j + 1] - delta[i + 2] + d[path[j + 1]][path[i + 2]];
        lambda3 = lambda2 + delta[j] - delta[i + 1] + d[path[j]][path[i + 1]];
    }
    if (j + 2 == n) {
        lambda4 = 0;
    } else {
        lambda4 = lambda3 + delta[i + 1] - delta[j + 2] + d[path[i + 1]][path[j + 2]];
    }
    /// Return total improvement.
    return lambda1 + lambda2 * (j - i - 1) + lambda3 + lambda4 * (n - j - 2);
}

MainType CvrpPath::improvPos1Point(
        Pos i,
        Pos j
) const
{
    /// Make things simpler.
    const auto &n = inst->n;
    const auto &d = inst->rho;
    if (i < j) {
        /// Declare variables.
        MainType lambda1, lambda2, lambda3;
        /// The line bellow prevents segmentation fault.
        int j1 = (j + 1 == n) ? 0 : j + 1, j2 = (j + 2 == n) ? 0 : j + 2;
        /// Define lambdas.
        lambda1 = delta[i] - delta[j1] + d[path[i]][path[j1]];
        lambda2 = lambda1 + delta[j + 1] - delta[i + 1] + d[path[j + 1]][path[i + 1]];
        lambda3 = lambda2 + delta[j] - delta[j2] + d[path[j]][path[j2]];
        /// Return total improvement.
        return lambda1 + lambda2 * (j - i) + ((!j2) ? 0 : lambda3 * (n - j - 2));
    } else if (j + 1 < i) {
        /// If (j + 1 < i) -> swap (i and j) and (0 and 1).
        std::swap(i, j);
        /// Declare variables.
        MainType lambda1, lambda2, lambda3;
        /// The line bellow prevents segmentation fault.
        int j1 = (j + 1 == n) ? 0 : j + 1;
        /// Define lambdas.
        lambda1 = delta[i] - delta[i + 2] + d[path[i]][path[i + 2]];
        lambda2 = lambda1 + delta[j] - delta[i + 1] + d[path[j]][path[i + 1]];
        lambda3 = lambda2 + delta[i + 1] - delta[j1] + d[path[i + 1]][path[j1]];
        /// Return total improvement.
        return lambda1 * (j - i - 1) + lambda2 + ((!j1) ? 0 : lambda3 * (n - j - 1));
    } else return 0;
}
