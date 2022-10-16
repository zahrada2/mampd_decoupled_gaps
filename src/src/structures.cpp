/**
 * File:    structures.cpp
 *
 * Date:    17/08/2021 
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "structures.hpp"

using namespace cvrplib;

Goals::Goals()
        : c(false), t(false), cost(-1), time(-1.0), timer()
{

}
        
bool Instance::setMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface)
{
  mapfSolver = mapf_interface;
  return true;
}

bool Instance::setSecondaryMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface)
{
  secondary_mapf_solver = mapf_interface;
  return true;
}

const bool &Goals::isGoalCostSet() const
{
    return c;
}

const bool &Goals::isGoalTimeSet() const
{
    return t;
}

void Goals::setGoalCost(MainType cost)
{
    c = true;
    Goals::cost = cost;
}

void Goals::setGoalTime(double time)
{
    t = true;
    Goals::time = time;
}

void Goals::unsetGoalCost()
{
    c = false;
}

void Goals::unsetGoalTime()
{
    t = false;
}

void Goals::resetTimer()
{
    timer.restart();
}

bool Goals::isGoalCostMet(MainType &cost) const
{
    return c && (cost <= Goals::cost);
}

bool Goals::isGoalTimeMet() const
{
    return t && (time <= timer.getTimeInSeconds());
}

const MainType &Goals::getGoalCost() const
{
    return cost;
}

const double &Goals::getGoalTime() const
{
    return time;
}

Instance::Instance(unsigned n)
        : n(n),
          sumWeights(0),
          w(n, 0),
          theta1(n, 0),
          rho(n, TdppVector(n, 0)),
          theta3(n, TdppMatrix(n, TdppVector(n, 0))),
          neighborsDist(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1)),
          neighborsRatio(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1))
{

}

/* Instance::Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, */ 
/*     uint16_t numVehicles, uint16_t vehicleCapacity, bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::vector<uint16_t> orders, std::vector<int> assigned_stations) */
Instance::Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, uint16_t numVehicles, uint16_t vehicleCapacity, bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::string problem_name, int numRun)
        : n(n),
          starting_depots(starting_depots),
          ending_depots(ending_depots),
          demands(demands),
          numVehicles(numVehicles),
          useIntegratedMapf(useIntegratedMapf),
          mapfLevelOps(mapfLevelOps),
          alpha(1),
          sumWeights(0),
          w(n, 0),
          theta1(n, 0),
          rho(n, TdppVector(n, 0)),
          theta3(n, TdppMatrix(n, TdppVector(n, 0))),
          neighborsDist(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1)),
          neighborsRatio(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1)),
          vehicleCapacity(vehicleCapacity),
          waitTimes(waitTimes),
          /* orders(orders), */
          /* assigned_stations(assigned_stations), */
          problem_name(problem_name),
          numRun(numRun)
{

}

Instance::Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, uint16_t numVehicles, uint16_t vehicleCapacity, bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::string problem_name, int numRun, bool loadSolution, bool useTtd, bool useFa, std::vector<double> closestDepotDistances)
        : n(n),
          starting_depots(starting_depots),
          ending_depots(ending_depots),
          demands(demands),
          numVehicles(numVehicles),
          useIntegratedMapf(useIntegratedMapf),
          mapfLevelOps(mapfLevelOps),
          alpha(1),
          sumWeights(0),
          w(n, 0),
          theta1(n, 0),
          rho(n, TdppVector(n, 0)),
          theta3(n, TdppMatrix(n, TdppVector(n, 0))),
          neighborsDist(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1)),
          neighborsRatio(n, std::vector<int>(std::max(static_cast<int>(n) - 1, 0), -1)),
          vehicleCapacity(vehicleCapacity),
          waitTimes(waitTimes),
          /* orders(orders), */
          /* assigned_stations(assigned_stations), */
          problem_name(problem_name),
          numRun(numRun),
          loadSolution(loadSolution),
          useTtd(useTtd),
          useFa(useFa),
          closestDepotDistances(closestDepotDistances)

{

}


void Instance::init()
{
    sortNeighborsDistance();
    sortNeighborsRatio();
    computeSumWeights();
}

void Instance::setProblemName(std::string problem){
  problem_name = problem;
}

void Instance::setNumRun(int numRuns){
  numRun = numRuns;
}

void Instance::sortNeighborsDistance()
{
    for (unsigned int i = 0; i < n; i++) {
        std::vector<pti> aux(n - 1);
        int cnt = 0;
        for (unsigned int j = 0; j < n; j++) {
            if (i == j) continue;
            aux[cnt] = pti(rho[i][j], j);
            cnt++;
        }
        std::sort(aux.begin(), aux.end());
        for (unsigned int j = 0; j < n - 1; j++) {
            neighborsDist[i][j] = aux[j].second;
        }
    }
}

void Instance::sortNeighborsRatio()
{
    for (unsigned int i = 0; i < n; i++) {
        std::vector<pti> aux(n - 1);
        int cnt = 0;
        for (unsigned int j = 0; j < n; j++) {
            if (i == j) continue;
            aux[cnt] = pti(rho[i][j] / (w[j] + 1), j);
            cnt++;
        }
        std::sort(aux.begin(), aux.end());
        for (unsigned int j = 0; j < n - 1; j++) {
            neighborsRatio[i][j] = aux[j].second;
        }
    }
}

void Instance::computeSumWeights()
{
    sumWeights = 0;
    for (unsigned int i = 0; i < n; i++) {
        sumWeights += w[i];
    }
}
