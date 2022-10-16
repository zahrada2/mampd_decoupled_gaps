/**
 * File:    Solver.cpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#include "Solver.hpp"

#include "solutions_impl.hpp"
#include "methods.hpp"

using namespace cvrplib;
using namespace cvrplib::solutions::impl;

Solver::Solver()
        : goals(),
          newData(false),
          newMode(true),
          scale(1),
          mode(Mode::undefined),
          method(Method::undefined),
          output(),
          instance(nullptr)
{
    addMethod(Method::fmivns, "fmivns");
    addMethod(Method::simpleTwoPhase, "simpleTwoPhase");
    addMethod(Method::simplified_vns, "simplified_vns");
}
        
bool Solver::setMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface)
{
  mapf_solver = mapf_interface;
  mapf_solver->initializeMAPF();
  return true;
}

bool Solver::setSecondaryMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface)
{
  secondary_mapf_solver = mapf_interface;
  secondary_mapf_solver->initializeMAPF();
  return true;
}

bool Solver::setMethod(Method method)
{
    auto m = method2string.find(method);
    if (m != method2string.end()) {
        Solver::method = m->first;
        return true;
    } else {
        return false;
    }
}

bool Solver::setMethod(std::string method)
{
    auto m = string2method.find(toLower(move(method)));
    if (m != string2method.end()) {
        Solver::method = m->second;
        return true;
    } else {
        return false;
    }
}

std::string Solver::getMethodName()
{
    auto m = method2string.find(method);
    if (m != method2string.end()) {
        return m->second;
    } else {
        return "N/A";
    }
}

void Solver::setData(in::CvrpStructures &dataCVRP)
{
    UnsignedVector demands = dataCVRP.q;
    auto n = dataCVRP.d.size();
    if (instance == nullptr || instance->n != n) {
        /* instance = std::make_unique<Instance>(n, dataCVRP.starting_depots, dataCVRP.ending_depots, demands, dataCVRP.numVehicles, dataCVRP.vehicleCapacity, dataCVRP.useIntegratedMapf, dataCVRP.mapfLevelOps, dataCVRP.waitTimes, dataCVRP.orders, dataCVRP.assigned_stations); */
        instance = std::make_unique<Instance>(n, dataCVRP.starting_depots, dataCVRP.ending_depots, demands, dataCVRP.numVehicles, dataCVRP.vehicleCapacity, dataCVRP.useIntegratedMapf, dataCVRP.mapfLevelOps, dataCVRP.waitTimes, dataCVRP.problemName, dataCVRP.numRun, dataCVRP.loadSolution, dataCVRP.useTtd, dataCVRP.useFa, dataCVRP.closestDepotDistances);
        /* instance->setProblemName(dataCVRP.problemName); */
        /* instance->setNumRun(dataCVRP.numRun); */
        instance->setMAPFSolver(mapf_solver);
        instance->setSecondaryMAPFSolver(secondary_mapf_solver);
        instance->numVehicles = dataCVRP.numVehicles;
    }
    for (unsigned i = 0; i < n; ++i) {
        instance->w[i] = 1;
        for (unsigned j = 0; j < n; ++j) {
            instance->rho[i][j] = static_cast<MainType>(dataCVRP.d[i][j] * static_cast<double>(scale));
        }
    }
    instance->init();
    if (mode != Mode::cvrp) {
        mode = Mode::cvrp;
        newMode = true;
    }
    newData = true;
}

void Solver::setData(in::TdpStructures &dataTDP)
{
    auto n = dataTDP.d.size();
    if (instance == nullptr || instance->n != n) {
        instance = std::make_unique<Instance>(n);
    }
    for (unsigned i = 0; i < n; ++i) {
        instance->w[i] = 1;
        for (unsigned j = 0; j < n; ++j) {
            instance->rho[i][j] = static_cast<MainType>(dataTDP.d[i][j] * static_cast<double>(scale));
        }
    }
    instance->init();
    if (mode != Mode::tdp) {
        mode = Mode::tdp;
        newMode = true;
    }
    newData = true;
}

void Solver::setData(in::GspStructures &dataGSP)
{
    auto n = dataGSP.d.size();
    if (instance == nullptr || instance->n != n) {
        instance = std::make_unique<Instance>(n);
    }
    for (unsigned i = 0; i < n; ++i) {
        instance->w[i] = static_cast<MainType>(dataGSP.w[i] * static_cast<double>(scale));
        for (unsigned j = 0; j < n; ++j) {
            instance->rho[i][j] = static_cast<MainType>(dataGSP.d[i][j] * static_cast<double>(scale));
        }
    }
    instance->init();
    if (mode != Mode::gsp) {
        mode = Mode::gsp;
        newMode = true;
    }
    newData = true;
}

void Solver::setData(in::AgspStructures &dataAGSP)
{
    auto n = dataAGSP.rho.size();
    if (instance == nullptr || instance->n != n) {
        instance = std::make_unique<Instance>(n);
    }
    for (unsigned i = 0; i < n; ++i) {
        instance->w[i] = static_cast<MainType>(dataAGSP.w[i] * static_cast<double>(scale));
        instance->theta1[i] = static_cast<MainType>(dataAGSP.theta1[i] * static_cast<double>(scale));
        for (unsigned j = 0; j < n; ++j) {
            instance->rho[i][j] = static_cast<MainType>(dataAGSP.rho[i][j] * static_cast<double>(scale));
            for (unsigned k = 0; k < n; ++k) {
                instance->theta3[i][j][k] =
                        static_cast<MainType>(dataAGSP.theta3[i][j][k] * static_cast<double>(scale));
            }
        }
    }
    instance->init();
    if (mode != Mode::agsp) {
        mode = Mode::agsp;
        newMode = true;
    }
    newData = true;
}

const out::Output *Solver::solve(
        const unsigned itersOuter,
        const unsigned itersInner,
        const unsigned operatorSeq,
        const unsigned shakersSeq,
        const bool itersInnerFixed,
        const bool initialGreedy
)
{

    /// Check if the instance exists.
    if (instance == nullptr) {
        CVRPLIB_LOG_WARNING("Instance is nullptr! Probably because data was not set.");
        return nullptr;
    }

    /// Check if the method was set.
    if (method == Method::undefined || method == Method::count) {
        CVRPLIB_LOG_WARNING("Method is not properly set (Method::undefined or Method::count)!");
        return nullptr;
    }

    /// If the mode have changed, then change implementation of the solutions.
    if (newMode) {
        switch (mode) {
            case Mode::tdp : {
                CVRPLIB_LOG_WARNING("tdp not supported in this version");                 
                return nullptr;
            }
            case Mode::gsp : {
                CVRPLIB_LOG_WARNING("gsp not supported in this version");                 
                return nullptr;
            }
            case Mode::cvrp : {
                pathRes1 = std::make_unique<CvrpPath>(*instance);
                pathAux1 = std::make_unique<CvrpPath>(*instance);
                pathAux2 = std::make_unique<CvrpPath>(*instance);
                pathAux3 = std::make_unique<CvrpPath>(*instance);
                break;
            }
            case Mode::agsp : {
                CVRPLIB_LOG_WARNING("Mode AGSP is not implemented yet.");
                return nullptr;
            }
            default : {
                CVRPLIB_LOG_WARNING("Solver's mode is undefined! Probably because data was not set.");
                return nullptr;
            }
        }
        newMode = false;
        newData = false;
    }

    /// If the data have changed (but not the mode), then change solutions's instances.
    if (newData) {
        pathRes1->setInstance(*instance);
        pathAux1->setInstance(*instance);
        pathAux2->setInstance(*instance);
        pathAux3->setInstance(*instance);
        newData = false;
    }


    MainType cost;
    methods::References refs{cost, goals, *pathRes1, *pathAux1, *pathAux2, *pathAux3};
    SimpleClock clock1{};
    if (goals.isGoalTimeSet()) {
        goals.resetTimer();
    }
    switch (method) {
        case Method::simpleTwoPhase: {
            methods::simpleTwoPhase(refs, itersOuter);
            break;
        }
        case Method::fmivns: {
            methods::fmivns(refs, itersOuter);
            break;
        }
        case Method::simplified_vns: {
            methods::simplified_vns(refs, itersOuter);
            break;
        }
        default: {
            return nullptr;
        }
    }
    output.time = clock1.getTimeInSeconds();
    output.cost = cost;
    return &output;
}

const solutions::Solution &Solver::getPath()
{
    return *pathRes1;
}

void Solver::computeSum(
        const DoubleVector &weights,
        double &sum
)
{
    sum = 0.0;
    for (auto &w : weights) sum += w;
}

void Solver::normalizeWeights(DoubleVector &weights)
{
    double sum;
    Solver::computeSum(weights, sum);
    for (auto &w : weights) w /= sum;
}

void Solver::getMax(
        const DoubleMatrix &distances,
        double &max
)
{
    max = 0.0;
    for (auto &row : distances) for (auto &d : row) if (d > max) max = d;
}

void Solver::divideDistancesByMaxDistance(DoubleMatrix &distances)
{
    double max;
    Solver::getMax(distances, max);
    for (auto &row : distances) for (auto &d : row) d /= max;
}

Mode Solver::getMode() const
{
    return mode;
}

void Solver::setMode(Mode mode)
{
    if (Solver::mode != mode) {
        Solver::mode = mode;
        newMode = true;
    }
}

long Solver::getScale() const
{
    return scale;
}

void Solver::setScale(long scale)
{
    Solver::scale = scale;
}

void Solver::addMethod(
        Method method,
        std::string name
)
{
    method2string.emplace(method, toLower(name));
    string2method.emplace(toLower(name), method);
}
