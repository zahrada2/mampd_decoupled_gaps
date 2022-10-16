/**
 * File:    MAPF_interface.cpp
 *
 * Date:    09/02/2021
 * Author:  David Zahradka 
 * E-mail:  zahrada2@fel.cvut.cz
 *
 */

#include "MAPF_interface.hpp"

#include <limits>

using namespace cvrplib;

/// --------------------------------------------------------------------------------------------------------------------
/// Getters / setters
/// --------------------------------------------------------------------------------------------------------------------

void MAPF_interface::initializeMAPF()
{
  CVRPLIB_LOG_INFO("MAPF interface constructor successfuly called\n");
}

MAPF_interface::MAPF_interface(int _seed, bool _useTTD, bool _blockGoals)
{
  seed = _seed;
  useTTD =_useTTD;
  blockGoals = _blockGoals;
  CVRPLIB_LOG_INFO("MAPF interface object successfuly created\n");
}

bool MAPF_interface::setMap(boost::shared_ptr<MapData> map_){

   this->map = map_;

  CVRPLIB_LOG_INFO("Vertex map successfuly loaded. Currently has " << map->getMapHeight()*map->getMapWidth() << " locations.\n");
  return true;
}

/// --------------------------------------------------------------------------------------------------------
/// Connection function
/// --------------------------------------------------------------------------------------------------------
int MAPF_interface::getAg(std::vector<int> goals_in_path, MAPF_solution solution){
  return 0;
}

MAPF_solution MAPF_interface::computeCost(MAPF_input input){
  MAPF_solution solution;
  /// Do cost computation
  return solution;
}

MainType MAPF_interface::estimateCost(MAPF_input input){
    MainType ret = -1;
    /// Do cost computation
    return ret;
}
void MAPF_interface::saveSolutionIntoXML(std::string fileName, MAPF_solution solution)
{
    return;
}

void MAPF_interface::saveInstanceIntoXML(std::string fileName, MAPF_input& input, MapData& map_data)
{
    return;
}

void MAPF_interface::saveSolutionMampdIR(std::string fileName, MAPF_solution solution,
                                int numVehicles, std::string map_name, std::string instance_name, std::vector<std::pair<int, int>> vertex_locations, double vrp_cost, std::vector<std::vector<int>> assignments, MAPF_input mapf_task){
    return;
}

void MAPF_interface::saveSolutionMapfIR(std::string fileName, MAPF_solution solution, std::vector<uint16_t> starting_depots, std::vector<uint16_t> ending_depots,
        int numVehicles, std::string map_name, std::vector<std::pair<int, int>>, int cost_lower_bound, int final_cost, std::string method)
/* void MAPF_interface::saveSolutionMapfIR(std::string fileName, MAPF_solution solution) */
{
    return;
}

bool MAPF_interface::computeRelocateCost(int POI_ID, int newAgentID, int newPlace, int oldAgentID, int oldPlace)
{
    /// Do cost computation
    return false;
}

bool MAPF_interface::computeTwoOpt(int agentID, int startPlace, int endPlace)
{
    /// Do cost computation
    return false;
}

bool MAPF_interface::computeExchange(int agentA_ID, int agentB_ID, int placeA, int placeB)
{
    /// Do cost computation
    return false;
}

bool MAPF_interface::computeThreeOpt(int agentID, int placeI, int placeJ, int placeK)
{
    /// Do cost computation
    return false;
}

bool MAPF_interface::lastInstanceSolutionFound()
{
    return true;
}

MAPF_solution MAPF_interface::getBestSolution()
{
    MAPF_solution solution;
    return solution;
}

void MAPF_interface::setInitSolution(MAPF_solution solution, MAPF_input input)
{
    ///store initial MAPF- and VRP-solutions
    return;
}


/// --------------------------------------------------------------------------------------------------------------------
/// Protected constructor
/// --------------------------------------------------------------------------------------------------------------------


/* MAPF_interface::MAPF_interface( */
/*         const cvrplib::Instance &inst, */
/*         std::string name */
/* ) */
        /* : inst(&inst), */
        /*   path(inst.n, INVALID_PATH_FLAG), */
        /*   pathInv(inst.n, std::numeric_limits<Pos>::max()), */
        /*   name(std::move(name)) */

