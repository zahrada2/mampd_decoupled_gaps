#include "vrplib.hpp"
#include "unistd.h"

using namespace imr::vrp;

//TODO: implement logger

std::string VRPProblemLoader::KeywordsStr[] = {
        "NAME",
        "TYPE",
        "COMMENT",
        "DIMENSION",
        "CAPACITY",
        "VEHICLES",
        "EDGE_WEIGHT_TYPE",
        "EDGE_WEIGHT_FORMAT",
        "EDGE_DATA_FORMAT",
        "NODE_COORD_TYPE",
        "DISPLAY_DATA_TYPE",
        "N/A"
};

/// --------------------------------------------------------------------------
std::string VRPProblemLoader::DataSectionStr[] = {
        "NODE_COORD_SECTION",
        "DEPOT_SECTION",
        "STARTING_DEPOT_SECTION",
        "ENDING_DEPOT_SECTION",
        "DEMAND_SECTION",
        "EDGE_DATA_SECTION",
        "FIXED_EDGES_SECTION",
        "DISPLAY_DATA_SECTION",
        "TOUR_SECTION",
        "EDGE_WEIGHT_SECTION",
        "WAITING_TIME_SECTION",
        "N/A"
};

std::string VRPProblemLoader::ProblemTypeStr[] = {
        "VRP",
        "CVRP",
        "TSP",
        "N/A"
};



VRPProblemLoader::VRPProblemLoader(){
  specified_starting_depots = false;
  specified_ending_depots = false;
  std::cout << "VRP Problem loader initialized\n";
}

bool VRPProblemLoader::loadFile(std::string filename){

  if(!detectType(filename)){
    std::cout << "could not open file " << filename << "\n";
    return false;
  }
  std::ifstream fin;
  fin.open(filename);
  if (!fin){
    std::cout << "could not open file " << filename << "\n";
    return false;
  }
  Problem new_problem;
  std::string delimiter = ": ";
  std::string line;
  std::vector<std::string> line_split;

  while (!fin.eof()){
    std::getline(fin, line);
    line_split = split(line, delimiter);
    std::string type = line_split[0];
 
    std::cout << "Loaded field type: " << type << "\n";
    std::string value;
    /* DataSection::Type data_section_type; */
    Spec::TSpec spec;
    if (isDataSection(type)){
      std::cout << "got data section\n";
      parseDataSection(fin, new_problem);
      // END because there won't be any specs remaining
      break;

    } else {
      // TODO: Separate into another function, then switch to dintataSection parsing function
      if (getSpecification(type, spec)){
        value = line_split[1];
        /* std::cout << "got spec\n"; */
        switch(spec){
          // TODO: create a problem struct, populate it here
          // TODO: replace with a friend operator?
          case Spec::NAME:
            /* std::cout << "found name spec\n"; */
            value = line_split[1];
            /* std::cout << value << "\n"; */
            break;

          case Spec::TYPE:
            {
              /* std::cout << "found type\n"; */
              bool known_problem = false;
              /* std::cout << "starting to look\n"; */
              for (int i = 0; i < ProblemType::NUMBER; ++i){
                if (ProblemTypeStr[i] == value) {
                    /* type = (DataSection::Type) i; */
                    new_problem.problem_type = (ProblemType) i;
                    /* std::cout << "found!\n"; */
                    known_problem = true;
                    break;
                }
              }
              if (!known_problem){
                std::cout << "Cannot parse this type of problem! '" << value << "'\n";
                return 0;
              }
              /* std::cout << newProblem.problem_type << "\n"; */
              break;
            }

          case Spec::COMMENT:
            // Ignore comments
            break;

          case Spec::DIMENSION:
            new_problem.dimension = stoi(value);
            break;
          
          case Spec::VEHICLES:
            new_problem.numVehicles = stoi(value);
            break;

          case Spec::CAPACITY:
            new_problem.capacity = stoi(value);
            break;

            // UNUSED SPECS
          default:
            /* std::cout << "Unknown spec found! This was supposed to be checked earlier. How did you even get here?\n"; */
            break;
        }
      } else { 
        std::cout << "Unknown data field found!\n";
      }
    }
  }
  problem_data = new_problem;

  return true;
}


bool VRPProblemLoader::isDataSection(std::string name){
  bool ret = false;
  for (int i = 0; i < DataSection::NUMBER; i++) {
      if (DataSectionStr[i] == name) {
          ret = true;
          break;
      }
  }
  return ret;
}

bool VRPProblemLoader::getDataSection(std::string name, DataSection::Type &type) {
  bool ret = false;
  for (int i = 0; i < DataSection::NUMBER; i++) {
      if (DataSectionStr[i] == name) {
          type = (DataSection::Type) i;
          ret = true;
          break;
      }
  }
  if (!ret) {
    std::cout << "Unknown VRPLIB specification '" + name + "'\n";
  }
  return ret;
}

bool VRPProblemLoader::parseDataSection(std::ifstream &fin, Problem &new_problem){
  std::string delimiter = " ";
  std::string line;
  std::vector<std::string> line_split;
  // TODO: implement switching of sections
  DataSection::Type current_section = DataSection::NODE_COORD_SECTION; 
  while(!fin.eof()){
    /* std::cout << "reading data section ...\n"; */
    std::getline(fin, line);
    line_split = split(line, delimiter);
    if (line_split[0] == "EOF"){
      return true;
    } else if (line_split[0] == "DEPOT_SECTION" || line_split[0] == "START_DEPOT_SECTION"){
      current_section = DataSection::STARTING_DEPOT_SECTION;
      std::cout << "Switching to: " << line_split[0] << "\n";
    } else if (line_split[0] == "END_DEPOT_SECTION"){
      //TODO: implement
      current_section = DataSection::ENDING_DEPOT_SECTION;
      std::cout << "Switching to: " <<  line_split[0] << "\n";
    } else if (line_split[0] == "DEMAND_SECTION"){
      std::cout << "Switching to: " <<  line_split[0] << "\n";
      //TODO: implement
    } else if (line_split[0] == "WAITING_TIME_SECTION"){
      current_section = DataSection::WAITING_TIME_SECTION;
      std::cout << "Switching to: " <<  line_split[0] << "\n";
      //TODO: implement
    } else {
      /* std::cout << line_split[0] << " " << line_split[1] << " " << line_split[2] << "\n"; */
      switch(current_section){
        // TODO: replace with an operator?
        case DataSection::NODE_COORD_SECTION:
          if ((line_split.size() == 3)){
            VRPNode new_node(stoi(line_split[1]), stoi(line_split[2]));
            new_problem.goals.push_back(new_node);
          } else {
            std::cout << "Not enough integers to form a vertex! Skipping ...\n";
            std::cout << line << "\n";
          }
          break;

        case DataSection::DEPOT_SECTION: 
          if (stoi(line) == -1){
            // end of depot section
            break;
          } else if (stoi(line) > 0){
            new_problem.starting_depots.push_back(stoi(line)-1);
          } else {
            std::cout << "Invalid depot ID! Skipping ...\n";
          }
          break;

        case DataSection::STARTING_DEPOT_SECTION:
          if (stoi(line) == -1){
            // end of depot section
            break;
          } if (stoi(line) > 0){
            new_problem.starting_depots.push_back(stoi(line)-1);
          } else {
            std::cout << "Invalid depot ID! Skipping ...\n";
          }
          break;

        case DataSection::ENDING_DEPOT_SECTION:
          if (stoi(line) != -1){
            new_problem.ending_depots.push_back(stoi(line)-1);
          } else {
            // end of ending depot section
          }
          break;

        case DataSection::WAITING_TIME_SECTION:
          new_problem.waiting_times.push_back(stoi(line));
          break;

        default:
          break;
      }

        /* std::cout << "[loader] created new node: " <<  stoi(line_split[1]) << " " << stoi(line_split[2]) << "\n"; */ 
        /* << line_split; */
        /* line >> new_node; */
        /* std::pair<int, int> new_vertex = std::make_pair(stoi(line_split[1]), stoi(line_split[2])); */
        /* new_problem.goals.push_back(new_vertex); */
        /* std::cout << new_problem.goals.size() << "\n"; */
        /* std::cout << new_problem.goals[0].getLocation().first << " " << new_problem.goals[0].getLocation().second; */
    }
  }
  /* std::cout << line; */
  return true;
}


bool VRPProblemLoader::getSpecification(std::string name, Spec::TSpec &spec) {
  bool ret = false;
  for (int i = 0; i < Spec::NUMBER; i++) {
      if (KeywordsStr[i] == name) {
          spec = (Spec::TSpec) i;
          ret = true;
          break;
      }
  }
  if (!ret) {
    std::cout << "Unknown VRPLIB specification '" + name + "'\n";
  }
  return ret;
}


// TODO: Move to toolkit
std::vector<std::string> VRPProblemLoader::split (std::string s, std::string delimiter) {
      size_t pos_start = 0, pos_end, delim_len = delimiter.length();
      std::string token;
      std::vector<std::string> res;

      while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
      }

      res.push_back (s.substr (pos_start));
      return res;
}

Problem VRPProblemLoader::getProblemData(){
  return problem_data;
}

bool VRPProblemLoader::detectType(std::string filename){
  std::ifstream fin;
  fin.open(filename);
  if (!fin){
    std::cout << "could not open file " << filename << "\n";
    return false;
  }

  std::string line;
  while (!fin.eof()){
    std::getline(fin, line);
    if (line == "DEPOT_SECTION" || line == "STARTING_DEPOT_SECTION"){
      specified_starting_depots = true;
      std::cout << "Detected starting depot section\n";
    } else if (line == "ENDING_DEPOT_SECTION"){
      specified_ending_depots = true;
      std::cout << "Detected ending depot section\n";
    }
  }
  return true;
}
