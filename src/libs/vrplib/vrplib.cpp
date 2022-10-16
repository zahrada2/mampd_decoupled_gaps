/*
 * File name: vrplib.cpp
 * Date:      01/06/2021
 * Author:    David Zahradka
 */

#include "vrplib.hpp"
/* #include "logging.h" */
/* #include "stringconversions.h" */

using namespace imr::vrp;

VRPLib::VRPLib() {
  /* *loader = VRPProblemLoader(); */
  VRPProblemLoader *new_loader = new VRPProblemLoader();
  loader = new_loader;
  std::cout << "vrplib initialized\n";
};

VRPLib::~VRPLib() {
}


bool VRPLib::loadProblem(std::string filePath){
  return(loader->loadFile(filePath));
}

Problem VRPLib::getProblem(){
  return loader->getProblemData();
}

/* end of tsplib.cc */
