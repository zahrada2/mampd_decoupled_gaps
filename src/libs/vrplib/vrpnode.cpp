#include "vrpnode.hpp"

using namespace imr::vrp;

VRPNode::VRPNode(){
  _initialized = false;
  _x = -1;
  _y = -1;
}

VRPNode::VRPNode(int x, int y){
  _x = x;
  _y = y;
  /* std::cout << "created new node with params: " << _x << " " << _y << "\n"; */
  _initialized = true;
}

VRPNode::VRPNode(int x, int y, int demand){
  _initialized = true;
  _x = x;
  _y = y;
  _demand = demand;
}



std::pair<int, int> VRPNode::getLocation(){
  return std::make_pair(this->_x, this->_y);
}

void VRPNode::assignLocation(int x, int y){
  _x = x;
  _y = y;
  _initialized = true;
}

void VRPNode::assignDemand(int demand){
  _demand = demand;
}

float VRPNode::distanceSqrt(VRPNode destination_node){

   float dx = _x - destination_node._x;
   /* std::cout << "start x: " << _x << " destination x: " << destination_node._x << "\n"; */
   float dy = _y - destination_node._y;
   return sqrt(dx*dx + dy*dy);
}

