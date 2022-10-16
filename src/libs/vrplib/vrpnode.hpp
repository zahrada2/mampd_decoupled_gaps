#include <iostream>
#include <cmath>
#include <utility>

namespace imr {
    namespace vrp {


        class VRPNode{
          public:

            // constructors
            VRPNode();
            
            VRPNode(int x, int y);
            
            VRPNode(int x, int y, int demand);

            // setters
            void assignLocation(int x, int y);

            void assignDemand(int demand);
            
            // getters

            std::pair<int, int> getLocation();

            float distanceSqrt(VRPNode destination_node);
         
            inline std::ostream &operator<< (std::ostream &os) {
              os << this->_x << " " << this->_y;
              return os;
            }

            inline std::istream &operator>> (std::istream &is) {
              this->_initialized = true;
              return (is >> this->_i >> this->_x >> this->_y);
            }


          private:
            int _x;
            int _y;
            int _i;
            int _demand;
            bool _initialized;

        };
    }
}

