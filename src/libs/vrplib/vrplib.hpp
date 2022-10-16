/*
 * File name: vrplib.h
 * Date:      01/06/2021 
 * Author:    David Zahradka
 */

#ifndef __VRPLIB_H__
#define __VRPLIB_H__

#include <vector>
#include <string>
#include <fstream>

#include "vrplib_loader.hpp"

namespace imr {
    namespace vrp {

        /// --------------------------------------------------------------------------
        /// @brief  TSPLib loader wrapper
        /// --------------------------------------------------------------------------
        class VRPLib {
            /* typedef std::vector<CTSPProblemLoader *> Loaders; */
            /* Loaders loaders; */
        public:

            VRPLib();

            virtual ~VRPLib();

            bool loadProblem(std::string filePath);

            Problem getProblem();

        private:
            VRPProblemLoader *loader;
            /* Problem problemData; */ 


        };

    } //end namespace
} //end namespace
#endif

/* end of tsplib.h */
