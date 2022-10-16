/**
 * File:    solutions_impl.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_SOLUTIONS_IMPL_HPP
#define CVRP_SOLUTIONS_IMPL_HPP

#include "structures.hpp"
#include "Solution.hpp"

namespace cvrplib {

    namespace solutions {

        namespace impl {


            /// --------------------------------------------------------------------------------------------------------
            /// Cvrp-Solution-naive
            /// --------------------------------------------------------------------------------------------------------

            class CvrpPathNaive : public Solution {

            public:

                explicit CvrpPathNaive(
                        const Instance &inst,
                        std::string name = "CVRP-Solution-naive"
                );

                void reinitStructures(unsigned n) override;

                void updateStructures() override;

                std::unique_ptr<Solution> makeNewNaive() const override;
                
                MainType computeDemands(uint16_t vehicle) const override;
                
                MainType computeDemandsIfAdded(uint16_t vehicle, Ver vertex) const override;

                bool canFitIntoCapacity(uint16_t vehicle, Ver vertex) const override;

                MainType computeTSPCost() const override;

                MainType computeTDPCost() const override;

                void precomputeTTDDiscount() override;

                MainType computeTTD() const override;

                MainType computeCost() const override;
                
                MainType computeCostSelected(bool use_ttd) const override;
                
                bool isFeasible() const;
                
                bool isRelocateFeasible(uint16_t vehicle_from, uint16_t vehicle_to, Ver vertex) const;
                
                bool isExchangeFeasible(uint16_t vehicle1, uint16_t vehicle2, uint16_t ver1, uint16_t ver2) const;

                MainType computeSingleRouteCost(uint16_t vehicle) const override;

                MainType computeCycleCost() const override;

                MainType improvPos2Opt(
                        Pos i,
                        Pos j
                ) const override;

                MainType improvPos2String(
                        Pos i,
                        Pos j,
                        Pos X,
                        Pos Y
                ) const override;

                MainType improvPos2Point(
                        Pos i,
                        Pos j
                ) const override;

                MainType improvPos1Point(
                        Pos i,
                        Pos j
                ) const override;

            };

            /// --------------------------------------------------------------------------------------------------------
            /// Cvrp-Solution
            /// --------------------------------------------------------------------------------------------------------

            class CvrpPath : public CvrpPathNaive {

            public:

                explicit CvrpPath(
                        const Instance &inst,
                        std::string name = "CVRP-Solution"
                );

                void reinitStructures(unsigned n) override;

                void updateStructures() override;

                std::unique_ptr<Solution> makeNewNaive() const override;
                
                bool canFitIntoCapacity(uint16_t vehicle, Ver vertex) const override;
                MainType computeDemands(uint16_t vehicle) const override;
                
                MainType computeDemandsIfAdded(uint16_t vehicle, Ver vertex) const override;
                
                bool isRelocateFeasible(uint16_t vehicle_from, uint16_t vehicle_to, Ver vertex) const;
                
                bool isExchangeFeasible(uint16_t vehicle1, uint16_t vehicle2, uint16_t ver1, uint16_t ver2) const;

                MainType improvPos2Opt(
                        Pos i,
                        Pos j
                ) const override;

                MainType improvPos2String(
                        Pos i,
                        Pos j,
                        Pos X,
                        Pos Y
                ) const override;

                MainType improvPos2Point(
                        Pos i,
                        Pos j
                ) const override;

                MainType improvPos1Point(
                        Pos i,
                        Pos j
                ) const override;

            protected:

                TdppVector f, l, delta;

            };

        }

    }

}

#endif //CVRP_SOLUTIONS_IMPL_HPP
