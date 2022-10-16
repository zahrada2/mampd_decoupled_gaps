/**
 * File:    Solution.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_SOLUTION_HPP
#define CVRP_SOLUTION_HPP

#include "structures.hpp"
#include "MAPF_interface.hpp"

namespace cvrplib {

    namespace solutions {

        using Ver = int;
        using Pos = unsigned;

        class Solution {
        public:

            bool useMapf = false;
            /* bool useTTD = false; */
            int best_own_result = INT_MAX;
            int loaded_cost = -1;
            int precomputedTTD = -1;
            int previous_cost = -1;
            MAPF_solution best_sol;
            /// --------------------------------------------------------------------------------------------------------
            /// Public constructors + destructor
            /// --------------------------------------------------------------------------------------------------------

            Solution(const Solution &) = default;

            Solution(Solution &&) = default;

            Solution &operator=(const Solution &) = default;

            Solution &operator=(Solution &&) = default;

            virtual ~Solution() = default;

            /// --------------------------------------------------------------------------------------------------------
            /// Getters / setters
            /// --------------------------------------------------------------------------------------------------------
            
            const uint16_t getNumVehicles() const;

            void updateDemands();

            std::string toString();

            void resetDemands();
            
            void printOperatorStats();

            void setVrpTimes(std::vector<double> times);

            void setMapfTimes(std::vector<double> times);
            
            void setIterationStamp(unsigned iteration_when_found);
            
            void setMinimumVrpCost(MainType minimum_cost);
            
            void setImprovements(std::vector<long> improvements_to_set);

            void setCostFunctionValues(std::vector<long> cfv_to_set);
            
            const unsigned getVrpOperatorCount(unsigned operator_id) const;

            const unsigned getMapfOperatorCount(unsigned operator_id) const;

            const double getVrpOpTime(unsigned operator_id) const;

            const double getMapfOpTime(unsigned operator_id) const;
            
            const unsigned getIterationStamp() const;
            
            const MainType getMinimumVrpCost() const;
            
            const std::vector<long> getImprovements() const;
            
            const std::vector<long> getCostFunctionValues() const;

            void setVrpOpCounts(std::vector<long> op_count);

            void setMapfOpCounts(std::vector<long> op_count);

            bool switchMapf();

            bool getMapfMode();

            const std::string &getName() const;

            const Instance &instance() const;

            const unsigned &n() const;

            const MainType &sumWeights() const;

            const std::vector<Ver> &getSequence() const;

            bool setSequence(
                    const std::vector<Ver> &path,
                    bool checkIfValid = false
            );

            Ver getVer(Pos pos) const;
            
            Ver getVerCvrp(uint16_t vehicle, Pos pos) const;

            Pos getPos(Ver ver) const;

            void set(
                    Ver ver,
                    Pos pos
            );

            void initializePathCvrp(uint16_t numVehicles);
            
            void setCvrp(
                    Ver ver,
                    Pos pos,
                    uint16_t path_id
            );
            
            void pushCvrp(
                    uint16_t path_id,
                    Ver ver
            );
            
            void pushBackCvrp(
                    uint16_t path_id,
                    Ver ver
            );

            void rememberCost(MainType cost){
              current_cost = cost;
            }

            MainType recallCost(){
              return current_cost;
            }

            void printSolution();

            const std::vector<std::vector<Ver>> getCvrpPath() const;
            
            void setCvrpPath(std::vector<std::vector<Ver>> new_path);
            
            int getNumVertices(uint16_t vehicle) const;

            void setInstance(const Instance &inst);

            void setHistogram(std::vector<uint16_t> histogram);

            std::vector<uint16_t> getHistogram() const;

            /// --------------------------------------------------------------------------------------------------------
            /// Utils
            /// --------------------------------------------------------------------------------------------------------

            virtual void clear();

            virtual void clone(
                    Solution &path,
                    bool name = false
            );

            virtual std::string str() const;

            virtual bool isValid() const;

            virtual bool isValid(bool checkInverse) const;

            virtual void reinitStructures(unsigned n) = 0;

            virtual void updateStructures() = 0;

            virtual std::unique_ptr<Solution> makeNewNaive() const = 0;

            /// --------------------------------------------------------------------------------------------------------
            /// Costs calculation
            /// --------------------------------------------------------------------------------------------------------
            
            virtual MainType computeTSPCost() const = 0;

            virtual MainType computeTDPCost() const = 0;
            
            virtual MainType computeTTD() const = 0;
            
            virtual void precomputeTTDDiscount() = 0;
            
            virtual MainType computeCostSelected(bool use_ttd) const = 0;

            virtual MainType computeCost() const = 0;
            
            virtual MainType computeSingleRouteCost(uint16_t vehicle) const = 0;

            virtual MainType computeCycleCost() const = 0;
                
            virtual bool isRelocateFeasible(uint16_t vehicle_from, uint16_t vehicle_to, Ver vertex) const = 0;
                
            virtual bool isExchangeFeasible(uint16_t vehicle1, uint16_t vehicle2, uint16_t ver1, uint16_t ver2) const = 0;
            
            virtual MainType computeDemands(uint16_t vehicle) const = 0;
            
            virtual MainType computeDemandsIfAdded(uint16_t vehicle, Ver vertex) const = 0;
                
            virtual bool canFitIntoCapacity(uint16_t vehicle, Ver vertex) const = 0;


            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Neighborhood 1: sequence insert
            /// --------------------------------------------------------------------------------------------------------
            
            MainType trySequenceInsert(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos); 
            
            void sequenceInsert(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos);
            
            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Neighborhood 2: reverse sequence insert
            /// --------------------------------------------------------------------------------------------------------
            
            MainType trySequenceInsertReverse();

            void sequenceInsertReverse(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos);
            
            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Neighborhood 3: single customer (vertex) swap 
            /// --------------------------------------------------------------------------------------------------------
            
            MainType tryVertexSwap();

            void vertexSwap(uint16_t src_vehicle, Pos src_pos, uint16_t tgt_vehicle, Pos tgt_pos);
            
            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Neighborhood 4: sequence swap 
            /// --------------------------------------------------------------------------------------------------------

            MainType trySequenceSwap();

            void sequenceSwap(uint16_t src_vehicle, Pos src_pos, uint16_t amount, uint16_t tgt_vehicle, Pos tgt_pos);

            /// --------------------------------------------------------------------------------------------------------------------
            /// Operator CVRP - Exchange 
            /// --------------------------------------------------------------------------------------------------------------------

            MainType tryExchange(uint16_t vehicle1, uint16_t vehicle2, Pos i, Pos j);

            void exchange(uint16_t vehicle1, uint16_t vehicle2, Pos i, Pos j);

            /// --------------------------------------------------------------------------------------------------------------------
            /// Operator CVRP - Relocate 
            /// --------------------------------------------------------------------------------------------------------------------
            
            MainType tryRelocate(uint16_t src_vehicle, uint16_t tgt_vehicle, Pos i, Pos j);

            void relocate(uint16_t src_vehicle, uint16_t tgt_vehicle, Pos i, Pos j);

            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Two-opt 
            /// --------------------------------------------------------------------------------------------------------
            
            MainType tryTwoOptCvrp(uint16_t vehicle,
                    Pos i,
                    Pos j
            );
 
            void twoOptCvrp(uint16_t vehicle,
                    Pos i,
                    Pos j
            );
            
            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Three-opt 
            /// --------------------------------------------------------------------------------------------------------
            
            MainType tryThreeOptCvrp(uint16_t vehicle,
                    Pos i_in,
                    Pos j_in,
                    Pos k_in
            );

            void threeOptCvrp(uint16_t vehicle,
                    Pos i_in,
                    Pos j_in,
                    Pos k_in
            );


            /// --------------------------------------------------------------------------------------------------------
            /// CVRP VND Operator - Double-bridge (4opt)
            /// --------------------------------------------------------------------------------------------------------
            

            MainType tryDoubleBridgeCvrp(uint16_t vehicle,
                    Pos i,
                    Pos j,
                    Pos k,
                    Pos l
            );

            void doubleBridgeCvrp(uint16_t vehicle,
                    Pos i,
                    Pos j,
                    Pos k,
                    Pos l
            );

            /// --------------------------------------------------------------------------------------------------------
            /// Operator - Two-opt-flip move
            /// --------------------------------------------------------------------------------------------------------

            void moveVer2Opt(
                    Ver u,
                    Ver v
            );

            MainType improvVer2Opt(
                    Ver u,
                    Ver v
            ) const;

            void movePos2Opt(
                    Pos i,
                    Pos j
            );

            virtual MainType improvPos2Opt(
                    Pos i,
                    Pos j
            ) const = 0;

            /// --------------------------------------------------------------------------------------------------------
            /// Operator - Two-string move
            /// --------------------------------------------------------------------------------------------------------

            void moveVer2String(
                    Ver u,
                    Ver v,
                    Pos X,
                    Pos Y
            );

            MainType improvVer2String(
                    Ver u,
                    Ver v,
                    Pos X,
                    Pos Y
            ) const;

            void movePos2String(
                    Pos i,
                    Pos j,
                    Pos X,
                    Pos Y
            );

            virtual MainType improvPos2String(
                    Pos i,
                    Pos j,
                    Pos X,
                    Pos Y
            ) const = 0;

            /// --------------------------------------------------------------------------------------------------------
            /// Operator - Two-point move
            /// --------------------------------------------------------------------------------------------------------

            void moveVer2Point(
                    Ver u,
                    Ver v
            );

            MainType improvVer2Point(
                    Ver u,
                    Ver v
            ) const;

            void movePos2Point(
                    Pos i,
                    Pos j
            );

            virtual MainType improvPos2Point(
                    Pos i,
                    Pos j
            ) const = 0;

            /// --------------------------------------------------------------------------------------------------------
            /// Operator - One-point move
            /// --------------------------------------------------------------------------------------------------------

            void moveVer1Point(
                    Ver u,
                    Ver v
            );

            MainType improvVer1Point(
                    Ver u,
                    Ver v
            ) const;

            void movePos1Point(
                    Pos i,
                    Pos j
            );

            virtual MainType improvPos1Point(
                    Pos i,
                    Pos j
            ) const = 0;

        protected:

            /// --------------------------------------------------------------------------------------------------------
            /// Protected constructor
            /// --------------------------------------------------------------------------------------------------------

            explicit Solution(
                    const Instance &inst,
                    std::string name = "unknown"
            );

            /// --------------------------------------------------------------------------------------------------------
            /// Protected variables
            /// --------------------------------------------------------------------------------------------------------

            const Instance *inst;

            std::vector<Ver> path;
            std::vector<Pos> pathInv;
            std::vector<long> vrp_operator_calls{0,0,0,0,0};
            std::vector<double> vrp_operator_times{0,0,0,0,0};
            std::vector<long> mapf_operator_calls{0,0,0,0,0};
            std::vector<double> mapf_operator_times{0,0,0,0,0};
            std::vector<long> improvements;
            std::vector<long> costFunctionValues;
            std::vector<uint16_t> _histogram;

            bool useMapfLevelOperators = false;
            
            std::vector<std::vector<Ver>> pathCvrp;
            std::vector<std::vector<Pos>> pathInvCvrp;

            MainType current_cost;
            MainType minimum_vrp_cost;
            std::vector<MainType> current_demands;

            unsigned iterationStamp;
            

        private:

            static const auto INVALID_PATH_FLAG = MainType(-1);

            /// --------------------------------------------------------------------------------------------------------
            /// Private variables
            /// --------------------------------------------------------------------------------------------------------

            std::string name;

        };

    }

}

#endif //CVRP_SOLUTION_HPP
