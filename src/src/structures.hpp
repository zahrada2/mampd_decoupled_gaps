/**
 * File:    structures.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_STRUCTURES_HPP
#define CVRP_STRUCTURES_HPP

#include "utils.hpp"
#include "MAPF_interface.hpp"

#include <vector>
#include <algorithm>

namespace cvrplib {

    using MainType = long long;

    using pii = std::pair<int, int>;
    using pti = std::pair<MainType, int>;

    using UnsignedVector = std::vector<uint16_t>;
    using DoubleVector = std::vector<double>;
    using DoubleMatrix = std::vector<DoubleVector>;
    using DoubleCubicMatrix = std::vector<DoubleMatrix>;

    using TdppVector = std::vector<MainType>;
    using TdppMatrix = std::vector<TdppVector>;
    using TdppCubicMatrix = std::vector<TdppMatrix>;

    namespace in {
        
        struct CvrpStructures {
            DoubleMatrix d;
            UnsignedVector starting_depots;
            UnsignedVector ending_depots;
            UnsignedVector q;
            unsigned numVehicles;
            unsigned vehicleCapacity;
            bool useIntegratedMapf;
            bool mapfLevelOps;
            std::vector<int> waitTimes;
            /* std::vector<uint16_t> orders; */
            /* std::vector<int> assigned_stations; */
            std::string problemName;
            int numRun;
            bool loadSolution;
            bool useTtd;
            bool useFa;
            std::vector<double> closestDepotDistances;
            
        };
        
        struct TdpStructures {
            DoubleMatrix d;
        };

        struct GspStructures {
            DoubleVector w;
            DoubleMatrix d;
        };

        struct AgspStructures {
            DoubleVector w;
            DoubleVector theta1;
            DoubleMatrix rho;
            DoubleCubicMatrix theta3;
        };

    }

    namespace out {

        struct Output {
            MainType cost;
            double time;
        };

    }

    class Goals {

    public:
        explicit Goals();

        const bool &isGoalCostSet() const;

        const bool &isGoalTimeSet() const;

        const MainType &getGoalCost() const;

        const double &getGoalTime() const;

        void setGoalCost(MainType cost);

        void setGoalTime(double time);

        void unsetGoalCost();

        void unsetGoalTime();

        void resetTimer();

        bool isGoalCostMet(MainType &cost) const;

        bool isGoalTimeMet() const;

    private:
        bool c;
        bool t;
        MainType cost;
        double time;
        SimpleClock timer;

    };

    struct Instance {

        boost::shared_ptr<MAPF_interface> mapfSolver;
        boost::shared_ptr<MAPF_interface> secondary_mapf_solver;
        unsigned n;
        UnsignedVector demands;
        UnsignedVector starting_depots;
        UnsignedVector ending_depots;
        uint16_t numVehicles;
        uint16_t vehicleCapacity;
        bool useIntegratedMapf;
        bool mapfLevelOps;
        float alpha;
        int waitTime;
        std::vector<int> waitTimes;
        MainType sumWeights;
        TdppVector w;
        TdppVector theta1;
        TdppMatrix rho;
        TdppCubicMatrix theta3;
        std::vector<std::vector<int>> neighborsDist;
        std::vector<std::vector<int>> neighborsRatio;
        std::vector<uint16_t> orders;
        std::vector<int> assigned_stations;
        int numRun;
        std::string problem_name;
        bool loadSolution = false;
        bool useTtd;
        bool useFa;
        std::vector<double> closestDepotDistances;

        explicit Instance(unsigned n);

        
        explicit Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, uint16_t numVehicles, uint16_t vehicleCapacity, bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::string problem_name, int numRun);
        /// Constructor for CVRP-type instance with customer demands
        //
        explicit Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, uint16_t numVehicles, uint16_t vehicleCapacity, bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::string problem_name, int numRun, bool loadSolution, bool useTtd, bool useFa, std::vector<double> closestDepotDistances);

        /// Constructor for CVRP-type instance with customer demands
        /* explicit Instance(unsigned n, UnsignedVector starting_depots, UnsignedVector ending_depots, UnsignedVector demands, uint16_t numVehicles, uint16_t vehicleCapacity, */ 
        /*     bool useIntegratedMapf, bool mapfLevelOps, std::vector<int> waitTimes, std::vector<uint16_t> orders, std::vector<int> assigned_stations); */



        void init();

        void setProblemName(std::string problem);
        void setNumRun(int numRuns);

        bool setMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface);

        bool setSecondaryMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface);

    private:

        void sortNeighborsDistance();

        void sortNeighborsRatio();

        void computeSumWeights();

    };
    
    /* struct CvrpInstance : Instance { */

    /*     unsigned n; */
    /*     /1* MainType sumWeights; *1/ */
    /*     /1* TdppVector w; *1/ */
    /*     /1* TdppVector theta1; *1/ */
    /*     /1* TdppMatrix rho; *1/ */
    /*     /1* TdppCubicMatrix theta3; *1/ */
    /*     /1* std::vector<std::vector<int>> neighborsDist; *1/ */
    /*     /1* std::vector<std::vector<int>> neighborsRatio; *1/ */

    /*     /1* explicit CvrpInstance(unsigned n); *1/ */

    /*     /1* void init(); *1/ */

    /* private: */

    /*     /1* void sortNeighborsDistance(); *1/ */

    /*     /1* void sortNeighborsRatio(); *1/ */

    /*     /1* void computeSumWeights(); *1/ */

    /* }; */

}

#endif //CVRP_STRUCTURES_HPP
