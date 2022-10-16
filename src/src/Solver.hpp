/**
 * File:    Solver.hpp
 *
 * Date:    17/08/2021
 * Author:  David Zahradka
 * E-mail:  zahrada2@fel.cvut.cz
 * Based on code by Jan Mikula
 *
 */

#ifndef CVRP_SOLVER_HPP
#define CVRP_SOLVER_HPP

#include "Solution.hpp"
#include "MAPF_interface.hpp"
#include "MAPF/MAPF_SIPP.hpp"
#include "MAPF/MAPF_PBS.hpp"

#include <map>

namespace cvrplib {

    enum class Mode : int {
        undefined,
        tdp,
        cvrp,
        gsp,
        agsp,
        count
    };

    enum class Method : int {
        undefined,
        vns,
        fmivns,
        simplified_vns,
        simpleTwoPhase,
        twoPhaseVns,
        clarkeWright,
        mcwsa,
        mcwsa_alternative,
        count
    };

    class Solver {

    public:

        Solver();

        bool setMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface);

        bool setSecondaryMAPFSolver(boost::shared_ptr<MAPF_interface> mapf_interface);

        bool setMethod(Method method);

        bool setMethod(std::string method);

        std::string getMethodName();

        void setData(in::TdpStructures &dataTDP);
        
        void setData(in::CvrpStructures &dataCVRP);

        void setData(in::GspStructures &dataGSP);

        void setData(in::AgspStructures &dataAGSP);

        const out::Output *solve(
                unsigned itersOuter = 10,
                unsigned itersInner = 30,
                unsigned operatorSeq = 4,
                unsigned shakersSeq = 3,
                bool itersInnerFixed = false,
                bool initialGreedy = false
        );

        const solutions::Solution &getPath();

        static void computeSum(
                const DoubleVector &weights,
                double &sum
        );

        static void normalizeWeights(DoubleVector &weights);

        static void getMax(
                const DoubleMatrix &distances,
                double &max
        );

        static void divideDistancesByMaxDistance(DoubleMatrix &distances);

        Mode getMode() const;

        void setMode(Mode mode);

        long getScale() const;

        void setScale(long scale);

        Goals goals;

    private:

        boost::shared_ptr<MAPF_interface> mapf_solver;
        boost::shared_ptr<MAPF_interface> secondary_mapf_solver;

        bool newData;
        bool newMode;
        long scale;
        Mode mode;
        Method method;
        out::Output output;
        std::unique_ptr<Instance> instance;
        /* std::unique_ptr<CvrpInstance> cvrpInstance; */
        std::unique_ptr<solutions::Solution> pathRes1;
        std::unique_ptr<solutions::Solution> pathAux1;
        std::unique_ptr<solutions::Solution> pathAux2;
        std::unique_ptr<solutions::Solution> pathAux3;
        std::map<Method, std::string> method2string;
        std::map<std::string, Method> string2method;

        void addMethod(
                Method method,
                std::string name
        );

    };

}

#endif //CVRP_SOLVER_HPP
