#ifndef _CPD_SEARCH_TESTER_AREA_PERTURBATOR_HEADER__
#define _CPD_SEARCH_TESTER_AREA_PERTURBATOR_HEADER__

#include <string>

#include <cpp-utils/Random.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/Interval.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/profiling.hpp>

#include <cpd-search/PerturbatedCost.hpp>

#include "AbstractPerturbator.hpp"

namespace pathfinding::map_perturbator::utils {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding;


    /**
     * @brief perturbate an area circled shaped
     * 
     * each arc is perturbated using a normal distribution: edges near the epicenter are greatly affected
     * while edges far from it are almost unaffected
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class CircleAreaPerturbator: public AbstractAreaPerturbator<G, V> {
        using This = AreaPerturbator<G, V>;
        using Super = AbstractPerturbator<G, V>;
    private:

		Interval<int> range;
        /**
         * @brief radius of the perturbated area to generate
         * 
         */
        Interval<int> radius;
    public:
        AreaPerturbator(const Interval<int>& range, const Interval<int>& radius, const Random& rnd, bool ensureStartGoalAreNotAffected) : Super{false, rnd, ensureStartGoalAreNotAffected, pointChooser}, range{range}, radius{radius} {

        }
        virtual ~AreaPerturbator() {

        }
        AreaPerturbator(const This& o) = delete;
        AreaPerturbator(This&& o): Super{std:move(o)}, range{o.range}, radius{o.radius} {
            
        }
        This& operator =(const This& o) = delete;
        This& operator =(This&& o) {
            Super::operator =(::std::move(o));
            this->range = ::std::move(o.range);
            this->radius = ::std::move(o.radius);
            return *this;
        }
    protected:
        virtual cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> getEdgesToPerturbate(const IImmutableGraph<G, V, cost_t>& originalGraph, nodeid_t start, nodeid_t goal, nodeid_t epicenter) {
            //fetch the actual radius
            auto actualRadius = Random::next(this->radius);

            cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> result{};
            this->performDFS(
                originalGraph, 
                epicenter, 
                actualRadius, 
                result,
                startPath,
                goalPath
            );
            return result;
        }
        virtual void perturbateEdge(const IImmutableGraph<G, V, cost_t>& originalGraph, AdjacentGraph<G, V, PerturbatedCost>& graphToPerturbate, nodeid_t start, nodeid_t goal, nodeid_t epicenter, nodeid_t sourceOfEdgeToPerturbate, nodeid_t sinkOfedgeToPerturbate, int depthOfEdgeToPerturbate) {
            double multiplier = this->getNewWeightInArea(depth, upper, actualRadius, 1, 10.0);
            debug("multiplier is", multiplier);
            Interval<int> singleton{static_cast<int>(static_cast<double>(oldWeight) * multiplier)};
            debug("before was", oldWeight, "but now it will be", singleton.getLB());

            this->perturbateArc(
                *graphToPerturbate,
                sourceOfEdgeToPerturbate,
                sinkOfedgeToPerturbate,
                singleton,
                "ASSIGN",
                this->rnd
            );
        }
    private:
        double getNewWeightInArea(double x, double M, double stddev, double k, double lambda) const {
            //3*e^(-(1/(0.1*2))*(x/5)^2) + 1
            return M * exp(-lambda * 0.5*(x/stddev)*(x/stddev)) + k;
        }
    };

}

#endif