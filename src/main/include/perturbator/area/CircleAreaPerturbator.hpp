#ifndef _PATHFINDINGMAPPERTURBATORUTILS_CIRCLEAREAPERTURBATOR_HEADER__
#define _PATHFINDINGMAPPERTURBATORUTILS_CIRCLEAREAPERTURBATOR_HEADER__

#include <string>

#include <cpp-utils/Random.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/Interval.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/profiling.hpp>

#include <cpd-search/PerturbatedCost.hpp>

#include "AbstractAreaPerturbator.hpp"
#include "AbstractPointChooser.hpp"

namespace pathfinding::map_perturbator::utils {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding;


    /**
     * @brief perturbate an area circled shaped from the epicenter
     * 
     * each arc is perturbated using a normal distribution: edges near the epicenter are greatly affected
     * while edges far from it are almost unaffected
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class CircleAreaPerturbator: public AbstractAreaPerturbator<G, V> {
        using This = CircleAreaPerturbator<G, V>;
        using Super = AbstractAreaPerturbator<G, V>;
    private:

		Interval<int> range;
        /**
         * @brief radius of the perturbated area to generate
         * 
         */
        Interval<int> radius;
        /**
         * @brief Actual radius used while generating the perturbated graph
         * 
         */
        mutable int actualRadius;
    public:
        CircleAreaPerturbator(const Random& rnd, bool ensureStartGoalAreNotAffected, AbstractPointChooser<G,V>& pointChooser, const Interval<int>& range, const Interval<int>& radius) : Super{false, rnd, ensureStartGoalAreNotAffected, pointChooser}, range{range}, radius{radius}, actualRadius{0} {

        }
        virtual ~CircleAreaPerturbator() {

        }
        CircleAreaPerturbator(const This& o) = delete;
        CircleAreaPerturbator(This&& o): Super{std:move(o)}, range{o.range}, radius{o.radius}, actualRadius{o.actualRadius} {
            
        }
        This& operator =(const This& o) = delete;
        This& operator =(This&& o) {
            Super::operator =(::std::move(o));
            this->range = ::std::move(o.range);
            this->radius = ::std::move(o.radius);
            this->actualRadius = ::std::move(o.actualRadius);
            return *this;
        }
    protected:
        virtual cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> getEdgesToPerturbate(const IImmutableGraph<G, V, cost_t>& originalGraph, nodeid_t start, nodeid_t goal, nodeid_t epicenter) {
            //fetch the actual radius
            this->actualRadius = Random::next(this->radius);

            cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> result{};
            this->performDFS(
                originalGraph, 
                epicenter, 
                this->actualRadius, 
                result,
                start,
                goal
            );
            return result;
        }
        virtual void perturbateEdge(const IImmutableGraph<G, V, cost_t>& originalGraph, AdjacentGraph<G, V, PerturbatedCost>& graphToPerturbate, nodeid_t start, nodeid_t goal, nodeid_t epicenter, nodeid_t sourceOfEdgeToPerturbate, nodeid_t sinkOfedgeToPerturbate, int depthOfEdgeToPerturbate, cost_t originalWeight) {
            auto upper = this->range.getUpperbound();
            double multiplier = this->getNewWeightInArea(depthOfEdgeToPerturbate, upper, this->actualRadius, 1, 10.0);
            debug("multiplier is", multiplier);
            Interval<int> singleton{static_cast<int>(static_cast<double>(originalWeight) * multiplier)};
            debug("before was", originalWeight, "but now it will be", singleton.getLB());

            this->perturbateArc(
                graphToPerturbate,
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