#ifndef _PATHFINDINGMAPPERTURBATOR_POINTALONGOPTIMALPATHPOINTCHOOSER_HEADER
#define _PATHFINDINGMAPPERTURBATOR_POINTALONGOPTIMALPATHPOINTCHOOSER_HEADER

#include "AbstractPointChooser.hpp"

namespace pathfinding::map_perturbator::utils {

    template <typename G, typename V>
    class PointAlongOptimalPathPointChooser: public AbstractPointChooser<G, V> {
    public:
        typedef PointAlongOptimalPathPointChooser<G,V> This;
        typedef AbstractPointChooser<G,V> Super;
    private:
        /**
         * @brief a interval which endpoints are between 0 and 1
         * 
         * Represents the interval where we should fetch the point along an optimal path from start to goal.
         * 0 represents the start node while 1 represents the goal node.
         * For instance if it is [0,0.5] we will fetch a position in the first half of an optimal path
         * 
         */
        DoubleInterval whereToChoosePoint;
        /**
         * @brief cache containing the last optimal path computed
         * 
         */
        mutable NodePath optimalPathCache;
    public:
        PointAlongOptimalPathPointChooser(DoubleInterval whereToChoosePoint): whereToChoosePoint{whereToChoosePoint}, optimalPathCache{} {

        }
    public:
        virtual nodeid_t choosePoint(const IImmutableGraph<G, V, cost_t>& graph, nodeid_t start, nodeid_t goal) {
            auto point = Random::next(this->whereToChoosePoint);
            info("the point we have chosen is", point);
            
            //compute an optimal path
            costFunction_t<cost_t> mapper = [&](const cost_t& c) { return c;};
            this->optimalPathCache = getOptimalPathAsVertices(graph, start, goal, mapper);
            debug("DONE!");
            debug("optimal path is", this->optimalPathCache);
            auto positionInOptimal = floor(point * this->optimalPathCache.size());
            debug("position chosen is", positionInOptimal);
            return this->optimalPathCache.at(positionInOptimal);
        }
        /**
         * @brief get the path computed by the software (if any)
         * 
         * @return const NodePath a path the software has computed in the last call of ::choosePoint (if any).
         * null if no path has been computed.
         */
        virtual const NodePath* getPathComputed() const {
            return &this->optimalPathCache;
        }
    };

}

#endif