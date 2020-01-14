#ifndef _PATHFINDINGMAPPERTURBATOR_POINTALONGOPTIMALPATHPOINTCHOOSER_HEADER
#define _PATHFINDINGMAPPERTURBATOR_POINTALONGOPTIMALPATHPOINTCHOOSER_HEADER

#include "AbstractPointChooser.hpp"

namespace pathfinding::map_perturbator::utils {

    template <typename G, typename V, typename E, typename GETCOST>
    class PointAlongOptimalPathPointChooser: AbstractPointChooser<G, V, E, GETCOST> {
    public:
        typedef PointAlongOptimalPathPointChooser<G,V,E> This;
        typedef AbstractPointChooser<G,V,E> Super;
    private:
        /**
         * @brief a interval which endpoints are between 0 and 1
         * 
         * Represents the interval where we should fetch the point along an optimal path from start to goal.
         * 0 represents the start node while 1 represents the goal node.
         * For instance if it is [0,0.5] we will fetch a position in the first half of an optimal path
         * 
         */
        Interval<double> whereToChoosePoint;
        /**
         * @brief cache containing the last optimal path computed
         * 
         */
        mutable NodePath optimalPathCache;
    public:
        PointAlongOptimalPathPointChooser(Interval<double> whereToChoosePoint): whereToChoosePoint{whereToChoosePoint}, optimalPathCache{} {

        }
    public:
        virtual nodeid_t choosePoint(const ImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, GETCOST getCost) {
            auto point = Random::nextDouble(this->whereToChoosePoint);
            
            //compute an optimal path
            this->optimalPath = getOptimalPathAsVertices(graph, start, goal, getCost);
            auto positionInOptimal = floor(point * optimalPath.size());

            return this->optimalPath.at(positionInOptimal);
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
    }

}

#endif