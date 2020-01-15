#ifndef _PATHFINDINGMAPPERTURBATORUTILS_ABSTARCTPOINTCHOOSER_HEADER__
#define _PATHFINDINGMAPPERTURBATORUTILS_ABSTARCTPOINTCHOOSER_HEADER__

#include <cpp-utils/igraph.hpp>

#include <pathfinding-utils/IPath.hpp>

namespace pathfinding::map_perturbator::utils {

    using namespace cpp_utils::graphs;
    using namespace pathfinding;

    /**
     * @brief represents a class whose job is to fetch a point in the map
     * 
     * such point will be the epicenter of a ::AbstractAreaPerturbator instance
     * 
     * @tparam G paylaod of the graph
     * @tparam V type every vertex in the graph has associated
     */
    template <typename G, typename V>
    class AbstractPointChooser {
    public:
        /**
         * @brief choose a point in the map
         * 
         * @param graph the map where we need to choose a point
         * @param start the start point of a query
         * @param goal the goal point of a query
         * @return nodeid_t the node we have chosen
         */
        virtual nodeid_t choosePoint(const IImmutableGraph<G, V, cost_t>& graph, nodeid_t start, nodeid_t goal) = 0;
        /**
         * @brief get the path computed by the software (if any)
         * 
         * @return const NodePath a path the software has computed in the last call of ::choosePoint (if any).
         * null if no path has been computed.
         */
        virtual const NodePath* getPathComputed() const = 0;
    };

}

#endif