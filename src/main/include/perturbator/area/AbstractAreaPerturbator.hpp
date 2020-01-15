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
#include "AbstractPointChooser.hpp"

namespace pathfinding::map_perturbator::utils {

    using namespace pathfinding::search;

    /**
     * @brief exception to throw when you fetch the start node of a query and you didn't want it
     * 
     */
    class StartNodeInterceptedException : cpp_utils::exceptions::AbstractException {
    public:
        StartNodeInterceptedException();
    };

    /**
     * @brief exception to throw when you fetch the goal node of a query and you didn't want it
     * 
     */
    class GoalNodeInterceptedException : cpp_utils::exceptions::AbstractException {
    public:
        GoalNodeInterceptedException();
    };

    /**
     * @brief given a point in the map, perturbate the map around that point in a certain way
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class AbstractAreaPerturbator: public AbstractPerturbator<G, V> {
    public:
        using This = AbstractAreaPerturbator<G, V>;
        using Super = AbstractPerturbator<G, V>;
    protected:
        const Random& rnd;
        /**
         * @brief if true we will throw error if we're trying to change an edge hose endpoint is either the start/end of the query
         * 
         */
        bool ensureStartGoalAreNotAffected;
        AbstractPointChooser<G, V>& pointChooser;
        /**
         * @brief cache graph containing the original graph but with perturbated cost
         * 
         * null pointer if the cache has been created yet
         */
        mutable std::unique_ptr<IImmutableGraph<G, V, PerturbatedCost>> perturbatedGraphCache;
    public:
        AbstractAreaPerturbator(bool allowEdgeCostDecrements, const Random& rnd, bool ensureStartGoalAreNotAffected, AbstractPointChooser<G,V>& pointChooser) : Super{allowEdgeCostDecrements}, rnd{rnd}, ensureStartGoalAreNotAffected{ensureStartGoalAreNotAffected}, pointChooser{pointChooser}, perturbatedGraphCache{nullptr} {

        }
        virtual ~AbstractAreaPerturbator() {

        }
        AbstractAreaPerturbator(const This& o) = delete;
        This& operator=(const This& o) = delete;
        This& operator =(This&& o) {
            Super::operator =(std::move(o));
            this->rnd = std::move(o.rnd);
            this->ensureStartGoalAreNotAffected = std::move(o.ensureStartGoalAreNotAffected);
            this->pointChooser = std::move(o.pointChooser);
            this->perturbatedGraphCache = std::move(o.perturbatedGraphCache);
            return *this;
        }
    protected:
        /**
         * @brief fetch a list of edges to perturbate
         * 
         * @param originalGraph the graph where we need to apply perturbations
         * @param start start node of the query
         * @param goal end node of the query
         * @param epicenter place in the center of the shape we need to perturbate
         * @return cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> list of edges we need to perturbate. Each tuple is composed as follws: the first element is the source id of an edge, the second element is the sink if of the same edge to perturbate, while the third element is the distance from the epicenter.
         */
        virtual cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> getEdgesToPerturbate(const IImmutableGraph<G, V, cost_t>& originalGraph, nodeid_t start, nodeid_t goal, nodeid_t epicenter) = 0;
        /**
         * @brief perturbate an edge
         * 
         * @post
         *  @li graphToPerturbate has a perturbate arc
         * 
         * @param originalGraph 
         * @param start 
         * @param goal 
         * @param epicenter 
         * @param sourceOfEdgeToPerturbate 
         * @param sinkOfedgeToPerturbate 
         * @param depthOfEdgeToPerturbate 
         */
        virtual void perturbateEdge(const IImmutableGraph<G, V, cost_t>& originalGraph, AdjacentGraph<G, V, PerturbatedCost>& graphToPerturbate, nodeid_t start, nodeid_t goal, nodeid_t epicenter, nodeid_t sourceOfEdgeToPerturbate, nodeid_t sinkOfedgeToPerturbate, int depthOfEdgeToPerturbate, cost_t originalWeight) = 0;
    protected:
        virtual AdjacentGraph<G, V, PerturbatedCost>* perturbateGraph(int experimentId, const GridMapScenarioExperiment& experiment, nodeid_t start, nodeid_t goal, const IImmutableGraph<G, V, cost_t>& originalGraph) {
            //build result
            if (this->perturbatedGraphCache == nullptr) {
                debug("cache is null. Create perturbatedGraphCache");
                std::function<PerturbatedCost(const cost_t& c)> mapper =  [&](const cost_t& c) -> PerturbatedCost { return PerturbatedCost{c, false};};
                auto p = originalGraph.mapEdges(mapper);
                this->perturbatedGraphCache = std::move(p);
            }
            //copy the template perturbatedGraphTemplate
            AdjacentGraph<G, V, PerturbatedCost>* result = new AdjacentGraph<G, V, PerturbatedCost>{*this->perturbatedGraphCache};

            //choose the epicenter
            nodeid_t epicenter = this->pointChooser.choosePoint(originalGraph, start, goal);
            debug("optimal path is", this->pointChooser.getPathComputed());
            debug("epicenter is ", epicenter);

            //fetch the edges that we need to change
            cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>> edgesToPerturbate = this->getEdgesToPerturbate(originalGraph, start, goal, epicenter);
            debug("edges to perturbate are", edgesToPerturbate.size());

            //ok, now we alter all arcs involving these nodes
            if (edgesToPerturbate.size() == 0) {
                throw cpp_utils::exceptions::ImpossibleException{"the DFS generated %ld edges to perturbate!", edgesToPerturbate.size()};
            }

            for (auto edgeInfo: edgesToPerturbate) {
                nodeid_t sourceId = std::get<0>(edgeInfo);
                nodeid_t sinkId = std::get<1>(edgeInfo);
                int depth = std::get<2>(edgeInfo);

                if (this->ensureStartGoalAreNotAffected) {
                    if (sinkId == start) {
                        throw StartNodeInterceptedException{};
                    }
                    if (sinkId == goal) {
                        throw GoalNodeInterceptedException{};
                    }
                }

                cost_t oldWeight = originalGraph.getEdge(sourceId, sinkId);

                debug("we're affecting ", sourceId, "->", sinkId, "! start=", start, "end=", goal, " depth=", depth, "(oldweight is", oldWeight, ")");
                this->perturbateEdge(
                    originalGraph, *result,
                    start, goal, epicenter,
                    sourceId, sinkId, depth,
                    oldWeight
                );
            }

            return result;
        }
        void performDFS(const IImmutableGraph<G, V, cost_t>& graph, nodeid_t epicenter, int maxDepth, cpp_utils::vectorplus<std::tuple<nodeid_t, nodeid_t, int>>& edges, nodeid_t startPath, nodeid_t endPath) const {
            std::vector<bool> visited(graph.numberOfVertices(), false);

            //first element is the nodeid_t of the vertex in the DFS, the second element is the depth of the vertex in the DFS
            cpp_utils::vectorplus<std::tuple<nodeid_t, int>> q;
            q.addTail(std::make_tuple(epicenter, 0));

            while (!q.isEmpty()) {
                auto ni = q.pop();
                nodeid_t vertexId = std::get<0>(ni);
                int depth = std::get<1>(ni);

                if (visited[vertexId]) {
                    continue;
                }
                visited[vertexId] = true;
                if (this->ensureStartGoalAreNotAffected) {
                    debug("we're affecting ", vertexId, "! start=", startPath, "end=", endPath, " depth=", ni.depth, "max depth= ", maxDepth);
                    if (vertexId == startPath) {
                        throw StartNodeInterceptedException{};
                    }
                    if (vertexId == endPath) {
                        throw GoalNodeInterceptedException{};
                    }
                }

                debug("we're in node ", vertexId, " the out degree is ", graph.getOutDegree(vertexId));
                for (auto e : graph.getOutEdges(vertexId)) {
                    nodeid_t next = e.getSinkId();
                    if (visited[next]) {
                        //the sink has arelady been dealt with. Ignore: in this way we add the edge u->v, but not
                        //the edge v->u
                        continue;
                    }
                    if ((depth + 1) >= maxDepth) {
                        //the sink is beyond maxDepth. Ignore
                        continue;
                    }

                    edges.addTail(std::make_tuple(vertexId, next, depth));
                    q.addTail(std::make_tuple(next, depth + 1));
                }

            }
        }

    };

}

#endif