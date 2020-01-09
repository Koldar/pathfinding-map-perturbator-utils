/*
 * RandomPerturbator.h
 *
 *  Created on: Oct 25, 2018
 *      Author: koldar
 */

#ifndef _PATHFINDINGAPPERTURBATORUTILS_RANDOM_PERTURBATOR_HEADER__
#define _PATHFINDINGAPPERTURBATORUTILS_RANDOM_PERTURBATOR_HEADER__

#include "AbstractPerturbator.hpp"
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/adjacentGraph.hpp>

namespace pathfinding::map_perturbator::utils {

	using namespace cpp_utils::graphs;
	using namespace cpp_utils;
	using namespace pathfinding::search;

	template <typename G, typename V>
	class RandomPerturbator : public AbstractPerturbator<G, V> {
		typedef RandomPerturbator<G, V> This;
		typedef AbstractPerturbator<G, V> Super;
	protected:
		uint32_t edgesToAlter;
		Interval<double> range;
		const std::string& mode;
		const Random& rnd;
	public:
		RandomPerturbator(bool allowEdgeCostDecrements, uint32_t edgesToAlter, const Interval<double>& range, const std::string& mode, const Random& rnd) : Super{allowEdgeCostDecrements}, edgesToAlter{edgesToAlter}, range{range}, mode{mode}, rnd{rnd} {

		}
		virtual ~RandomPerturbator() {

		}
		RandomPerturbator(const This& o): Super{o.allowEdgeCostDecrements}, edgesToAlter{o.edgesToAlter}, range{o.range}, mode{o.mode}, rnd{o.rnd} {

		}
		RandomPerturbator(This&& o): Super{o.allowEdgeCostDecrements}, edgesToAlter{o.edgesToAlter}, range{o.range}, mode{o.mode}, rnd{o.rnd} {
		}
		This& operator = (const This& o) = delete;
		This& operator = (This&& o) = delete;
	public:
		virtual AdjacentGraph<G, V, PerturbatedCost>* perturbateGraph(int experimentId, const GridMapScenarioExperiment& experiment, nodeid_t start, nodeid_t goal, const IImmutableGraph<G, V, cost_t>& originalGraph) {
			AdjacentGraph<G, V, PerturbatedCost>* result = new AdjacentGraph<G, V, PerturbatedCost>{
				*originalGraph.mapEdges(std::function<PerturbatedCost(const cost_t&)>([&](const cost_t& c) { return PerturbatedCost{c, false};}))
			};

			debug("graph is: ", originalMap);

			double totalEdges = 0;
			//we have changed both edges
			SetPlus<Edge<cost_t>> edges = originalGraph.getEdgeSet(true);

			//every time we perturbe an edge we change w of them (a->b and b->a).
			//but on the frontend we required to change (say 50) edges we obviously mean 50 DIFFERENT edges.
			//hence the actual perturbated density will be twice the normal edges
			for (; this->edgeChanged<this->edgesToAlter;) {

				debug("perturbating another edge ", this->edgeChanged);
				//remember: this is just a copy of the edge in the graph. We still need to check it on the perturbated graph!
				Edge<cost_t> consideredEdge = edges.getRandomItem();
				nodeid_t sourceId = consideredEdge.getSourceId();
				nodeid_t sinkId = consideredEdge.getSinkId();

				if (result->getEdge(sourceId, sinkId).isPerturbated()) {
					//we have chosen a perturbated edge. try another one
					warning("apparently the edge we have chosen is perturbated.... weird...");
					continue;
				}

				this->perturbateArc(*result, sourceId, sinkId, this->range, this->mode, this->rnd);

				edges.remove(consideredEdge);
			}

			return result;
		}
	};

}


#endif /* RANDOMPERTURBATOR_H_ */
