/*
 * AbstractPerturbator.hpp
 *
 *  Created on: Oct 25, 2018
 *      Author: koldar
 */

#ifndef _PATHFINDINGAPPERTURBATORUTILS_ABSTRACT_PERTURBATOR_HEADER__
#define _PATHFINDINGAPPERTURBATORUTILS_ABSTRACT_PERTURBATOR_HEADER__

#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/Interval.hpp>
#include <cpp-utils/Random.hpp>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/adjacentGraph.hpp>

#include <pathfinding-utils/DijkstraSearchAlgorithm.hpp>
#include <pathfinding-utils/GridMapScenarioLoader.hpp>

#include <cpd-search/PerturbatedCost.hpp>

namespace pathfinding::map_perturbator::utils {

	using namespace pathfinding;
	using namespace pathfinding::maps;
	using namespace cpp_utils;
	using namespace cpp_utils::graphs;
	using namespace pathfinding::search;

	/**
	 * @brief a class which alter a map into something else
	 * 
	 */
	template <typename G, typename V>
	class AbstractPerturbator : public ICleanable {
	protected:
		/**
		 * @brief number of edges changed
		 * 
		 */
		int edgeChanged;
		/**
		 * @brief if true, we won't block edge cost changes which decreases the original weight of the edge
		 * 
		 */
		bool allowEdgeCostDecrements;
	public:
		AbstractPerturbator(bool allowEdgeCostDecrements): edgeChanged{0}, allowEdgeCostDecrements{allowEdgeCostDecrements} {
		}

		virtual ~AbstractPerturbator() {

		}

		/**
		 * @brief perturbe a graph
		 * 
		 * @param experimentId the experiment Id in a scenario of testing
		 * @param experiment the related experiment
		 * @param start nodeid_t where the search needs to start
		 * @param goal nodeid_t where the search needs to end
		 * @param originalGraph the graph we need to perturbate
		 * @return std::unique_ptr<AdjacentGraph<G, V, PerturbatedCost>> a copy of @c originalGraph containing the perturbation
		 */
		virtual AdjacentGraph<G, V, PerturbatedCost>* perturbateGraph(int experimentId, const GridMapScenarioExperiment& experiment, nodeid_t start, nodeid_t goal, const IImmutableGraph<G, V, cost_t>& originalGraph) = 0;
		/**
		 * @brief number of edges we have changed so far
		 * 
		 * @return int 
		 */
		int getEdgeChanged() const {
			return this->edgeChanged;
		}
	protected:
		/**
		 * @brief call this routing whenever you want to alter a graph
		 * 
		 * @param graph the graph that will contain the perturbation
		 * @param sourceId source id of the edge to alter
		 * @param sinkId sink id of the edge to alter
		 * @param perturbationEntity range of the eprturbation
		 * @param perturbationMode how we need to interpret @c perturbationEntity
		 * @param randomEngine random generator
		 */
		virtual void perturbateArc(INonExtendableGraph<G, V, PerturbatedCost>& graph, nodeid_t sourceId, nodeid_t sinkId, const Interval<int>& perturbationEntity, const std::string& perturbationMode, const Random& random) {

			if (graph.getEdge(sourceId, sinkId).isPerturbated()) {
				throw cpp_utils::exceptions::ImpossibleException{"arc %ld->%ld has already been perturbated!", sourceId, sinkId};
			}

			//ALTER SAID ARC

			int randomNumber = random.nextInt(perturbationEntity);

			//we need to change the arc
			cost_t oldWeight = graph.getEdge(sourceId, sinkId).getCost();
			cost_t newWeight = 0;

			if (perturbationMode == std::string{"ASSIGN"}) {
				newWeight = randomNumber;
			} else if (perturbationMode == std::string{"SUM"}) {
				newWeight = oldWeight + randomNumber;
			} else if (perturbationMode == std::string{"MULTIPLY"}) {
				newWeight = oldWeight * randomNumber;
			} else {
				throw cpp_utils::exceptions::InvalidScenarioException<std::string>{perturbationMode};
			}

			if (!this->allowEdgeCostDecrements) {
				if (newWeight < oldWeight) {
					log_error("the perturbation may only increase weights, not decrease them! we're tyring to change", sourceId, "->", sinkId, "from", oldWeight, "to", newWeight);
					throw cpp_utils::exceptions::makeInvalidArgumentException("can't replace weight", oldWeight, "with a smaller value ", newWeight, "!");
				}
			}

			graph.changeWeightUndirectedEdge(sourceId, sinkId, PerturbatedCost{newWeight, true});
			//we have changed both edges
			this->edgeChanged += 2;
		}

		virtual void perturbateArc(INonExtendableGraph<G, V, PerturbatedCost>& graph, nodeid_t sourceId, nodeid_t sinkId, const Interval<double>& perturbationEntity, const std::string& perturbationMode, const Random& random) {
			if (graph.getEdge(sourceId, sinkId).isPerturbated()) {
				throw cpp_utils::exceptions::ImpossibleException{"arc %ld->%ld has already been perturbated!", sourceId, sinkId};
			}

			//ALTER SAID ARC

			double randomNumber = random.nextDouble(perturbationEntity);
			critical("random number is", randomNumber, perturbationEntity);

			//we need to change the arc
			cost_t oldWeight = graph.getEdge(sourceId, sinkId).getCost();
			cost_t newWeight = 0;

			if (perturbationMode == std::string{"ASSIGN"}) {
				newWeight = static_cast<cost_t>(randomNumber);
			} else if (perturbationMode == std::string{"SUM"}) {
				newWeight = oldWeight + static_cast<cost_t>(randomNumber);
			} else if (perturbationMode == std::string{"MULTIPLY"}) {
				newWeight = static_cast<cost_t>(static_cast<double>(oldWeight) * randomNumber);
			} else {
				throw cpp_utils::exceptions::InvalidScenarioException<std::string>{perturbationMode};
			}

			if (!this->allowEdgeCostDecrements) {
				if (newWeight < oldWeight) {
					log_error("the perturbation may only increase weights, not decrease them! we're tyring to change", sourceId, "->", sinkId, "from", oldWeight, "to", newWeight);
					throw cpp_utils::exceptions::makeImpossibleException("can't replace the weight ", oldWeight, " with a smaller value", newWeight, "!");
				}
			}

			graph.changeWeightUndirectedEdge(sourceId, sinkId, PerturbatedCost{newWeight, true});
			//we have changed both edges
			this->edgeChanged += 2;
		}

		//virtual std::vector<xyLoc> getOptimalPath(const AdjGraph& graph, const Mapper& mapper, const xyLoc& start, const xyLoc& goal) const;
	public:
		void cleanup() {
			this->edgeChanged = 0;
		}
	};

}

#endif /* _EDGE_COST_CHANGES_TESTER_IPERTURBATOR_HEADER__ */
