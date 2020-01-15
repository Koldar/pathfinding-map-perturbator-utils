#ifndef _PATHFINDINGMAPPERTURBATORUTILS_UTILS__
#define _PATHFINDINGMAPPERTURBATORUTILS_UTILS__

#include <cpp-utils/adjacentGraph.hpp>
#include <boost/filesystem.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>
#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/GridBranching.hpp>

namespace pathfinding::map_perturbator::utils {

	using namespace cpp_utils;
	using namespace cpp_utils::graphs;
	using namespace pathfinding::maps;

	/**
	 * @brief load a map
	 * 
	 * by default we load a moving AI map with the following weights:
	 * 
	 * @li . is 1000
	 * @li T is 1500
	 * @li S is 2000
	 * @li W is 2500
	 * @li @ is invalicable
	 * 
	 * The map build is 8-connected
	 * 
	 * @param mapFilename  filename of the map to load
	 * @return AdjacentGraph<std::string, xyLoc, cost_t> the graph representing the map
	 */
	static AdjacentGraph<std::string, xyLoc, cost_t> loadMap(const boost::filesystem::path& mapFilename) {
		MovingAIGridMapReader reader{
			'.', 1000, color_t::WHITE,
			'T', 1500, color_t::GREEN,
			'S', 2000, color_t::CYAN,
			'W', 2500, color_t::BLUE,
			'@', cost_t::INFTY, color_t::BLACK
		};
		pathfinding::maps::GridMap gridMap = reader.load(mapFilename);
		GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
		return AdjacentGraph<std::string, xyLoc, cost_t>(*converter.toGraph(gridMap));
	}

	/**
	 * @brief Create a Grid Map Perturbated Map object
	 * 
	 * @tparam G 
	 * @tparam V 
	 * @tparam E 
	 * @param gridmap 
	 * @param baseMapGraph 
	 * @param perturbatedGraph 
	 * @param worseEdgeColor 
	 * @param betterEdgeColor 
	 * @return GridMapImage& 
	 */
	template <typename G, typename V, typename E>
	GridMapImage* createGridMapPerturbatedMap(const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, E>& perturbatedGraph, color_t worseEdgeColor, color_t betterEdgeColor, const std::function<cost_t(const E&)>& costFunction) {
		auto image = const_cast<GridMapImage*>(gridMap.getPPM());
		for (auto it=perturbatedGraph.beginEdges(); it!=perturbatedGraph.endEdges(); ++it) {
			if (!it->getPayload().isPerturbated()) {
				continue;
			}
			xyLoc source = perturbatedGraph.getVertex(it->getSourceId());
			xyLoc sink = perturbatedGraph.getVertex(it->getSinkId());
			int pixel1x, pixel1y, pixel2x, pixel2y;
			switch (xyLoc::getDirection(source, sink)) {
				case Direction::SOUTHEAST:  pixel1x=2; pixel1y=2; pixel2x=0; pixel2y=0; break;
				case Direction::EAST:       pixel1x=2; pixel1y=1; pixel2x=0; pixel2y=1; break;
				case Direction::NORTHEAST:  pixel1x=2; pixel1y=0; pixel2x=0; pixel2y=2; break;

				case Direction::SOUTHWEST:  pixel1x=0; pixel1y=2; pixel2x=2; pixel2y=0; break;
				case Direction::WEST:       pixel1x=0; pixel1y=1; pixel2x=2; pixel2y=1; break;
				case Direction::NORTHWEST:  pixel1x=0; pixel1y=0; pixel2x=2; pixel2y=2; break;
				
				case Direction::SOUTH:      pixel1x=1; pixel1y=2; pixel2x=1; pixel2y=0; break;
				case Direction::NORTH:      pixel1x=1; pixel1y=0; pixel2x=1; pixel2y=2; break;
				default:
					throw cpp_utils::exceptions::makeImpossibleException("invalid direction!");
			}
			cost_t oldWeight = baseMapGraph.getEdge(it->getSourceId(), it->getSinkId());
			cost_t newWeight = costFunction(perturbatedGraph.getEdge(it->getSourceId(), it->getSinkId()));
			color_t perturbatedColor;
			if (newWeight > oldWeight) {
				//cost has increased
				perturbatedColor = worseEdgeColor;
			} else {
				//cost has decreased
				perturbatedColor = betterEdgeColor;
			}
			image->setPixelInGrid(source.x, source.y, pixel1x, pixel1y, perturbatedColor); 
			image->setPixelInGrid(sink.x, sink.y, pixel2x, pixel2y, perturbatedColor);
		}

		return image;
	}

}

#endif