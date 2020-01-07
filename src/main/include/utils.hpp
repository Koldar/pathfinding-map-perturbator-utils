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

}

#endif