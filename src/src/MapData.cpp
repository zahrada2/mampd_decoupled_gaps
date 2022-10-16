#include "MapData.h"

MapData::MapData(std::string map_name):
	gridMap(map_name),
	distanceMatrix(gridMap)
{
	distanceMatrix.compute();
    gridmapLoaded = true;
    generate_valid_moves();
	//readScenarios(map_name); //substituted by other source
}


void MapData::generate_valid_moves()
{
    std::vector<std::pair<int, int>> moves;
    valid_moves.resize(gridMap.get_height());
    for(int i = 0; i < gridMap.get_height(); i++)
        valid_moves[i].resize(gridMap.get_width());
    moves = {{0,1}, {1,0}, {-1,0}, {0,-1}};
    for(int i = 0; i < gridMap.get_height(); i++)
        for(int j = 0; j < gridMap.get_width(); j++)
        {
            std::vector<bool> valid(moves.size(), true);
                for(int k = 0; k < moves.size(); k++)
                    if(!cellOnGrid(j + moves[k].second, i + moves[k].first) || !cellIsTraversable(j + moves[k].second, i + moves[k].first))
                        valid[k] = false;
            std::vector<std::pair<int, int>> v_moves = {};
            for(int k = 0; k < valid.size(); k++)
                if(valid[k])
                    v_moves.push_back(moves[k]);
            valid_moves[i][j] = v_moves;
        }
}

int MapData::countFiles(std::string path) {
	DIR *dp;
	int i = 0;
	struct dirent *ep;
	dp = opendir(path.data());

	if (dp != NULL) {
		while (ep = readdir(dp)) {
			i++;
		}
		(void)closedir(dp);
	} else {
		perror("MapParser::countFiles: Couldn't open the directory");
	}

	return i;
}

void MapData::readScenarios(std::string map_name) {
	std::vector<STask> tasks;
	std::ofstream assignments_f;

	std::string scen_path = "/scenarios/" + map_name + "/scen-even/";
	int scenario_count = countFiles(scen_path);

	for (int i = 1; i < scenario_count; i++) {
		tasks.clear();
		readScenario(scen_path + map_name + "-even-" + std::to_string(i) + ".scen", tasks);
		for (auto &task : tasks) {
			// TODO: Just a workaround for now
			poiCoords.push_back({task.goal_x, task.goal_y});
			poiIDs.push_back(gridMap.get_id(task.goal_x, task.goal_y));
		}
	}
}

void MapData::readScenario(std::string name, std::vector<STask> &tasks) {
	std::ifstream infile(name.c_str(), std::ios_base::in);
	std::string line;
	int bucket, map_width, map_height, start_x, start_y, goal_x, goal_y;
	std::string map;
	double optimal_length;
	std::vector<STask> result;

	std::cout << "MapParser::readScenario: Loading scenario \"" << name << "\"..." << std::endl;

	float ver;
	std::string first;
	infile >> first;

	// Check if a version number is given
	if (first != "version") {
		ver = 0.0;
		infile.seekg(0, std::ios::beg);
	} else {
		infile >> ver;
	}

	std::cout << "MapParser::readScenario: Scenario version is " << ver << "." << std::endl;
	if (ver != 1.0) {
		std::cout << "MapParser::readScenario: Wrong version of scenario file!" << std::endl;
	}

	while (infile >> bucket >> map >> map_width >> map_height >> start_x >> start_y >> goal_x >> goal_y >> optimal_length) {
		tasks.push_back(STask(start_x, start_y, goal_x, goal_y));
	}

	std::cout << "MapParser::readScenario: Loaded " << tasks.size() << " tasks." << std::endl;
}

MapData::Map MapData::getMap() const {
	if (!gridmapLoaded) std::cerr << "MapData::getMap:: ERROR: Gridmap was not loaded!" << std::endl;
	return gridMap;
}

int MapData::getMapWidth() const {
	if (!gridmapLoaded) std::cerr << "MapData::getMapWidth:: ERROR: Gridmap was not loaded!" << std::endl;
	return gridMap.get_width();
}

int MapData::getMapHeight() const {
	if (!gridmapLoaded) std::cerr << "MapData::getMapHeight:: ERROR: Gridmap was not loaded!" << std::endl;
	return gridMap.get_height();
}
int MapData::getNodeId(int x, int y) const {
	if (!gridmapLoaded) std::cerr << "MapData::getNodeId:: ERROR: Gridmap was not loaded!" << std::endl;
    return gridMap.get_id(x,y);
}

std::pair<int, int> MapData::getNodeCoords(int id) const {
	if (!gridmapLoaded) std::cerr << "MapData::getNodeCoords:: ERROR: Gridmap was not loaded!" << std::endl;
    return gridMap.get_coord(id);
}

bool MapData::cellOnGrid(int x, int y) const
{
	if (!gridmapLoaded) std::cerr << "MapData::cellOnGrid:: ERROR: Gridmap was not loaded!" << std::endl;
    return (x >= 0 && x < gridMap.get_width() && y >= 0 && y < gridMap.get_height());
}

bool MapData::cellIsTraversable(int x, int y) const
{
	if (!gridmapLoaded) std::cerr << "MapData::cellIsTraversable:: ERROR: Gridmap was not loaded!" << std::endl;
    return gridMap.get_value(x,y) == 0;
}

std::vector<std::pair<int,int>> MapData::getPOICoords() {
	if (!poisLoaded) std::cerr << "MapData::getPOICoords:: ERROR: POIs were not loaded!" << std::endl;
	return poiCoords;
}

std::vector<int> MapData::getPOIIDs() {
	if (!poisLoaded) std::cerr << "MapData::getPOIIDs:: ERROR: POIs were not loaded!" << std::endl;
	return poiIDs;
}

// TTD modifications

double MapData::getMinimalDepotDistance(int poi_id)
{
    if (!poisLoaded) std::cerr << "MapData::getPOIIDs:: ERROR: POIs were not loaded!" << std::endl;
    if (!distancesLoaded){
      std::cerr << "Distances not yet loaded!" << std::endl;
      return 0.0;
    } else {
      /* std::cout << "poi id: " << poi_id << std::endl; */
      /* std::cout << "distances size: " << poiMinimalDepotDistances.size() << std::endl; */
      return poiMinimalDepotDistances[poi_id];
      /* return minimalDepotDistances[poi_id]; */
    }
}

void MapData::setMinimalDepotDistances(std::vector<double> distances)
{
    /* minimalDepotDistances = distances; */ 
    this->poiMinimalDepotDistances = distances;
    distancesLoaded = true;
    /* if (!poisLoaded) std::cerr << "MapData::getPOIIDs:: ERROR: POIs were not loaded!" << std::endl; */
    /* return poiMinimalDepotDistances[poi_id]; */
}

void MapData::setPOICoords(std::vector<std::pair<int, int>> pois) {
	poiCoords = pois;
	for (int i = 0; i < pois.size(); i++) {
		poiIDs.push_back(i);
	}
	poisLoaded = true;
    /* poiMinimalDepotDistances.resize(pois.size(), 0); */
    //TODO - compute minimal depot distances
    // Separate method for now
}

int MapData::getWaitTime(int POI_id){
    return waiting_times[POI_id];
}

void MapData::setWaitTime(std::vector<int> waitings) {
    waiting_times = waitings;
}

MapData::DistanceMatrix MapData::getDistanceMatrix() {
	if (!gridmapLoaded) std::cerr << "MapData::getDistanceMatrix:: ERROR: Gridmap was not loaded!" << std::endl;
	return distanceMatrix;
}
