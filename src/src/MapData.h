#ifndef MAPDATA_H
#define MAPDATA_H

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <queue>
#include <cmath>

#define NEIGHBORHOOD_8 false

class MapData {
  private:
	struct STask {
		int start_x, start_y;
		int goal_x, goal_y;
        STask(int start_x, int start_y, int goal_x, int goal_y) : start_x(start_x), start_y(start_y), goal_x(goal_x), goal_y(goal_y){}
	};
    
    std::vector<int> waiting_times;

	int countFiles(std::string path);

	void readScenarios(std::string path);

	void readScenario(std::string name, std::vector<STask> &tasks);

  public:
	struct Map {
	  private:
		int height;
		int width;
		double *data;
        std::vector<std::pair<int,int>> coords;
		bool loaded = false;

	  public:

    void freeData(){
      delete[] data;
    }
		/**
		 * @brief Map data structure. Directly parses input file in constructor.
		 * @param map_name Path to an input file in MovingAI Benchmark format.
		 */
		Map(std::string map_name) {
			std::ifstream infile(map_name.c_str(), std::ios_base::in);
			std::string line;
			std::string a,b;

			std::cout << "Map::Map: Loading map " << map_name << "." << std::endl;

			for (int i = 1; i < 5; i++) {
				getline(infile, line, '\n');
				std::stringstream strStream(line);
				strStream >> a >> b;
				switch (i) {
					case 1:
						// type
						break;
					case 2:
						height = std::stoi(b);
						break;
					case 3:
						width = std::stoi(b);
            //TODO: this causes a tiny memory leak
						data = new double[height * width];
						coords = std::vector<std::pair<int,int>>(height * width);
						break;
					case 4:
						break;
				}
			}

			int nid, x, y;
			nid = x = y = 0;
			// load data
			while (getline(infile, line, '\n')) {
				for (size_t k = 0; k < line.size(); k++) {
					data[nid] = line[k] == '.' ? 0 : 1;
					coords[nid] = {x,y};
					nid++;
					x++;
				}
				y++;
				x = 0;
			}

            loaded = true;
			std::cout << "Map::Map: Loaded map with height " << height << " and width " << width << "." << std::endl;
		}



		// getters
		int get_height() const {
			if (!loaded) std::cerr << "Map::get_height:: ERROR: Map not loaded!" << std::endl;
			return height;
		}
		int get_width() const {
			if (!loaded) std::cerr << "Map::get_width:: ERROR: Map not loaded!" << std::endl;
			return width;
		}
		int get_id(int x, int y) const {
			if (!loaded) std::cerr << "Map::get_id:: ERROR: Map not loaded!" << std::endl;
			return y * width + x;
		}
		std::pair<int, int> get_coord(int id) const {
			if (!loaded) std::cerr << "Map::get_coord:: ERROR: Map not loaded!" << std::endl;
			return coords[id];
		}
		double get_value(int x, int y) const {
			if (!loaded) std::cerr << "Map::get_value:: ERROR: Map not loaded!" << std::endl;
			return data[y * width + x];
		}
		double get_value(int id) const {
			if (!loaded) std::cerr << "Map::get_value:: ERROR: Map not loaded!" << std::endl;
			auto coord = coords[id];
			return data[coord.second * width + coord.first];
		}

	};

	struct DistanceMatrix {
		using Node = std::pair<int, int>;
	  private:
		Map map;
		std::vector<std::vector<double>> distances;
		bool initialized;

		class NodeCostComparator {
		public:
			bool operator()(const std::pair<double, Node>& x, const std::pair<double, Node>& y) {
				return x.second > y.second;
			}
		};


		std::vector<Node> adjacent(Node node, bool N8 = false){
			std::vector<Node> adj;
			if (N8) {
				if (node.first + 1 < map.get_width() && node.second + 1 < map.get_height() && map.get_value(node.first + 1, node.second + 1) < 0.5){
					adj.push_back({node.first + 1, node.second + 1});
				}
				if (node.first - 1 >= 0 && node.second - 1 >= 0 && map.get_value(node.first - 1, node.second - 1) < 0.5){
					adj.push_back({node.first - 1, node.second - 1});
				}
				if (node.first - 1 >= 0 && node.second + 1 < map.get_height() && map.get_value(node.first - 1, node.second + 1) < 0.5){
					adj.push_back({node.first - 1, node.second + 1});
				}
				if (node.first + 1 < map.get_width() && node.second - 1 >= 0 && map.get_value(node.first + 1, node.second - 1) < 0.5){
					adj.push_back({node.first + 1, node.second - 1});
				}
			} else {
				if (node.first + 1 < map.get_width() && map.get_value(node.first + 1, node.second) < 0.5){
					adj.push_back({node.first + 1, node.second});
				}
				if (node.first - 1 >= 0 && map.get_value(node.first - 1, node.second) < 0.5){
					adj.push_back({node.first - 1, node.second});
				}
				if (node.second + 1 < map.get_height() && map.get_value(node.first, node.second + 1) < 0.5){
					adj.push_back({node.first, node.second + 1});
				}
				if (node.second - 1 >= 0 && map.get_value(node.first, node.second - 1) < 0.5){
					adj.push_back({node.first, node.second - 1});
				}
			}
			return adj;
		}

		std::vector<double> compute_dijkstra(Node s){
			std::priority_queue<std::pair<double, Node>, std::vector<std::pair<double, Node>>, NodeCostComparator> pq;
			auto & dist = distances[map.get_id(s.first, s.second)];

			pq.push({0, s});
			int start_id = map.get_id(s.first, s.second);
			//std::cout << "   from " << s.first << ";" << s.second << " id: " << start_id << std::endl;
			dist[start_id] = 0;

			while (!pq.empty()) {
				Node u = pq.top().second;
				auto const u_d = dist[map.get_id(u.first, u.second)];
				pq.pop();

				std::vector<Node> adj = adjacent(u);
				for (Node &v : adj) {
					auto & v_d = dist[map.get_id(v.first, v.second)];
					if (v_d > u_d + 1) {
						v_d = u_d + 1;
						pq.push({u_d + 1, v});
					}
				}

#if NEIGHBORHOOD_8
				std::vector<Node> adj8 = adjacent(u, true);
				for (Node &v : adj8) {
					auto & v_d = dist[map.get_id(v.first, v.second)];
					if (v_d > u_d + 1.41421356237) {
						v_d = u_d + 1.41421356237;
						pq.push({u_d + 1, v});
					}
				}
#endif
			}

			return dist;
		}

	  public:

    void freeData(){
      map.freeData();
    }
		/**
		 * @brief DistanceMatrix data structure. Initialization via compute() required.
		 * @param map_ Map structure to build on.
		 * @param pois_ Points between which the distances are computed.
		 */
		DistanceMatrix(Map map_) :
			map (map_),
			distances (map.get_width() * map.get_height(), std::vector<double>(map.get_width() * map.get_height(), INFINITY)),
			initialized (false)
		{}

		/**
		 * @brief compute Fill the distance matrix. Iterates over given points of interest and uses Dijkstra
		 * to find the shortest path length to all other nodes. Second loop notes the distance from upper loop node to this one.
		 */
		void compute() {
			if (!initialized) {
				int max_id = map.get_width() * map.get_height();
				for (int id = 0; id < max_id; id++) {
					// if wall, skip
					if (map.get_value(id) > 0.5) continue;
					// compute dijkstra to all
					auto coord = map.get_coord(id);
					//std::cout << "dijkstra from " << coord.first << ";" << coord.second << ", id: " << id << ":" << std::endl;
					compute_dijkstra(coord);
				}
				initialized = true;
				/* for (int a = 0; a < max_id; a++) { */
				/* 	for (int b = 0; b < max_id; b++) { */
				/* 		std::cout << map.get_coord(a).first << ";" << map.get_coord(a).second << "->" << map.get_coord(b).first << ";" << map.get_coord(b).second << ": " << std::to_string(distances[a][b]) << std::endl; */
				/* 	} */
				/* } */
			}
		}

		/**
		 * @brief get_value Retrieve the real distance between two nodes in map. Initialization with compute() required prior to use.
		 * @param a node id
		 * @param b node id
		 * @return The length of the closest path between a and b.
		 */
		double get_value(int a, int b) const {
			if (initialized) {
				if (a < distances.size() && b < distances.size()) {
					return distances[a][b];
				} else {
					std::cout << "DistanceMatrix::get_value: Distance between " << a << " and " << b << " is undefined!" << std::endl;
					return -1;
				}
			} else {
				std::cout << "DistanceMatrix::get_value: Distance matrix has not been initialized! (use compute())" << std::endl;
				return -1;
			}
		}

		/**
		 * @brief get_value Retrieve the real distance between two nodes in map. Initialization with compute() required prior to use.
		 * @param x_a node coordinates
		 * @param y_a node coordinates
		 * @param x_b node coordinates
		 * @param y_b node coordinates
		 * @return The length of the closest path between a and b.
		 */
		double get_value(int x_a, int y_a, int x_b, int y_b) const {
			return get_value(map.get_id(x_a, y_a), map.get_id(x_b, y_b));
		}

	};

	MapData(std::string map_name);

	/**
	 * @brief getMap Get the MapData::Map struture, which holds the data about the traversability of nodes, as well as conversions between ids and coordinates.
	 * @return
	 */
	Map getMap() const;
	int getMapWidth() const;
    int getMapHeight() const;
    int getNodeId(int x, int y) const;
    std::pair<int, int> getNodeCoords(int id) const;
    std::vector<std::pair<int, int>> getValidMoves(int i, int j) const {return valid_moves[i][j];}
    bool cellOnGrid(int x, int y) const;
    bool cellIsTraversable(int x, int y) const;

    int getWaitTime(int POI_id);

    void setWaitTime(std::vector<int> waitings);

	/**
	 * @brief getPOI Get the set of points of interest based on the scenarios for the gridmap. Returns either vector of coordinates or ids.
	 */
	std::vector<std::pair<int,int>> getPOICoords();
	std::vector<int> getPOIIDs();

  // TTD modifications
  std::vector<double> minimalDepotDistances;
  bool distancesLoaded = false;

  double getMinimalDepotDistance(int poi_id);

  void setMinimalDepotDistances(std::vector<double> distances);

	/**
	 * @brief setPOI Set the set of points of interest.
	 */
	void setPOICoords(std::vector<std::pair<int,int>> pois);

	/**
	 * @brief getDistanceMatrix Get MapData::DistanceMatrix structure, which is already initialized and ready for querries for distances between nodes on the gridmap.
	 */
	DistanceMatrix getDistanceMatrix();

  private:
    void generate_valid_moves();
	Map gridMap;
	DistanceMatrix distanceMatrix;
	std::vector<std::pair<int,int>> poiCoords;
    std::vector<std::vector<std::vector<std::pair<int, int>>>> valid_moves;
    std::vector<int> poiIDs;
    std::vector<double> poiMinimalDepotDistances;
	bool gridmapLoaded = false;
	bool poisLoaded = false;
};

#endif // MAPDATA_H
