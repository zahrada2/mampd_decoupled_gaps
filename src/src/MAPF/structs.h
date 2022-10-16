#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include <list>
#include <map>
#include <set>

/* enum PBSversion{ */
/*     v0,  //brute-force */
/*     v1,  //prioritized */
/*     v2,  //reuse constraints */
/*     v2b, //reuse constraints and old paths */
/*     v3,  //reuse constraints + online learning */
/*     v3b  //reuse constraints and old paths + online learning */
/* }; */

struct Agent
{
    std::string id;
    int int_id;
    int curPOI_id;
    int start_i;
    int start_j;
    double start_heading;
    int goal_i;
    int goal_j;
    double goal_heading;
    double size;
    double rspeed;
    double mspeed;
    Agent(int s_i=-1, int s_j=-1, int g_i=-1, int g_j=-1){ start_i = s_i; start_j = s_j; goal_i = g_i; goal_j = g_j;
             size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; rspeed = CN_DEFAULT_RSPEED;
             start_heading = CN_DEFAULT_SHEADING; goal_heading = CN_DEFAULT_GHEADING; }
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct SafeInterval
{
    double begin;
    double end;
    int id;
    SafeInterval(double begin_=0, double end_=CN_INFINITY, int id_=0):begin(begin_), end(end_), id(id_) {}
};

struct Node
{
    int     id, i, j, f, g;
    Node*   parent;
    std::pair<int, int> interval;
    Node(int _i = -1, int _j = -1, int _f = -1, int _g = -1, Node* _parent = nullptr, int begin = -1, int end = -1)
        :i(_i), j(_j), f(_f), g(_g), parent(_parent), interval(std::make_pair(begin, end)) {id = i*1000 + j;}

    ~Node() { parent = nullptr; }
};

struct section
{
    section(int _i1=-1, int _j1=-1, int _i2=-1, int _j2=-1, double _g1=-1, double _g2=-1)
        :i1(_i1), j1(_j1), i2(_i2), j2(_j2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):i1(a.i), j1(a.j), i2(b.i), j2(b.j), g1(a.g), g2(b.g){}
    int i1;
    int j1;
    int i2;
    int j2;
    double size;
    double g1;
    double g2;//is needed for goal and wait actions
    double mspeed;
    bool operator == (const section &comp) const {return (i1 == comp.i1 && j1 == comp.j1 && g1 == comp.g1);}

};

struct Path
{
    bool pathfound;
    double pathlength;
    double TTD;
    double runtime;
    std::vector<Node> sections;
    int agent_id;
    int startPOI_ID;
    int goalPOI_ID;
    Path()
    {
        pathfound = false;
        runtime = 0;
        pathlength = 0;
        TTD = 0;
        agent_id = -1;
    }
};

using Ver = int;
struct MAPF_input {
    MAPF_input(){}
    std::vector<std::vector<Ver>> sequences;//a vector of sequences. Each sequence includes start, goal and assignements locations
    //some additional information, maybe previos sequences or made changes to be able to reuse previous plans
};

struct MAPF_solution
{
    MAPF_solution(){cost = CN_INFINITY;}
    std::vector<std::vector<Path>> paths;
    std::vector<Path> joined_paths;
    MAPF_input input;
    double cost;
    std::set<std::pair<int, int>> pbs_constraints;
};


struct Move
{
    int t;
    int i1, j1, i2, j2;//in case of wait action i1==i2, j1==j2
    Move(int _t = -1, int _i1 = -1, int _j1 = -1, int _i2 = -1, int _j2 = -1)
        : t(_t), i1(_i1), j1(_j1), i2(_i2), j2(_j2) {}
    Move(const Move& move) : t(move.t), i1(move.i1), j1(move.j1), i2(move.i2), j2(move.j2) {}
    Move(Node a, Node b) : t(a.g), i1(a.i), j1(a.j), i2(b.i), j2(b.j) {}
    bool operator <(const Move& other) const
    {
        if(i1 < other.i1)      return true;
        else if(i1 > other.i1) return false;
        else if(j1 < other.j1) return true;
        else if(j1 > other.j1) return false;
        else if(i2 < other.i2) return true;
        else if(i2 > other.i2) return false;
        else if(j2 < other.j2) return true;
        else                   return false;
    }
};

struct Conflict
{
    int agent1, agent2;
    int t;
    Move move1, move2;
    Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), int _t = -1)
        : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2) {}
};

struct PBS_Constraint
{
    int agent1;
    int agent2;
    int entries;
};

struct PBS_Node
{
    std::vector<Path> joined_paths;
    std::vector<std::vector<Path>> sequenced_paths;
    PBS_Node* parent;
    std::set<std::pair<int, int>> priorities;
    int cost;
    int id;
    int constraints_num;
    int colliding_agents;
    std::vector<Conflict> conflicts;
    PBS_Node(std::vector<Path> _jpaths = {}, std::vector<std::vector<Path>> _spaths = {}, PBS_Node* _parent = nullptr, std::set<std::pair<int, int>> _priorities = {}, double _cost = 0, int _constraints_num = 0, int _colliding_agents = 0, std::vector<Conflict> _conflicts = {})
        :joined_paths(_jpaths), sequenced_paths(_spaths), parent(_parent), priorities(_priorities), cost(_cost), constraints_num(_constraints_num), colliding_agents(_colliding_agents), conflicts(_conflicts) {}
    ~PBS_Node()
    {
        parent = nullptr;
        joined_paths.clear();
        sequenced_paths.clear();
    }
};

struct PBS_Open_Elem
{
    PBS_Node* tree_pointer;
    int cost;
    int colliding_agents;
    int constraints_num;

    PBS_Open_Elem(PBS_Node* _tree_pointer = nullptr, int _cost = -1, int _colliding_agents = -1, int _constraints_num = -1)
        : tree_pointer(_tree_pointer), cost(_cost), colliding_agents(_colliding_agents), constraints_num(_constraints_num) {}
    ~PBS_Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

class PBS_Tree
{
    std::list<PBS_Node> tree;
    std::vector<std::list<PBS_Open_Elem>> open;
    int open_size;
public:
    int get_open_size()
    {
        return open.size();
    }

    void clear()
    {
        tree.clear();
        open.clear();
        open_size = 0;
    }

    void add_node(PBS_Node node)
    {
        tree.push_back(node);
        bool inserted = false;
        for(int k = 0; k < open.size(); k++)
        {
            if(open[k].begin()->constraints_num == node.constraints_num)
            //if(open[k].begin()->cost - 2*open[k].begin()->constraints_num  + 3*open[k].begin()->colliding_agents == 3*node.colliding_agents + node.cost - 2*node.constraints_num)
            {
                PBS_Open_Elem elem(&tree.back(), node.cost, node.colliding_agents, node.constraints_num);
                for(auto it = open[k].begin(); it != open[k].end(); it++)
                    if(it->colliding_agents > elem.colliding_agents || (it->colliding_agents == elem.colliding_agents && it->cost > elem.cost)
                        || (it->colliding_agents == elem.colliding_agents && it->cost == elem.cost && it->tree_pointer->id < elem.tree_pointer->id))
                    {
                        inserted = true;
                        open[k].insert(it, elem);
                        break;
                    }
                if(!inserted)
                {
                    open[k].push_back(elem);
                    inserted = true;
                }
                break;
            }
            else if (open[k].begin()->constraints_num < node.constraints_num)
            //else if(open[k].begin()->cost - 2*open[k].begin()->constraints_num  + 3*open[k].begin()->colliding_agents > 3*node.colliding_agents + node.cost - 2*node.constraints_num)
            {
                std::list<PBS_Open_Elem> open_elem = {PBS_Open_Elem(&tree.back(), node.cost, node.colliding_agents, node.constraints_num)};
                open.insert(open.begin() + k, open_elem);
                inserted = true;
                break;
            }
        }
        if(!inserted)
        {
            std::list<PBS_Open_Elem> open_elem = {PBS_Open_Elem(&tree.back(), node.cost, node.colliding_agents, node.constraints_num)};
            open.push_back(open_elem);
        }
        open_size++;
    }

    PBS_Node* get_front()
    {
        return open[0].begin()->tree_pointer;
    }

    void pop_front()
    {
        open[0].pop_front();
        if(open[0].empty())
            open.erase(open.begin());
        open_size--;
        return;
    }

    std::vector<Path> get_joined_paths(PBS_Node node, int size)
    {
        std::vector<Path> paths(size);
        while(node.parent != nullptr)
        {
            if(paths.at(node.joined_paths.begin()->agent_id).sections.empty())
                paths.at(node.joined_paths.begin()->agent_id) = *node.joined_paths.begin();
            node = *node.parent;
        }
        for(unsigned int i = 0; i < node.joined_paths.size(); i++)
            if(paths.at(i).sections.empty())
                paths.at(i) = node.joined_paths.at(i);
        return paths;
    }

    std::vector<std::vector<Path>> get_sequenced_paths(PBS_Node node, int size)
    {
        std::vector<std::vector<Path>> paths(size);
        while(node.parent != nullptr)
        {
            if(paths[node.sequenced_paths.begin()->begin()->agent_id].empty())
                paths[node.sequenced_paths.begin()->begin()->agent_id] = *node.sequenced_paths.begin();
            node = *node.parent;
        }
        for(unsigned int i = 0; i <node.sequenced_paths.size(); i++)
            if(paths[i].empty())
                paths[i] = node.sequenced_paths[i];
        return paths;
    }
};

struct Config
{
    int connectedness;
    bool allowanyangle;
    bool planforturns;
    double timelimit;
    int rescheduling;
    double inflatecollisionintervals;
    int initialprioritization;
    double startsafeinterval;
    double additionalwait;

    Config()
    {
        connectedness = CN_DEFAULT_CONNECTEDNESS;
        allowanyangle = CN_DEFAULT_ALLOWANYANGLE;
        planforturns = CN_DEFAULT_PLANFORTURNS;
        timelimit = CN_DEFAULT_TIMELIMIT;
        rescheduling = CN_DEFAULT_RESCHEDULING;
        inflatecollisionintervals = CN_DEFAULT_INFLATEINTERVALS;
        initialprioritization = CN_DEFAULT_INITIALPRIORITIZATION;
        startsafeinterval = CN_INFINITY;//CN_DEFAULT_STARTSAFEINTERVAL;
        additionalwait = CN_DEFAULT_ADDITIONALWAIT;
    }
};

struct Vector2D {
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

struct Point {
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};
#endif
