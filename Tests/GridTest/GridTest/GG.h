// Copyright 2016 Carrie Rebhuhn

/*
* 
*/
#ifndef PLANNING_GRIDGRAPH_H_
#define PLANNING_GRIDGRAPH_H_

// from boost
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/unordered_map.hpp>

// from stl
#include <float.h>
#include <fstream>
#include <utility>
#include <string>
#include <vector>
#include <functional>

// from libraries
#include "Math/easymath.h"
#include "Planning/IBoostGraph.h"
#include "Planning/Planning.h"
//#include "Planning\GridGraph.h"

/**
* This is a specialization of a graph for an 8-connected grid.
*/
typedef boost::grid_graph<2> grid;

// A hash function for vertices.
struct vertex_hash :std::unary_function<grid::vertex_descriptor, std::size_t> {
    std::size_t operator()(grid::vertex_descriptor const& u) const {
        return this->operator()(u[0], u[1]);
    }
    std::size_t operator()(size_t x, size_t y) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, x);
        boost::hash_combine(seed, y);
        return seed;
    }
};


struct vertex_equal {
    bool operator()(const grid::vertex_descriptor &rhs, const grid::vertex_descriptor &lhs) const {
        return rhs[0] == lhs[0] && rhs[1] == lhs[1];
    }
};

// graph -- should this be grid? filtered?
typedef IBoostGraph<grid, easymath::XY, vertex_hash, vertex_equal> GridBase;
class GridGraph: public GridBase {
public:
    // Distance traveled in the GridGraph
    typedef std::pair<int, int> edge;
    typedef std::vector<std::vector<bool> > barrier_grid;

    typedef boost::grid_graph<2> grid;
    typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

    // A hash function for vertices.
    struct vertex_hash :std::unary_function<vertex_descriptor, std::size_t> {
        std::size_t operator()(vertex_descriptor const& u) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, u[0]);
            boost::hash_combine(seed, u[1]);
            return seed;
        }
    };

    typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
    typedef std::vector<vertex_descriptor> vertex_vector;
    typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
        filtered_grid;

    // Euclidean heuristic
    struct euclidean_heuristic :
        public boost::astar_heuristic<filtered_grid, double> {
        euclidean_heuristic() {}
        double operator()(vertex_descriptor v) {
            double dx = static_cast<double>(m_goal[0] - v[0]);
            double dy = static_cast<double>(m_goal[1] - v[1]);
            return sqrt(dx*dx + dy*dy);
        }
        vertex_descriptor m_goal;
    };

    // Types required for boost A* use
    typedef typename GridBase::vertex_descriptor vertex_descriptor;
    typedef typename GridBase::dist_map dist_map;
    typedef typename GridBase::pred_map pred_map;

    
    GridGraph & operator =(const GridGraph &) {
        return *this;
    }
    ~GridGraph() {}

    vertex_descriptor get_descriptor(easymath::XY pt) {
        return{ static_cast<size_t>(pt.x), static_cast<size_t>(pt.y) };
    }
    easymath::XY get_vertex_base(vertex_descriptor v) {
        return easymath::XY(static_cast<double>(v[0]), static_cast<double>(v[1]));
    }
    double get_x(vertex_descriptor v) { return static_cast<double>(v[0]); }
    double get_y(vertex_descriptor v) { return static_cast<double>(v[1]); }

    // Exception thrown when the goal vertex is found
    struct found_goal {};

    // Visitor that terminates when we find the goal vertex
    struct astar_goal_visitor :public boost::default_astar_visitor {
        // astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {};
        astar_goal_visitor() {}

        void examine_vertex(vertex_descriptor u, const filtered_grid&) {
            if (u == m_goal)
                throw found_goal();
        }

        vertex_descriptor m_goal;
    };

    explicit GridGraph(barrier_grid obstacle_map) :
        m_grid(create_grid(obstacle_map.size(), obstacle_map[0].size())),
        g(create_barrier_grid()),
        m_solution_length(0) {
        int v_index = 0;
        for (size_t y = 0; y < obstacle_map[0].size(); y++) {
            for (size_t x = 0; x < obstacle_map.size(); x++) {
                if (obstacle_map[x][y]) {  // there is an obstacle!
                    vertex_descriptor u = vertex(v_index, m_grid);
                    m_barriers.insert(u);  // insert a barrier!
                }
                v_index++;  // increment v_index even if no barrier added!
            }
        }
    }

    GridGraph(const matrix2d &members, int m1, int m2) :
        m_grid(create_grid(members.size(), members[0].size())),
        g(create_barrier_grid()) {
        barrier_grid obstacle_map(members.size());
        for (size_t i = 0; i < members.size(); i++) {
            for (size_t j = 0; j < members[i].size(); j++) {
                obstacle_map[i][j] = members[i][j] < 0;
            }
        }

        // This map only shows grid cells of membership m1 and m2 as passable.
        // Others are barriers.
        // Backflow (travel from m2 to m1) is allowed but will tend to be
        // suboptimal, so is improbable.
        int v_index = 0;
        for (size_t y = 0; y < obstacle_map[0].size(); y++) {
            for (size_t x = 0; x < obstacle_map.size(); x++) {
                if (obstacle_map[x][y] ||
                    // there is an obstacle, or wrong membership
                    (members[x][y] != m1 && members[x][y] != m2)) {
                    vertex_descriptor u = vertex(v_index, m_grid);
                    m_barriers.insert(u);  // insert a barrier!
                }
                v_index++;
            }
        }
    }

    // The length of the AStarGrid along the specified dimension.
    vertices_size_type length(std::size_t d) const {
        return m_grid.length(d);
    }

    double solve(size_t xsource, size_t ysource, size_t xgoal, size_t ygoal) {
        boost::static_property_map<double> weight(1);
        // The predecessor map is a vertex-to-vertex mapping.
        typedef boost::unordered_map<vertex_descriptor,
            vertex_descriptor,
            vertex_hash> pred_map;
        pred_map predecessor;
        boost::associative_property_map<pred_map> pred_pmap(predecessor);

        // The distance map is a vertex-to-distance mapping.
        typedef boost::unordered_map<vertex_descriptor,
            double, vertex_hash> dist_map;
        dist_map dmap;
        boost::associative_property_map<dist_map> dist_pmap(dmap);

        vertex_descriptor s = { { xsource, ysource } };
        vertex_descriptor gl = { { xgoal, ygoal } };
        heuristic.m_goal = gl;
        visitor.m_goal = gl;
        m_solution.clear();

        try {
            astar_search(g, s, heuristic,
                boost::weight_map(weight).
                predecessor_map(pred_pmap).
                distance_map(dist_pmap).
                visitor(visitor));
        }
        catch (found_goal fg) {
            (void)fg;
            // Walk backwards from the goal through the predecessor chain adding
            // vertices to the solution path.
            for (vertex_descriptor u = gl; u != s; u = predecessor[u])
                m_solution.push_back(u);
            m_solution.push_back(s);
            m_solution_length = dmap[gl];
            return m_solution_length;
        }
        double maxdist = DBL_MAX;
        return maxdist;
    }

    std::vector<easymath::XY> astar(easymath::XY source, easymath::XY goal) {
        std::cout << "going to " << goal.x << "," << goal.y << "\n";
        size_t xs = static_cast<size_t>(source.x);
        size_t ys = static_cast<size_t>(source.y);
        size_t xg = static_cast<size_t>(goal.x);
        size_t yg = static_cast<size_t>(goal.y);
        solve(xs, ys, xg, yg);

        std::vector<easymath::XY> soln;
        // for (vertex_vector::iterator it
        // = m_solution.begin(); it != m_solution.end(); ++it) {
        for (boost::array<size_t, 2U> it : m_solution) {
            // soln.push_back(easymath::XY(it->front(), it->back()));
            soln.push_back(easymath::XY(it.front(), it.back()));
        }
        std::reverse(soln.begin(), soln.end());
        return soln;
    }

    bool solved() const { return !m_solution.empty(); }
    bool solution_contains(vertex_descriptor u) const {
        return std::find(m_solution.begin(), m_solution.end(), u)
            != m_solution.end();
    }

    euclidean_heuristic heuristic;
    astar_goal_visitor visitor;

    double m_solution_length;  // The length of the solution path
    vertex_vector m_solution;  // The vertices on a solution path through the AStarGrid

//private:
    // Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y) {
        boost::array<std::size_t, 2> lengths = { { x, y } };
        return grid(lengths);
    }
    // Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid() {
        return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
    }

    grid m_grid;  //! The grid underlying the AStarGrid
    filtered_grid g;  //! The underlying AStarGrid grid with barrier vertices filtered out
    vertex_set m_barriers;  //! The barriers in the AStarGrid
};
#endif  // PLANNING_GRIDGRAPH_H_