// Sandbox.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <algorithm>

#include "../../libraries/FileIO/easyio/easyio.h"
#include "../../libraries/Planning/AStar_grid.h"
#include "Matrix.h"


typedef std::map<int,std::map<int,AStar_grid*> > grid_lookup;


grid_lookup getMasks(Matrix<bool,2> *obstacle_map, Matrix<bool,2> *connection_map, Matrix<int,2> *membership_map){
	grid_lookup m2astar;
	for (int i=0; i<connection_map->dim1(); i++){
		for (int j=0; j<connection_map->dim2(); j++){
			if ((*connection_map)(i,j)>0){
				m2astar[i][j] = new AStar_grid(obstacle_map,membership_map,i,j);
			}
		}
	}
	return m2astar;
}


int main (int argc, char const *argv[]){


	matrix2d m = FileManip::readDouble("C:/Users/Carrie/Documents/Visual Studio 2012/Projects/projects/IROS2015/IROS2015/agent_map/obstacle_map.csv");
	matrix2d mems = FileManip::readDouble("C:/Users/Carrie/Documents/Visual Studio 2012/Projects/projects/IROS2015/IROS2015/agent_map/membership_map.csv");
	matrix2d conns = FileManip::readDouble("C:/Users/Carrie/Documents/Visual Studio 2012/Projects/projects/IROS2015/IROS2015/agent_map/connections.csv");
	matrix2d agentxy = FileManip::readDouble("C:/Users/Carrie/Documents/Visual Studio 2012/Projects/projects/IROS2015/IROS2015/agent_map/agent_map.csv");

	
	Matrix<bool,2> * obstacles = new Matrix<bool,2>(m.size(),m[0].size());
	Matrix<int, 2> * members = new Matrix<int,2>(mems.size(),mems[0].size());
	Matrix<bool,2> * connections = new Matrix<bool,2>(conns.size(), conns[0].size());

	for (int i=0; i<conns.size(); i++){
		for (int j=0; j<conns[0].size(); j++){
			(*connections)(i,j) = conns[i][j];
		}
	}

	for (int i=0; i<m.size(); i++){
		for (int j=0; j<m[i].size(); j++){
			obstacles->row(i)[j] = m[i][j]>0.0;
		}
	}


	for (int i=0; i<mems.size(); i++){
		for(int j=0; j<mems[i].size(); j++){
			members->row(i)[j] = mems[i][j];
		}
	}

	grid_lookup masks = getMasks(obstacles,connections,members);


	//AStar_grid a(obstacles);

	XY source(140,35);
	XY goal(180,56);

	int msource = (*members)(source.x,source.y);
	int mgoal = (*members)(goal.x,goal.y);

	
	
	std::vector<XY> fullpath = masks[msource][mgoal]->m.get_solution_path(source,goal);



	//std::vector<XY> fullpath = a.m.get_solution_path(source, goal);
	//int dist = a.m.solve(source.x,source.y,goal.x,goal.y);
	
	printf("Dist=%i",fullpath.size());
	system("pause");

	/*
	a.m.solve(0,29,agentxy[0][0],agentxy[0][1]);
	std::cout << "Soln length" << a.m.m_solution_length << std::endl;


	std::cout << a.m << std::endl;

	system("pause");
	exit(1);
	/**/

	/*matrix2d voronoi_mems(obstacles->size());
	for (int i=0; i<obstacles->size(); i++){
		voronoi_mems[i] = matrix1d(obstacles->at(i).size(),-1); // -1 indicates no membership (should only be obstacles
		for (int j=0; j<obstacles->at(i).size(); j++){
			if(!obstacles->at(i)[j]){
				printf("(%i,%i),",i,j);
				std::vector<double> dist(agentxy.size());
				for (int k=0; k<agentxy.size(); k++){
					dist[k] = a.m.solve(i,j,agentxy[k][1],agentxy[k][0]); // swapping the xy coordinates for agentxy: may be in different reference frame
				}
				// find the shortest distance
				std::vector<double>::iterator shortest_dist = min_element(dist.begin(),dist.end());
				if (*shortest_dist == DBL_MAX){
					voronoi_mems[i][j] = -2; // open space, but it can't get to any other sector
				} else {
					voronoi_mems[i][j] = std::distance(dist.begin(), shortest_dist);
				}
				printf("%f,%f\n",*shortest_dist,voronoi_mems[i][j]);
				
			}
		}
	}

	PrintOut::toFile2D(voronoi_mems,"voronoi_mems.csv");*/


	//AStar_grid a(obstacles,members,0,1);

	// make an A* grid: base
	// filter based on given set



	/*
	std::map<int,std::map<int,AStar_grid> > m2astar;
	for (int i=0; i<conns.size(); i++){
	for (int j=i+1; j<conns[i].size(); j++){
	if (conns[i][j]>0){
	m2astar[i].insert(std::make_pair(j,AStar_grid(obstacles,members,i,j)));
	}
	}
	std::cout << i <<std::endl;
	}
	*/

	//printf("%i Astars created!!",m2astar.size());


	/*
	std::vector<AStar_grid> astars;
	for (int i=0; i<1000; i++){
	astars.push_back(AStar_grid(obstacles));
	std::cout << i << std::endl;
	}
	*/

	//	a.m.solve(0,0,1,1);

	// The default maze size is 20x10.  A different size may be specified on
	// the command line.
	/*
	if (a.m.solve(0,0,1,1))
	std::cout << "Solved the maze." << std::endl;
	else
	std::cout << "The maze is not solvable." << std::endl;
	std::cout << a.m << std::endl;*/

	system("pause");
	return 0;
}
