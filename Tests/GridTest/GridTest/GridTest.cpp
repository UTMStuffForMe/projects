// GridTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include "FileIO/FileIn.h"
#include "GG.h"

using namespace easymath;
using namespace std;
int main()
{
    matrix2d m = easyio::read2<double>("C:/Users/Carrie/Source/Repos/projects/IROS2015/IROS2015/agent_map/membership_map.csv");
    GridGraph gl(m < 0);
    XY a(200, 40);
    XY b(220, 45);

    cout << m[a.x][a.y];
    cout << m[b.x][b.y];

    system("pause");


    auto pth = gl.astar(a,b);
    for (auto l : pth) {
        cout << l.x << " "<< l.y << ", ";
    }

    auto pth2 = Planning::astar(&gl, a, b);
    for (auto l : pth2) {
        cout << l.x << " " << l.y << ", ";
    }

    system("pause");
    return 0;
}

