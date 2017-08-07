//
// Created by Raymond Linn on 8/2/17.
//

#ifndef PATH_PLANNING_UTILITY_H
#define PATH_PLANNING_UTILITY_H

#include <vector>
#include <math.h>

using namespace std;

// structure for map data read from input map data
typedef struct mapData {
    vector<double> vx;
    vector<double> vy;
    vector<double> vs;
    vector<double> vdx;
    vector<double> vdy;
} mapData_t;

// structure for car data read from simulator input through json format
typedef struct carData {
    double x;
    double y;
    double s;
    double d;
    double yaw_d;
    double yaw_r;
    double speed;
} carData_t;

namespace utility
{
    // For converting back and forth between radians and degrees.
    constexpr double pi();

    double deg2rad(double x);

    //double rad2deg(double x);

    // calculate distance between two points (x1,y1) and (x2, y2)
    double distance(double x1, double y1, double x2, double y2);

    // get the local points near by in local coordinates
    vector<vector<double>> getLocalPointsNearBy(carData_t &stCarData, mapData_t &stMapData, double nextD);

    // convert map coorodinates to local car coordinates
    vector<double> getLocalXY(carData_t &stCarData, double mapX, double mapY);

    // convert local coordinates to map coordinates
    vector<double> getMapXY(carData_t &stCarData, double localX, double localY);

    // convert Map Coordinates to local coordinates
    vector<vector<double>> getLocalPoints(carData_t &stCarData, vector<double>mapXs, vector<double>mapYs);

    // convert local coordinates to map coordinates vector
    vector<vector<double>> getMapCoordPoints(carData_t &carData, vector<double> localXs, vector<double> localYs);

    // calculate a closet point from given (x, y) to the list of map points
    // maps_x and maps_y need to be the same size
    // return the index of map point that is closet to the point (x, y)
    int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

    // to find the next way point within the given (x, y) and angle theta from the map points
    // return the index of next way point
    //int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);


    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    //vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    //vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

}

#endif //PATH_PLANNING_UTILITY_H
