//
// Created by Raymond Linn on 8/2/17.
//
#include <cmath>
#include "../header/utility.h"

using namespace std;
namespace utility
{
    constexpr double pi() {
      return M_PI;
    }

    double deg2rad(double x) {
      return x * pi() / 180;
    }
/*
    double rad2deg(double x) {
      return x * 180 / pi();
    }
*/
    // calculate distance between two points (x1,y1) and (x2, y2)
    double distance(double x1, double y1, double x2, double y2)
    {
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    // get the local points near by in local coordinates
    // try to set 25 points total with 6 from the back and 18 in the front with 1 closet to the car location
    vector<vector<double>> getLocalPointsNearBy(carData_t &stCarData, mapData_t &stMapData, double nextD) {

      vector<double> localXs;
      vector<double> localYs;
      vector<vector<double>> localPts;

      int closestWaypoint = ClosestWaypoint(stCarData.x, stCarData.y, stMapData.vx, stMapData.vy);

      // 6 points fro the back
      int previous = closestWaypoint - 6;
      if (previous < 0) {
        previous += stMapData.vx.size();
      }

      // total of 25 points
      // points are in map coordinates, convert them to local car coordinates
      // questions and answers discussed by John Chen and the others in slack - p-path-planning channel
      for (int i = 0; i < 25; i++) {
        int next = (previous+i) % stMapData.vx.size();
        vector<double> localxy = getLocalXY(stCarData,
                                            (stMapData.vx[next] + (nextD * stMapData.vdx[next])),
                                            (stMapData.vy[next] + (nextD * stMapData.vdy[next])));

        localXs.push_back(localxy[0]);
        localYs.push_back(localxy[1]);
      }

      localPts.push_back(localXs);
      localPts.push_back(localYs);

      return localPts;
    }

    // convert map coorodinates to local car coordinates
    vector<double> getLocalXY(carData_t &stCarData, double mapX, double mapY) {

      vector<double> localPt;

      float deltaX = mapX - stCarData.x;
      float deltaY = mapY - stCarData.y;
      localPt.push_back((deltaX * cos(stCarData.yaw_r)) + (deltaY * sin(stCarData.yaw_r)));
      localPt.push_back((-deltaX * sin(stCarData.yaw_r)) + (deltaY * cos(stCarData.yaw_r)));

      return localPt;
    }


    // convert local coordinates to map coordinates
    vector<double> getMapXY(carData_t &stCarData, double localX, double localY) {

      vector<double> mapPoint;

      mapPoint.push_back((localX * cos(stCarData.yaw_r)) - (localY * sin(stCarData.yaw_r)) + stCarData.x);
      mapPoint.push_back((localX * sin(stCarData.yaw_r)) + (localY * cos(stCarData.yaw_r)) + stCarData.y);

      return mapPoint;
    }

    // convert local coordinates to map coordinates vector
    vector<vector<double>> getMapCoordPoints(carData_t &stCarData, vector<double> localXs, vector<double> localYs) {

      vector<double> mapXs;
      vector<double> mapYs;
      vector<vector<double>> mapCoordPts;

      for (int i = 0; i < localXs.size(); i++) {
        vector<double> mapXY = getMapXY(stCarData, localXs[i], localYs[i]);
        mapXs.push_back(mapXY[0]);
        mapYs.push_back(mapXY[1]);
      }
      mapCoordPts.push_back(mapXs);
      mapCoordPts.push_back(mapYs);

      return mapCoordPts;
    }

    // convert Map Coordinates to local coordinates
    vector<vector<double>> getLocalPoints(carData_t &stCarData, vector<double>mapXs, vector<double>mapYs) {

      vector<double> localXs;
      vector<double> localYs;
      vector<vector<double>> localCoordPts;

      for (int i = 0; i < mapXs.size(); i++) {
        vector<double> localXY = getLocalXY(stCarData, mapXs[i], mapYs[i]);
        localXs.push_back(localXY[0]);
        localYs.push_back(localXY[1]);
      }
      localCoordPts.push_back(localXs);
      localCoordPts.push_back(localYs);

      return localCoordPts;
    }

    // calculate a closet point from given (x, y) to the list of map points
    // maps_x and maps_y need to be the same size
    // return the index of map point that is closet to the point (x, y)
    int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
    {
      double closestLen = 100000; //large number
      int closestWaypoint = 0;

      for(int i = 0; i < maps_x.size(); i++)
      {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
          closestLen = dist;
          closestWaypoint = i;
        }
      }
      return closestWaypoint;
    }
/*
    // to find the next way point within the given (x, y) and angle theta from the map points
    // return the index of next way point
    int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
    {
      int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

      double map_x = maps_x[closestWaypoint];
      double map_y = maps_y[closestWaypoint];
      double heading = atan2( (map_y-y),(map_x-x) );
      double angle = abs(theta-heading);

      if(angle > pi()/4)
      {
        closestWaypoint++;
        // safe guard check to make sure it does not go over the total map points and loop it to the first point
        if(closestWaypoint >= maps_x.size())
          closestWaypoint = 0;
      }
      return closestWaypoint;
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
    {
      int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

      int prev_wp;
      prev_wp = next_wp-1;
      if(next_wp == 0)
      {
        prev_wp  = maps_x.size()-1;
      }

      double n_x = maps_x[next_wp]-maps_x[prev_wp];
      double n_y = maps_y[next_wp]-maps_y[prev_wp];
      double x_x = x - maps_x[prev_wp];
      double x_y = y - maps_y[prev_wp];

      // find the projection of x onto n
      double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
      double proj_x = proj_norm*n_x;
      double proj_y = proj_norm*n_y;

      double frenet_d = distance(x_x,x_y,proj_x,proj_y);

      //see if d value is positive or negative by comparing it to a center point
      double center_x = 1000-maps_x[prev_wp];
      double center_y = 2000-maps_y[prev_wp];
      double centerToPos = distance(center_x,center_y,x_x,x_y);
      double centerToRef = distance(center_x,center_y,proj_x,proj_y);

      if(centerToPos <= centerToRef)
      {
        frenet_d *= -1;
      }

      // calculate s value
      double frenet_s = 0;
      for(int i = 0; i < prev_wp; i++)
      {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
      }
      frenet_s += distance(0,0,proj_x,proj_y);

      return {frenet_s,frenet_d};
    }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
    {
      int prev_wp = -1;

      while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
      {
        prev_wp++;
      }

      int wp2 = (prev_wp+1)%maps_x.size();

      double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
      // the x,y,s along the segment
      double seg_s = (s-maps_s[prev_wp]);

      double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
      double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

      double perp_heading = heading-pi()/2;

      double x = seg_x + d*cos(perp_heading);
      double y = seg_y + d*sin(perp_heading);

      return {x,y};
    }
*/

}

