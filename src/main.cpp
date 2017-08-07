#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "../header/utility.h"

#define SIMPLE_TEST 0
#define COMPLEX_PATH 0


using namespace std;
using json = nlohmann::json;
// added to use spline utility
using spline = tk::spline;

// ================== MACRO =======================================================
#define NUM_OF_POINTS     40
#define NUM_OF_HW_LANES   3

// ================== Global Variables ============================================
#if 1
// keeping velocity history for smooth speed
vector<double> g_vdVelocityHist;
// keeping path history
vector<vector<double>> g_vvdPathHist;

// my Car, previous Path and Sensor Fusion data to be stored
carData_t g_stCarData;
vector<vector<double>> g_vvdPreviousPath;
vector<vector<double>> g_vvdSensorFusion;
int g_iPrevPathSize;
int g_iSensorFusionSize;

// tracking lane changes
bool g_bIsChangingLane = false;
// tracking the count. Incrementing when it is good to change lane
int g_iChangingLaneCount = 0;
// current lane
int g_iCurLane;
// to wrap around the map
double g_dNextD = 6.0;
double g_dNextS = 0.4425; //0.445;
#endif



// ================== Utility functions starts ====================================
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/*
 * setting up the lane to track using spline function from
 * http://kluge.in-chemnitz.de/opensource/spline/
 */
void setupLane(spline &Lane, mapData_t &stMapData, double g_dNextD){
	// get local points near by
	vector<vector<double>> vvLocalPoints = utility::getLocalPointsNearBy(g_stCarData, stMapData, g_dNextD);

	// check if it is in the wrong way then change direction to get the local points
	if (vvLocalPoints[0][0] > 0.0) {
    g_stCarData.yaw_d += 180.0;
    vvLocalPoints = utility::getLocalPointsNearBy(g_stCarData, stMapData, g_dNextD);
	}

	Lane.set_points(vvLocalPoints[0], vvLocalPoints[1]);
}

/*
 * find the next path points that are used to feed the simulation
 * using the car data, previous path data and sensor fusion data with
 * map data to plan the next x and y values that will be in the planned path
 * outputs are vvdPathPoints
 */
vector<vector<double>> findNextPath(mapData_t &stMapData,
 																			carData_t &stCarData,
 																			vector<vector<double>> &vvdPreviousPath,
 																			vector<vector<double>> &vvdSensorFusion) {

  // planned path points to return to simulator
  vector<vector<double>> vvdPathPoints;
  memcpy(&g_stCarData, &stCarData, sizeof(carData_t));
  g_vvdPreviousPath = vvdPreviousPath;
  g_vvdSensorFusion = vvdSensorFusion;


  g_iPrevPathSize = g_vvdPreviousPath[0].size();
  g_iSensorFusionSize = g_vvdSensorFusion.size();

	// current lane
	g_iCurLane = (int)(round(round(g_stCarData.d - 2.0) / 4.0));

	// setup the lane using spline
	spline splLane;
	setupLane(splLane, stMapData, g_dNextD);

  //cout << "size of sensor fusion: " << iSensorFusionSize << endl;

	// I) Path Planning
  // ================= check to see if it is a start using size of previous path
	// start of the path
	if(g_iPrevPathSize == 0) {

    // 1) setting up velocity spline to track the velocity
    vector<double> vTimeForStart, vDistForStart;

    vTimeForStart.push_back(-1.0);
    vTimeForStart.push_back(12);
    vTimeForStart.push_back(20);
    vTimeForStart.push_back(40);
    vTimeForStart.push_back(80);

    vDistForStart.push_back(g_dNextS * 0.01);
    vDistForStart.push_back(g_dNextS * 0.10);
    vDistForStart.push_back(g_dNextS * 0.15);
    vDistForStart.push_back(g_dNextS * 0.25);
    vDistForStart.push_back(g_dNextS * 0.30);

    spline splVelocity;
    splVelocity.set_points(vTimeForStart, vDistForStart);

    // 2) using velocity and lane spline to form lane
    vector<double> vLocalXs, vLocalYs;
    double dX = 0.0;
    double dY = 0.0;

    for (int i = 0; i < NUM_OF_POINTS; i++) {
      dX += splVelocity(double(i));
      vLocalXs.push_back(dX);
      vLocalYs.push_back(splLane(dX));
    }

    // 3) calculate the smoother lane path by checking the distance and speed
    // if it is not within the intended speed, smoothen the lane path with
    // heading angle to smooth out the lane path
    dX = 0.0;
    for (int i = 0; i < NUM_OF_POINTS; i++) {
      double distance = utility::distance(dX, dY, vLocalXs[i], vLocalYs[i]);
      double speed = splVelocity(double(i));

      if ((distance < (speed * 0.8) || (distance > speed))) {
        double heading = atan2((vLocalYs[i] - dY), (vLocalXs[i] - dX));
        vLocalXs[i] = dX + splVelocity(double(i)) * cos(heading);
        vLocalYs[i] = splLane(vLocalXs[i]);
      }

      // save the velocity history and update dX, dY
      // using global variable to use it later it the already running state
      g_vdVelocityHist.push_back(utility::distance(dX, dY, vLocalXs[i], vLocalYs[i]));
      dX = vLocalXs[i];
      dY = vLocalYs[i];
    }

    // update s
    g_dNextS = splVelocity(NUM_OF_POINTS);
    //cout << "g_dNextS after start of the path: " << g_dNextS << endl;

    // convert local to map coordinates for simulator inputs
    vvdPathPoints = utility::getMapCoordPoints(g_stCarData, vLocalXs, vLocalYs);
    // using global variable to use it later it the already running state
    g_vvdPathHist = vvdPathPoints;
  } // if(iPrevPathSize == 0) ends -------------------------------

  // ======================== already started running
	else {

    // 1) get previous path as local points
    vector<vector<double>> vvdPrevLocalPoints = utility::getLocalPoints(g_stCarData, g_vvdPreviousPath[0], g_vvdPreviousPath[1]);
    assert(vvdPrevLocalPoints[0].size() == g_iPrevPathSize);
    assert(vvdPrevLocalPoints[1].size() == g_iPrevPathSize);

    // 2) Clear completed portion of the path from the Velocity and Path
    int iCompleted = NUM_OF_POINTS - g_iPrevPathSize;
    //cout << "Number of completed points: " << iCompleted << endl;
    g_vdVelocityHist.erase(g_vdVelocityHist.begin(), g_vdVelocityHist.begin() + iCompleted);
    g_vvdPathHist[0].erase(g_vvdPathHist[0].begin(), g_vvdPathHist[0].begin() + iCompleted);
    g_vvdPathHist[1].erase(g_vvdPathHist[1].begin(), g_vvdPathHist[1].begin() + iCompleted);

    // 3) check to see if changing lane
    if ((g_bIsChangingLane == true) &&
            (g_stCarData.d >= (g_dNextD - 0.1)) &&
            (g_stCarData.d <= (g_dNextD + 0.1)))
    {
      g_bIsChangingLane = false;
      g_iChangingLaneCount = 0;
    }

    // 4) setup Path and Velocity
    // setup lane with previous path in it
    spline splLaneWithPreviousPath;
    setupLane(splLaneWithPreviousPath, stMapData, g_dNextD);

    // set up spline using previous path
    vector<double> vd_localX, vd_localY;
    for (int i = 0; i < g_iPrevPathSize; i++) {
      vd_localX.push_back(vvdPrevLocalPoints[0][i]);
      vd_localY.push_back(vvdPrevLocalPoints[1][i]);
    }

    // add the next points to the splLaneWithPreviousPath spline
    double next = vvdPrevLocalPoints[0][g_iPrevPathSize - 1] + NUM_OF_POINTS;
    //cout << "new distance increment: " << next << endl;
    for (int i = 0; i < g_iPrevPathSize; i++) {
      vd_localX.push_back(next);
      vd_localY.push_back(splLaneWithPreviousPath(next));
      next += g_dNextS;
    }
    // set new path
    splLane.set_points(vd_localX, vd_localY);

    // setup the velocity spline with previous velocity
    vector<double> vTimeForRunning, vDistForRunning;
    for (int i = 0; i < g_iPrevPathSize; i++) {
      vTimeForRunning.push_back(double(i));
      vDistForRunning.push_back(g_vdVelocityHist[i]);
    }
    vTimeForRunning.push_back((NUM_OF_POINTS * 5.0));
    vDistForRunning.push_back(g_dNextS);

    // velocity spline with previous velocity
    spline splVelWithPrevVelocity;
    splVelWithPrevVelocity.set_points(vTimeForRunning, vDistForRunning);

    // 5) fill in the path with previous path
    for (int i = g_iPrevPathSize; i < NUM_OF_POINTS; i++) {
      vvdPrevLocalPoints[0].push_back(vvdPrevLocalPoints[0][i-1] + splVelWithPrevVelocity(double(i)));
      vvdPrevLocalPoints[1].push_back(splLane(vvdPrevLocalPoints[0][i]));
    }
    assert(vvdPrevLocalPoints[0].size() == NUM_OF_POINTS);
    assert(vvdPrevLocalPoints[1].size() == NUM_OF_POINTS);

    //cout << "Velocity Point size: " << vvdPrevLocalPoints[0].size() << endl;
    //cout << "Path Point size: " << vvdPrevLocalPoints[1].size() << endl;

    // 6) smooth out the lane path
    double localX = vvdPrevLocalPoints[0][0];
    double localY = vvdPrevLocalPoints[1][0];
    for (int i = 0; i < NUM_OF_POINTS; i++) {
      double distance = utility::distance(localX, localY, vvdPrevLocalPoints[0][i], vvdPrevLocalPoints[1][i]);
      if (distance > splVelWithPrevVelocity(double(i))) {
        double heading = atan2((vvdPrevLocalPoints[1][i] - localY),
                               (vvdPrevLocalPoints[0][i] - localX));
        vvdPrevLocalPoints[0][i] = localX + (splVelWithPrevVelocity(double(i)) * cos(heading));
        vvdPrevLocalPoints[1][i] = splLane(vvdPrevLocalPoints[0][i]);
      }
      if ( i >= g_iPrevPathSize) {
        // update the global Velocity History
        g_vdVelocityHist.push_back(utility::distance(localX, localY, vvdPrevLocalPoints[0][i], vvdPrevLocalPoints[1][i]));
      }
      localX = vvdPrevLocalPoints[0][i];
      localY = vvdPrevLocalPoints[1][i];
    }

    // 7) convert to the Map points for the simulator
    vector<vector<double>> mapPoints = utility::getMapCoordPoints(g_stCarData, vvdPrevLocalPoints[0], vvdPrevLocalPoints[1]);
    assert(mapPoints[0].size() == NUM_OF_POINTS);
    assert(mapPoints[1].size() == NUM_OF_POINTS);

    // get the previous path at start
    vvdPathPoints = g_vvdPreviousPath;
    // populate the rest with the new points
    for (int i = g_iPrevPathSize; i < mapPoints[0].size(); i++) {
      vvdPathPoints[0].push_back(mapPoints[0][i]);
      vvdPathPoints[1].push_back(mapPoints[1][i]);
    }
    // 8) update the global path history
    g_vvdPathHist.clear();
    g_vvdPathHist = vvdPathPoints;
	} // else ends ------------------------------------

  // II) Behavior Planning
  // ============================ Check other cars behavior with handling lane changes
  if(g_bIsChangingLane == false) {

    // look at the sensor fusion data and the other cars behaviors
    // 1) set the lanes
    vector<vector<vector<double>>> hwLanes(NUM_OF_HW_LANES);

    // 2) place the other cars from sensor fusion data to the respective lane
    for (int i = 0; i < g_iSensorFusionSize; i++) {
      vector<double> vCar = g_vvdSensorFusion[i];
      // simulation simulate one point to another in 20 ms
      g_vvdSensorFusion[i].push_back((utility::distance(0.0, 0.0, vCar[3], vCar[4]) * 0.02));
      // s value from my Car to the other car
      g_vvdSensorFusion[i].push_back(vCar[5] - g_stCarData.s);
      // add cars to the respective lanes
      for (int j = 0; j < NUM_OF_HW_LANES; j++) {
        // Lane width is 4 m
        if ((vCar[6] >= ((j * 4) - 0.3)) && (vCar[6] <= (((j + 1) * 4) + 0.3))){
          hwLanes[j].push_back(g_vvdSensorFusion[i]);
        }
      }
    }

    // 3) sorting lanes by the distance
    for (int i = 0; i < NUM_OF_HW_LANES; i++) {
      sort(hwLanes[i].begin(), hwLanes[i].end(),[](vector<double> &a, vector<double> &b){
          return (a[8] < b[8]);
      });
    }

    // 4) find the closet cars behind and ahead of my car
    vector<vector<int>> closestCars;
    closestCars.resize(NUM_OF_HW_LANES);
    for (int i = 0; i < NUM_OF_HW_LANES; i++) {
      int size = hwLanes[i].size();
      closestCars[i].push_back(-1);
      closestCars[i].push_back(-1);

      // closest cars behind
      for (int j = (size - 1); j >= 0; j--) {
        if (hwLanes[i][j][8] < 0) {
          closestCars[i][0] = j;
          break;
        }
      }

      // closest cars ahead
      for (int j = 0; j < size; j++) {
        if (hwLanes[i][j][8] > 0) {
          closestCars[i][1] = j;
          break;
        }
      }
    }

    // 5) select lane if need to change lane
    double distWeight = 3.0;
    double laneChangeWeight = 0.75; // 0.85;
    double velWeight = 3.0;
    double maxGap = 200.0;
    double minGap = 10.0;
    double maxDistInc = 0.4425;

    // store costs
    vector<pair<double, int>> vLaneCostPairs;

    // set the costs for the ahead of my car, behind my car and velocity of the car
    for (int i = 0; i < NUM_OF_HW_LANES; i++) {
      double laneChangeCost, distanceCost, velocityCost;

      // calculate lane change points
      laneChangeCost = laneChangeWeight * (1.0 - (fabs(i - g_iCurLane) / (NUM_OF_HW_LANES - 1)));

      // calculate distance and velocity points
      if(closestCars[i][1] == -1) {
        distanceCost = distWeight;
        velocityCost = velWeight;
      }
      else {
        distanceCost = distWeight * (1.0 - ((maxGap - hwLanes[i][closestCars[i][1]][8]) / maxGap));
        velocityCost = velWeight * (1.0 - (((maxDistInc * 2.0) - hwLanes[i][closestCars[i][1]][7]) / (maxDistInc * 2.0)));
      }
      // save it in the cost pairs
      vLaneCostPairs.push_back(make_pair((laneChangeCost + distanceCost + velocityCost), i));
    }
    sort(vLaneCostPairs.begin(), vLaneCostPairs.end());

    // get the costs
    vector<int> sortedLaneRanks;
    sortedLaneRanks.resize(NUM_OF_HW_LANES);

    for (int i = 0; i < NUM_OF_HW_LANES; i++) {
      sortedLaneRanks[i] = vLaneCostPairs[NUM_OF_HW_LANES - i - 1].second;
    }


    // 6) change lane if there is enough gaps and speeds as above calculated costs
    int iDestLane = g_iCurLane;

    for (int i = 0; i < NUM_OF_HW_LANES; i++) {

      int iSortedLane = sortedLaneRanks[i];
      // check if rank lane is already current lane
      if (iSortedLane == g_iCurLane) {
        g_iChangingLaneCount = 0;
        break;
      }

      // determine how many lanes need to be changed
      int iLanesNeedToChange = iSortedLane - g_iCurLane;
      int iDirectionNeed = iLanesNeedToChange / abs(iLanesNeedToChange);

      bool shouldChangeLane = true;

      // if the car is going fast, should not change the lane
      if((g_stCarData.speed >= 40.0) && (abs(iLanesNeedToChange) > 1)) {
        shouldChangeLane = false;
      }

      // check to see if there are cars behind and in front of my car with repect to the potential lanes to be changed
      for (int i = 1; i <= abs(iLanesNeedToChange); i++) {
        int iLane = g_iCurLane + (i * iDirectionNeed);
        int indexOfCarBehind = closestCars[iLane][0];
        int indexOfCarAhead = closestCars[iLane][1];
        // Car behind
        if (indexOfCarBehind != -1) {
          // compare distance and speed
          double dDist = abs(hwLanes[iLane][indexOfCarBehind][8]);
          double dVel = hwLanes[iLane][indexOfCarBehind][7];
          if (((dVel < g_dNextS) && (dDist > (minGap * 0.5))) ||
              ((dVel > g_dNextS) && (dDist > (minGap * 3.0)))) {
            // do nothing since it is okay to change
          }
          else {
            shouldChangeLane = false;
            break;
          }
        }
        // Car ahead
        if (indexOfCarAhead != -1) {
          // compare distance and speed
          double dDist = abs(hwLanes[iLane][indexOfCarAhead][8]);
          double dVel = hwLanes[iLane][indexOfCarAhead][7];
          if (dDist > (minGap * 2.0)) {
            // do nothing since it is okay to change
          }
          else {
            shouldChangeLane = false;
            break;
          }
        }
      } // for (int i = 0; i <= abs(iLanesNeedToChange); i++)

      // add to changing lane count and check the lanes are good to change
      if(shouldChangeLane == true) {
        g_iChangingLaneCount++;

        if (g_iChangingLaneCount > 20) {
          g_bIsChangingLane = true;
          iDestLane = iSortedLane;
          // reset lane count
          g_iChangingLaneCount = 0;
        }
        break;
      }
    } // for (int i = 0; i < NUM_OF_HW_LANES; i++)

    // 7) Update d and s
    // lane width is 4 m
    g_dNextD = (iDestLane * 4) + (4 * 0.5);
    int iIndexOfCar = closestCars[iDestLane][1];
    if (iIndexOfCar == -1) {
      g_dNextS = 0.4425;
    }
    else {
      double dDist = hwLanes[iDestLane][iIndexOfCar][8];
      double dVel = hwLanes[iDestLane][iIndexOfCar][7];
      if(dDist > (minGap * 4.0)) {
        g_dNextS = 0.4225;
      }
      else if (dDist < (minGap * 1.0)) {
        // stop
        g_dNextS = 0;
      }
      else {
        g_dNextS = ((g_dNextS * 0.90) < dVel) ? dVel : (g_dNextS * 0.90);
      }
    }

  } // if(g_bIsChangingLane == false) ends --------------------------

	return vvdPathPoints;
}
// ================== Utility functions ends ====================================

// ================== Main function starts ======================================
int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
	

  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  // Read map points from map_file
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
	mapData_t mapData;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	double s;
  	double d_x;
  	double d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
		mapData.vx.push_back(x);
		mapData.vy.push_back(y);
		mapData.vs.push_back(s);
		mapData.vdx.push_back(d_x);
		mapData.vdy.push_back(d_y);
  }

  h.onMessage([&mapData](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          const double yaw = j[1]["yaw"];
					double yaw_r = utility::deg2rad(yaw);
					// save car data
          carData_t carData = {
                  j[1]["x"],
                  j[1]["y"],
                  j[1]["s"],
                  j[1]["d"],
                  yaw,
									yaw_r,
                  j[1]["speed"]
          };

          // Previous path data given to the Planner
					vector<vector<double>> previousPath = {
									j[1]["previous_path_x"],
									j[1]["previous_path_y"]
					};

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensorFusion = j[1]["sensor_fusion"];

          json msgJson;

// ======== Final Implementation ========================================================

          // =============== plan the path ==============================================
          vector<vector<double>> nextPathPoints = findNextPath(mapData,
                                                           carData,
                                                           previousPath,
                                                           sensorFusion);
					msgJson["next_x"] = nextPathPoints[0];
					msgJson["next_y"] = nextPathPoints[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}