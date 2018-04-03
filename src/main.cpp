#include <fstream>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "path_planner.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string& s)
{
    using std::string;
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


int main()
{
    uWS::Hub h;
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    MapData mapData;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    mapData.max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        mapData.waypoints_x.push_back(x);
        mapData.waypoints_y.push_back(y);
        mapData.waypoints_s.push_back(s);
        mapData.waypoints_dx.push_back(d_x);
        mapData.waypoints_dy.push_back(d_y);
    }

    PathPlanner planner{
        move(mapData)
    };


    h.onMessage(
        [&planner](
            uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
            uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            //auto sdata = string(data).substr(0, length);
            //cout << sdata << endl;
            if (length > 2 && data[0] == '4' && data[1] == '2')
            {

                auto s = hasData(data);

                if (!s.empty())
                {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                        // j[1] is the data JSON object

                        // Main car's localization Data
                        {
                            State state{
                                j[1]["x"],
                                j[1]["y"],
                                j[1]["s"],
                                j[1]["d"],
                                j[1]["yaw"],
                                j[1]["speed"]
                            };

                            planner.UpdateLocalisation(state);
                        }

                        // Previous path data given to the Planner
                        planner.UpdateHistory({
                                                  j[1]["previous_path_x"],
                                                  j[1]["previous_path_y"]
                                              });
                        // Previous path's end s and d values
                        double end_path_s = j[1]["end_path_s"];
                        double end_path_d = j[1]["end_path_d"];

                        // Sensor Fusion Data, a list of all other cars on the same side of the road.
                        auto sensor_fusion = j[1]["sensor_fusion"];
                        SensorFusionData sensorFusionData{};

                        for (auto&& car : sensor_fusion)
                        {
                            sensorFusionData.id.push_back(car[0]);
                            sensorFusionData.x.push_back(car[1]);
                            sensorFusionData.y.push_back(car[2]);
                            sensorFusionData.vx.push_back(car[3]);
                            sensorFusionData.vy.push_back(car[4]);
                            sensorFusionData.s.push_back(car[5]);
                            sensorFusionData.d.push_back(car[6]);
                        }
                        planner.UpdateSensorFusion(move(sensorFusionData));

                        json msgJson;

                        auto path = planner.PlanPath();

                        msgJson["next_x"] = move(path.x);
                        msgJson["next_y"] = move(path.y);

                        auto msg = "42[\"control\"," + msgJson.dump() + "]";
//                        std::cout << msg << "\n";
                        //this_thread::sleep_for(chrono::milliseconds(1000));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
                else
                {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
