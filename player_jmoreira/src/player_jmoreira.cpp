/* Includes */
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


/* Namespaces */
using namespace std;
using namespace boost;
using namespace ros;
using namespace tf;


float randomizePosition(void) {
    srand( 6832 * time(NULL) );                             // set initial seed value to 5323
    return ( ((double)rand() / (RAND_MAX)) - 0.5) * 10;
}


/* Created namespace */
namespace jmoreira_ns {

    class Team {
        public:

            string team_name;
            vector<string> player_names;
            NodeHandle n;

            /* Constructor */
            Team(string team_name_in) {
                team_name = team_name_in;
                // read team players:
                n.getParam("/team_" + team_name, player_names);
            }

            void printTeamInfo() {
                cout << "Team " << team_name << " has players: " << endl;

                for (size_t i=0; i<player_names.size(); i++)
                    cout << player_names[i] << endl;
            }

            bool playerBelongsToTeam(string player_name) {
                for (size_t i=0; i<player_names.size(); i++) {
                    if (player_name == player_names[i])
                        return true;
                }

                return false;
            }
    };

    class Player {
        public:

            /* Properties */
            string player_name;
            
            Player(string player_name_in) { 

                player_name = player_name_in;
            }

            void setTeamName(string team_name_in) {
                if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
                    team_name = team_name_in;
                else
                    cout << "Cannot set team name " << team_name_in;
            }

            void setTeamName(int team_index) {
                if (team_index == 0)
                    setTeamName("red");
                else if (team_index == 1)
                    setTeamName("green");
                else if (team_index == 2)
                    setTeamName("blue");
                else
                    setTeamName("");
            }

            string getTeamName() {
                return team_name;
            }


        private:

            string team_name = "";
    };

    /* Herança */
    class MyPlayer : public Player {
        public: 

            boost::shared_ptr<Team> team_red;
            boost::shared_ptr<Team> team_green;
            boost::shared_ptr<Team> team_blue;
            boost::shared_ptr<Team> team_mine;
            boost::shared_ptr<Team> team_preys;
            boost::shared_ptr<Team> team_hunters;
            TransformBroadcaster br;
            TransformListener listener;
            boost::shared_ptr<Publisher> vis_pub;

            /* Methods */
            MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
                setTeamName(team_name_in);

                team_red    = (boost::shared_ptr<Team>) new Team("red");
                team_green  = (boost::shared_ptr<Team>) new Team("green");
                team_blue   = (boost::shared_ptr<Team>) new Team("blue");

                NodeHandle n;
                vis_pub     = (boost::shared_ptr<Publisher>) new Publisher;
                (*vis_pub)  = n.advertise<visualization_msgs::Marker>( "player_names", 0 );

                if (team_red->playerBelongsToTeam(player_name)) {
                    team_mine    = team_red;
                    team_preys   = team_green;
                    team_hunters = team_blue;
                }
                else if (team_green->playerBelongsToTeam(player_name)) {
                    team_mine    = team_green;
                    team_preys   = team_blue;
                    team_hunters = team_red;
                }
                else if (team_blue->playerBelongsToTeam(player_name)) {
                    team_mine    = team_blue;
                    team_preys   = team_red;
                    team_hunters = team_green;
                }
                else
                    cout << "Something wrong happened in team parameterization!!!" << endl;
            
                setTeamName(team_mine->team_name);
                
                /* Define initial position */
                float sx = randomizePosition();
                float sy = randomizePosition();
                

                Transform T;
                T.setOrigin( Vector3(sx, sy, 0.0) );
                Quaternion q;
                q.setRPY(0, 0, M_PI);
                T.setRotation(q);
                br.sendTransform( StampedTransform(T, Time::now(), "world", player_name) );
                Duration(0.1).sleep();
                br.sendTransform( StampedTransform(T, Time::now(), "world", player_name) );

                printInfo();
            }

            void printInfo(void) {
                ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
                ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
            }

            std::tuple<float, float> getDistanceAndAngleToPlayer(string other_player) {
                StampedTransform T; // The transform object
                float dist;
                float angle;

                try {
                    listener.lookupTransform(player_name, other_player, Time(0), T);
                }
                catch (TransformException& ex) {
                    ROS_ERROR("%s", ex.what());
                    Duration(0.01).sleep();
                    return {1000.0, 0.0};
                }

                dist  = sqrt(T.getOrigin().y() * T.getOrigin().y() + T.getOrigin().x() * T.getOrigin().x());
                angle = atan2(T.getOrigin().y(), T.getOrigin().x());
                return {dist, angle};
            }

            std::tuple<float, float> getDistanceAndAngleToArena(void) {
                return getDistanceAndAngleToPlayer("world");
            }

            void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg) {
                ROS_INFO("received a new msg");

                /* Step 1: Find out where I am */
                StampedTransform T0;
                try {
                    listener.lookupTransform("/world", player_name, Time(0), T0);
                }
                catch (TransformException ex) {
                    ROS_ERROR("%s", ex.what());
                    Duration(0.1).sleep();
                }

                /* Step 2: Decide how I want to move */
                
                //* PREYS
                vector<float> distance_to_preys;
                vector<float> angle_to_preys;
                // For each prey, find the closest. Then, follow it.
                for (size_t i=0; i<team_preys->player_names.size(); i++) {
                    ROS_WARN_STREAM("Preys = " << team_preys->player_names[i]);
                    std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_preys->player_names[i]);
                    distance_to_preys.push_back( std::get<0>(t) );
                    angle_to_preys.push_back( std::get<1>(t) );
                }
                int idx_closest_prey = 0;
                float distance_to_closest_prey = 1000;
                for (size_t i=0; i<distance_to_preys.size(); i++) {
                    if (distance_to_preys[i] < distance_to_closest_prey) {
                        idx_closest_prey = i;
                        distance_to_closest_prey = distance_to_preys[i];
                    }
                }
                
                //* HUNTERS
                vector<float> distance_to_hunters;
                vector<float> angle_to_hunters;
                // For each prey, find the closest. Then, follow it.
                for (size_t i=0; i<team_hunters->player_names.size(); i++) {
                    ROS_WARN_STREAM("Hunters = " << team_hunters->player_names[i]);
                    std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_hunters->player_names[i]);
                    distance_to_hunters.push_back( std::get<0>(t) );
                    angle_to_hunters.push_back( std::get<1>(t) );
                }
                int idx_closest_hunter = 0;
                float distance_to_closest_hunter = 1000;
                for (size_t i=0; i<distance_to_preys.size(); i++) {
                    if (distance_to_hunters[i] < distance_to_closest_hunter) {
                        idx_closest_hunter = i;
                        distance_to_closest_hunter = distance_to_hunters[i];
                    }
                }
                //* WORLD BOUNDARIES
                vector<float> distance_to_center;
                vector<float> angle_to_center;
                std::tuple<float, float> t = getDistanceAndAngleToArena();
                distance_to_center.push_back( std::get<0>(t) );
                angle_to_center.push_back( std::get<1>(t) );
                //* PREYS vs HUNTERS vs WORLD
                float dx;
                float angle;
                if ((distance_to_preys[idx_closest_prey] > distance_to_hunters[idx_closest_hunter]) && (distance_to_center[0] <= 4.8)) {
                    dx = 10; //(-1) * distance_to_hunters[idx_closest_hunter];
                    angle = (-1) * angle_to_hunters[idx_closest_hunter];
                }
                else if ((distance_to_preys[idx_closest_prey] < distance_to_hunters[idx_closest_hunter]) && (distance_to_center[0] <= 4.8)){
                    dx = 10; //distance_to_preys[idx_closest_prey];
                    angle = angle_to_preys[idx_closest_prey]; 
                }
                else {
                    dx = 0.05;
                    angle = angle_to_center[0];
                }
                
                /*
                float dx = 10;
                float angle = angle_to_preys[idx_closest_prey];
                */
                /*
                if (distance_to_center[0] > 4.0) {
                    dx = 1;
                    angle = angle_to_center[0];
                }
                if (distance_to_center[0] >= 4.9) {
                    dx = 0.2;
                    angle = angle_to_center[0];
                }
                */
 
                
                

                /* Step 2.5: check values */
                double dx_max = msg->turtle; // must be hardcoded
                dx > dx_max ? dx = dx_max: dx = dx;
                double angle_max = M_PI/30;
                fabs(angle) > fabs(angle_max) ? angle = angle_max * angle / fabs(angle): angle = angle;

                /* Step 3: Define local movement */
                Transform T1;
                T1.setOrigin( Vector3(dx, 0.0, 0.0) );
                Quaternion q;
                q.setRPY(0, 0, angle);
                T1.setRotation(q);

                /* Step 4: Define global movement */
                Transform Tglobal = T0 * T1;
                br.sendTransform( StampedTransform(Tglobal, Time::now(), "world", player_name) );
                
                /* Marker */
                visualization_msgs::Marker marker;
                marker.header.frame_id = player_name;
                marker.header.stamp = ros::Time();
                marker.ns = player_name;
                marker.id = 0;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::Marker::ADD;
                // marker.pose.position.x = 1;
                // marker.pose.position.y = 1;
                // marker.pose.position.z = 1;
                // marker.pose.orientation.x = 0.0;
                // marker.pose.orientation.y = 0.0;
                // marker.pose.orientation.z = 0.0;
                // marker.pose.orientation.w = 1.0;
                // marker.scale.x = 1;
                // marker.scale.y = 0.1;
                marker.scale.z = 0.6;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.text = player_name;

                //only if using a MESH_RESOURCE marker type:
                // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                vis_pub->publish( marker );
            }
    };
}
using namespace jmoreira_ns; 


/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {

    init(argc, argv, "player_jmoreira");
    NodeHandle n;

    MyPlayer player("jmoreira", "red");
    player.printInfo();

    Subscriber sub = n.subscribe("/make_a_play", 100, &MyPlayer::makeAPlayCallback, &player);

    Rate r(20);
    while( ok() ) {
        spinOnce();
        r.sleep();
    }

    return 1;
}