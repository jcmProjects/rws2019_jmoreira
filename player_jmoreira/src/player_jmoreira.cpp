/* Includes */
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


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

            void printInfo() {
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

            /* Methods */
            MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
                setTeamName(team_name_in);

                team_red    = (boost::shared_ptr<Team>) new Team("red");
                team_green  = (boost::shared_ptr<Team>) new Team("green");
                team_blue   = (boost::shared_ptr<Team>) new Team("blue");

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
                float dx = 0.1;
                float angle = M_PI/16;

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