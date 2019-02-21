#include <iostream>
#include <ros/ros.h>


const int MY_VAR = 2;


struct A {

    int b;
}


int doSomethin() {

    return 0;
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "player_jmoreira");
    // ros::Rate r(100);

    for (int i=0; i<10; i++) {
        std::cout << i+1 << std::endl;
    }

    return 0;
}