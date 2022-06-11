#ifndef PROBLEMDESCRIPTION_H
#define PROBLEMDESCRIPTION_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


typedef struct {
    int i; // customer number
    int x; // x coordinate
    int y; // y coordinate
    int d; // customer service duration
    int q; // customer demand
} Customer;


typedef struct {
    int i; // depot number
    int x; // x coordinate
    int y; // y coordinate
    int D; // maximum route duration for vehicles from depot
    int Q; // maximum allowed load for vehicles from depot
} Depot;


class ProblemDescription{

    //private:

    public:
    int m; // maximum number of vehicles for each depot
    int n; // total number of customers
    int t; // number of depots
    std::vector<Customer> customers;
    std::vector<Depot> depots;
    ProblemDescription(){

    }
    ProblemDescription(std::string filepath){
        std::ifstream file;
        file.open(filepath, std::ios::in);
        if(file.is_open()){
            std::string line;
            getline(file, line);
            std::stringstream ss(line);
            ss >> m >> n >> t;

            customers = std::vector<Customer>(n);
            depots = std::vector<Depot>(t);
            
            for(int i = 0; i < t; i++){
                getline(file, line);
                ss = std::stringstream(line);
                int D, Q;
                ss >> D >> Q;
                if(D == 0){
                    D = 10000000;
                }
                depots[i] = {i, 0, 0, D, Q};
            }

            for(int i = 0; i < n; i++){
                getline(file, line);
                ss = std::stringstream(line);
                int ii, x, y, d, q;
                ss >> ii >> x >> y >> d >> q;
                customers[i] = {i, x, y, d, q};
            }

            for(int i = 0; i < t; i++){
                getline(file, line);
                ss = std::stringstream(line);
                int ii, x, y;
                ss >> ii >> x >> y;
                depots[i].x = x;
                depots[i].y = y;
            }
        }

        file.close();
    }
};

#endif
