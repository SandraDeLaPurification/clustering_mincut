//
// Created by Sandra on 22/11/2019.
//

#ifndef START_DATAREADER_H
#define START_DATAREADER_H

#include <fstream>
#include <vector>
#include <iostream>

using namespace std;

class DataReader {

public :
    static vector<tuple<int, int, float>> read(int, const string&, const string&);
    static void normalizeWeight(const vector<int>&, const vector<int>&, vector<tuple<int, int,float>>*);
    static bool isIdIn(int, vector<int>);
    static int getIndexOf(int, vector<int>);
    static float getScore(int, int, const vector<tuple<int, int, float>>&);
    static int getIndexOfScore(int, int, const vector<tuple<int, int, float>>&);
};


#endif //START_DATAREADER_H
