//
// Created by Sandra on 22/11/2019.
//

#include <tuple>
#include <algorithm>
#include "DataReader.h"

vector<tuple<int, int, float>> DataReader::read(int nbLines, const string& firstSpeciesName, const string& secondSpeciesName) {
    // Open the File
    // Create a text string, which is used to output the text file
    string line, firstTemp, secondTemp, firstSpeciesID, secondSpeciesID, score;
    string delimiter = ",";
    vector<float> scores;
    vector<int> firstSpecies, secondSpecies;
    vector<tuple<int, int, float>> result = vector<tuple<int, int, float>>();

    int nbReadLines = 0, gap, index = 0, tempFirstIndex, tempSecondIndex;

    // Read from the text file
    ifstream readFile("../Data/" + firstSpeciesName + "/" + secondSpeciesName);

    if(readFile.is_open()){
        // Use a while loop together with the getline() function to read the file line by line
        // First read to get IDs of the two species
        while(nbReadLines != nbLines && getline(readFile, line)){
            if(line[0] == '[') {
                if(line[1] == '[') {
                    gap = 2;
                }
                else {
                    gap = 1;
                }
                firstSpeciesID = line.substr(gap, line.find(delimiter) - gap);
                firstTemp = line.substr(line.find(delimiter) + 2);
                secondSpeciesID = firstTemp.substr(0, firstTemp.find(delimiter));
                if(!isIdIn(stoi(firstSpeciesID), firstSpecies))
                    firstSpecies.emplace_back(stoi(firstSpeciesID));
                if(!isIdIn(stoi(secondSpeciesID), secondSpecies))
                    secondSpecies.emplace_back(stoi(secondSpeciesID));
            }
            nbReadLines++;
        }

        // Second read to map scores
        readFile.seekg(0, ios::beg);
        while(getline(readFile, line)){
            if(line[0] == '[') {
                if(line[1] == '[')
                    gap = 2;
                else
                    gap = 1;

                firstSpeciesID = line.substr(gap, line.find(delimiter) - gap);
                firstTemp = line.substr(line.find(delimiter) + 2);
                secondSpeciesID = firstTemp.substr(0, firstTemp.find(delimiter));
                secondTemp = firstTemp.substr(firstTemp.find(delimiter) + 2);
                score = secondTemp.substr(0, secondTemp.find(delimiter));

                tempFirstIndex = getIndexOf(stoi(firstSpeciesID), firstSpecies);
                tempSecondIndex = getIndexOf(stoi(secondSpeciesID), secondSpecies);
                if(tempFirstIndex != -1 && tempSecondIndex != -1)
                    result.emplace_back(make_tuple(firstSpecies[tempFirstIndex], secondSpecies[tempSecondIndex], stof(score)));
                index++;
            }
        }
    }
    else
        cout << "nope" << endl;

    // Close the file
    readFile.close();
    cout << "Reading done" << endl;

    normalizeWeight(firstSpecies, secondSpecies, &result);
    return result;
}

void DataReader::normalizeWeight(const vector<int>& firstSpecies, const vector<int>& secondSpecies, vector<tuple<int, int, float>> *scores) {
    float maxScore, threshold = 0.7, tempScore = 0;
    int tempIndex;
    for(int firstSpecie : firstSpecies){
        maxScore = 0;
        for(int secondSpecie : secondSpecies) {
            tempScore = getScore(firstSpecie, secondSpecie, *scores);

            if (tempScore > maxScore)
                maxScore = tempScore;
        }
        for(int secondSpecie : secondSpecies) {
            tempScore = getScore(firstSpecie, secondSpecie, *scores);
            if(tempScore != -1) {
                tempIndex = getIndexOfScore(firstSpecie, secondSpecie, *scores);
                scores->erase(scores->begin() + tempIndex);
                scores->emplace(scores->begin() + tempIndex, make_tuple(firstSpecie, secondSpecie, (tempScore / maxScore) - threshold));
            }
        }
    }
}

bool DataReader::isIdIn(int id, vector<int> species) {
    int index = 0;
    while(index < species.size()){
        if(species[index] == id)
            return true;
        index++;
    }
    return false;
}

int DataReader::getIndexOf(int id, vector<int> species) {
    int index = 0;
    while(index < species.size()){
        if(species[index] == id)
            return index;
        index++;
    }
    return -1;
}

float DataReader::getScore(int firstSpeciesId, int secondSpeciesId, const vector<tuple<int, int, float>>& scores) {
    int index = 0;
    while(index < scores.size())
        if(get<0>(scores[index]) == firstSpeciesId && get<1>(scores[index]) == secondSpeciesId)
            return get<2>(scores[index]);
        else
            index++;
    return -1;
}

int DataReader::getIndexOfScore(int firstSpeciesId, int secondSpeciesId, const vector<tuple<int, int, float>> &scores) {
    int index = 0;
    while(index < scores.size())
        if(get<0>(scores[index]) == firstSpeciesId && get<1>(scores[index]) == secondSpeciesId)
            return index;
        else
            index++;
    return -1;
}

