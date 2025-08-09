#include "brain_data.h"
#include <cmath>
#include "locate/math_utils.h"
#include "locate/pose.h"



std::vector<FieldMarker> Locator::getMarkers()
{
    std::vector<FieldMarker> res;
    for (size_t i = 0; i < markings.size(); i++){
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType = ' ';
        if (label == "L")
            markerType = 'L';
        else if (label == "T")
            markerType = 'T';
        else if (label == "X")
            markerType = 'X';
        else if (label == "P")
            markerType = 'P';
            
        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}



