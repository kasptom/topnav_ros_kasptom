#include <iostream>
#include "Marker.h"

using namespace cv;

Marker::Marker(int id, vector<Point2f> corners, Vec3d rotation, Vec3d translation) {
    Marker::id = id;
    Marker::corners = corners;
    // printCorners2D(corners);
    Marker::rotation = rotation;
    Marker::translation = translation;
}

/**
 * Prints coordinates of the ArUco marker on the image
 * @param corners (x,y) coordinates denominated in pixels
 */
void Marker::printCorners2D(vector<Point2f> corners) {
    printf("(%3.0f, %3.0f), (%3.0f, %3.0f), (%3.0f, %3.0f), (%3.0f, %3.0f) ",
           corners[0].x, corners[0].y,
           corners[1].x, corners[1].y,
           corners[2].x, corners[2].y,
           corners[3].x, corners[3].y);
}

vector<Point2f> Marker::getCorners() {
    return corners;
}

Vec3d Marker::getRotation() {
    return rotation;
}

Vec3d Marker::getTranslation() {
    return translation;
}

vector<Marker>
Marker::convertToMarkers(vector<int> ids, vector<vector<Point2f>> allCorners, vector<Vec3d> rotations,
                         vector<Vec3d> translations) {
    vector<Marker> markers;
    size_t size = ids.size();
    for (unsigned i = 0; i < size; i++) {
        Marker marker = Marker(ids.at(i), allCorners.at(i), rotations.at(i), translations.at(i));
        markers.push_back(marker);
    }

    return markers;
}

int Marker::getId() {
    return id;
}
