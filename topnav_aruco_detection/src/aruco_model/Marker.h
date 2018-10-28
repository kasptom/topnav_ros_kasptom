#ifndef ARUCO_MARKER_H
#define ARUCO_MARKER_H

#include <opencv2/aruco.hpp>

static const std::string TOPIC_NAME_ARUCO_DETECTION = "/capo/camera/aruco"; // NOLINT

using namespace cv;
using namespace std;

class Marker {
public:
    Marker(int id, vector<Point2f> corners, Vec3d rotation, Vec3d translation);

    vector<Point2f> getCorners();

    Vec3d getRotation();

    Vec3d getTranslation();

    int getId();

    static vector<Marker> convertToMarkers(vector<int> ids, vector<vector<Point2f>> allCorners, vector<Vec3d> rotations,
                                           vector<Vec3d> translations);

private:
    vector<Point2f> corners;
    Vec3d rotation;
    Vec3d translation;
    int id;

    void printCorners2D(vector<Point2f> corners);
};


#endif //ARUCO_MARKER_H
