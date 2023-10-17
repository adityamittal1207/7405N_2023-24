// //
// // Created by zayn on 8/8/21.
// //

// #ifndef INC_7405K_2021_2022_PUREPURSUIT_H
// #define INC_7405K_2021_2022_PUREPURSUIT_H

// #include "../Odometry.h"
// #include <vector>

// struct PPoint {
//     float x_;
//     float y_;

//     PPoint() : x_(0), y_(0) {}
//     PPoint(float x, float y) : x_(x), y_(y) {}
//     PPoint(Point pos) : x_(pos.x_), y_(pos.y_) {}

//     bool operator==(const PPoint& p2) {
//         return this->x_ == p2.x_ && this->y_ == p2.y_;
//     }

//     bool operator!=(const PPoint& p2) {
//         return this->x_ != p2.x_ || this->y_ != p2.y_;
//     }
// };

// class PurePursuit {
//    private:
//     double lookahead;     // radius of how far to look ahead for next point
//     double spacing;       // distance between each injected point
//     double tolerance;     //
//     double smoothWeight;  // how much should smoothing affect the path
//     double dataWeight;
//     Odometry& odom;

//     static double distance(PPoint point1, PPoint point2);
//     static double distance(double x1, double y1, double x2, double y2);
//     // Point getGoalPoint(Point curPoint);

//    public:
//     int curIndex;
//     int waypointAmt;


//     std::vector<PPoint> waypoints;
//     std::vector<double> distances;
//     std::vector<double> curvatures;
//     std::vector<double> targetVelocity;
//     std::vector<double> targetVelocityAccDec;


//     // Smoothing is from 0-1
//     PurePursuit(Odometry& odometry, double lookahead, double spacing, double tolerance, double smoothWeight);

//     // assumes currently that these two functions will only ever
//     // be called from one thread
//     void setWaypoints(const std::vector<PPoint>& points, bool smooth = true);
    
    
//     Pose getLookAhead(int prev_index, int next_index, Pose curPos);
//     Pose calcActLookAhead(Point curPos, int closestPointIndex);
//     int getClosestIndex(Point curPos);
//     int getClosestIndexNeon(PPoint curPos);
//     double getCurve(Pose curPos, Pose next);

//     int index() const { return curIndex; }
// };
// #endif  // INC_7405K_2021_2022_PUREPURSUIT_H
