// //
// // Created by zayn on 8/8/21.
// //

// #include "PurePursuit.h"
// #include "../../util/math.h"
// #include <cmath>
// #include <arm_neon.h>

// #define TO_RAD(n) n * M_PI / 180
// #define TARGET_VELOCITY 200
// #define TARGET_ACCEL 50
// #define STARTING_VEL 0

// PurePursuit::PurePursuit(Odometry& odometry, double lookahead, double spacing, double tolerance, double smoothWeight) : odom(odometry), lookahead(lookahead), curIndex(0), waypoints(), spacing(spacing), tolerance(tolerance), smoothWeight(smoothWeight), dataWeight(1.0f - smoothWeight) {}

// double PurePursuit::distance(PPoint point1, PPoint point2) {
//     return hsqrt(std::pow(point2.x_ - point1.x_, 2) + std::pow(point2.y_ - point1.y_, 2));
// }


// void PurePursuit::setWaypoints(const std::vector<PPoint>& points, bool smooth) {
//     waypoints.clear();

//     for (int i = 0; i < points.size() - 1; i++) {
//         PPoint start = points.at(i);
//         PPoint end = points.at(i + 1);

//         double vx = end.x_ - start.x_;
//         double vy = end.y_ - start.y_;
//         double magnitude = hsqrt(vx * vx + vy * vy);

//         vx = (vx / magnitude) * spacing;
//         vy = (vy / magnitude) * spacing;

//         int numOfPoints = ceil(magnitude / spacing);

//         for (int j = 0; j < numOfPoints; j++) {
//             waypoints.emplace_back(start.x_ + vx * j, start.y_ + vy * j);
//         }
//     }

//     if (smooth) {
//         std::vector<PPoint> original = waypoints;
//         double change = tolerance;

//         while (change >= tolerance) {
//             change = 0.0f;

//             for (int i = 1; i < original.size() - 1; i++) {
//                 PPoint oPoint = original.at(i);   // original point
//                 PPoint& point = waypoints.at(i);  // waypoint to modify
//                 PPoint hPoint = waypoints.at(i + 1);
//                 PPoint lPoint = waypoints.at(i - 1);

//                 double auxY = point.y_;
//                 double auxX = point.x_;

//                 point.y_ += dataWeight * (oPoint.y_ - point.y_) + smoothWeight * (lPoint.y_ + hPoint.y_ - (2.0 * point.y_));
//                 change += std::abs(auxY - point.y_);

//                 point.x_ += dataWeight * (oPoint.x_ - point.x_) + smoothWeight * (lPoint.x_ + hPoint.x_ - (2.0 * point.x_));
//                 change += std::abs(auxX - point.x_);
//             }
//         }
//     }
// }

// /**
//  * @brief Gets the next position for the robot to move to.
//  * @return Point - where the robot should move to
//  */
// int PurePursuit::getClosestIndex(Point curPos) {
//     int index = 0;
//     double temp = 10000000;

//     for (int i = curIndex; i < waypoints.size(); i++) {
//         const PPoint& potential = waypoints.at(i);
//         double dist = distance(curPos, potential);

//         if (dist < temp) {
//             temp = dist;
//             index = i;
//         }
//     }

//     curIndex = index;
//     return index;
// }

// Point PurePursuit::calcActLookAhead(Point curPos, int closestPointIndex) {
//     Point lookahead = curPos;
//     for (int i = closestPointIndex + 1; i < waypoints.size(); i++) {
//         int startPoint = i - 1;
//         int endPoint = i;

//         Point potential = getLookAhead(startPoint, endPoint, curPos);
//         if (potential != curPos) {
//             lookahead = potential;
//             break;
//         }
//     }

//     return lookahead;
// }


// Point PurePursuit::getLookAhead(int prev_index, int next_index, Point curPos) {
//     const PPoint& prev = waypoints.at(prev_index);
//     const PPoint& next = waypoints.at(next_index);

//     // Describes direction from start to end (vector)
//     double dx = next.x_ - prev.x_; 
//     double dy = next.y_ - prev.y_;

//     // Describes from center to start position (vector)
//     double fx = prev.x_ - curPos.x_;
//     double fy = prev.y_ - curPos.y_;

//     double a = dx * dx + dy * dy;
//     double b = 2 * (fx * dx + fy * dy);
//     double c = (fx * fx + fy * fy) - std::pow(lookahead, 2);
//     double discriminant = b * b - 4 * a * c;

//     if(discriminant < 0) {
//         return curPos;
//     } else {
//         discriminant = std::sqrt(discriminant);
//         double t1 = (-b - discriminant) / (2 * a);
//         double t2 = (-b + discriminant) / (2 * a);

//         if(t1 >= 0 && t1 <= 1) {
//             double px = prev.x_ + t1 * dx;
//             double py = prev.y_ + t1 * dy;
//             return Point(px, py);
//         }

//         if(t2 >= 0 && t2 <= 1) {
//             double px = prev.x_ + t2 * dx;
//             double py = prev.y_ + t2 * dy;
//             return Point(px, py);
//         }

//         return curPos;
//     }

// } 


// double PurePursuit::getCurve(Point curPos, Point lookaheadPos) {
//     double angle = curPos.heading_;

//     double a = -std::tan(angle); 
//     double b = 1;
//     double c = std::tan(angle) * curPos.y_ - curPos.x_; 

//     double x = std::abs(a * lookaheadPos.y_ + b * lookaheadPos.x_ + c) / std::sqrt(std::pow(a, 2) + std::pow(b, 2)); // 
//     double side = sign(std::sin(angle) * (lookaheadPos.y_ - curPos.y_) - std::cos(angle) * (lookaheadPos.x_ - curPos.x_));

//     return side * ((2 * x) / std::pow(lookahead, 2));
// }