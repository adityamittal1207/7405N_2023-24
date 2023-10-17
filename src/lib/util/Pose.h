#pragma once

/**
 * This is to make processing each Pose easier for the robot. Each Pose has an X coordinate, Y coordinate, and a
 * Heading.
 * @param x X Coordinate
 * @param y Y Coordinate
 * @param heading Angle/Rotation
 */
struct Pose {
    double x;
    double y;
    double theta;

    Pose() {
        this->x = 0;
        this->y = 0;
        this->theta = 0;
    }

    Pose(double x, double y) {
        this->x = x;
        this->y = y;
        this->theta = 0;
    }

    Pose(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    Pose(const Pose& Pose) {
        this->x = Pose.x;
        this->y = Pose.y;
        this->theta = Pose.theta;
    }

    void operator=(const Pose& equal) {
        this->x = equal.x;
        this->y = equal.y;
        this->theta = equal.theta;
    }

    Pose operator+(const Pose& add) {
        return Pose(this->x + add.x, this->y + add.y);
    }

    Pose operator-(const Pose& sub) {
        return Pose(this->x - sub.x, this->y - sub.y, this->theta - sub.theta);
    }

    Pose operator*(double mult) {
        return Pose(this->x * mult, this->y * mult);
    }

    void operator+=(const Pose& add) {
        this->x = this->x + add.x;
        this->y = this->y + add.y;
    }

    void operator/=(double div) {
        this->x /= div;
        this->y /= div;
    }

    double distanceTo(const Pose& to) {
        return sqrt(pow((to.x - x), 2) + pow(to.y - y, 2));
    }
    
    double angleTo(const Pose& to) {
        double tox = to.x - this->x;
        double toy = to.y - this->y;
        return std::atan2(tox, toy);
    }

    double magnitude() {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    double dot(const Pose& p) {
        return (x * p.x) + (y * p.y);
    }

    std::string toString() {
        std::string sx = std::to_string((x));
        std::string sy = std::to_string((y));
        std::string sh = std::to_string((theta));
        return "( " + sx + " , " + sy + ") h: " + sh;
    }
};