#include "main.h"
#include "okapi/api.hpp"
#include <cmath>
#include <vector>
#include <array>
using namespace okapi;

class PathFollower {
    public:
    // follower coordinates
    float position[2];

    // follower speed
    float speed;

    //status
    bool null;

    PathFollower(float x, float y, float set_speed) {
        position[0] = float(x); 
        position[1] = float (y);
        //position[0] = chassis->getState()[0];
        //position[1] = chassis->getState()[1];
        speed = set_speed;
        null=false;
    }

    /**
     * Moves the follower towards a point by the follower's speed.
     *
     * @param x The x value of the coordinate towards which to move.
     * @param y The y value of the coordinate towards which to move.
     */
    void moveFollowerTowardsPoint(float x, float y) {
        // move the point to origin (the follower's coordinates)
        float offsetX = x - position[0];
        float offsetY = y - position[1];

        // normalize the vector
        float distanceToPoint = (float) sqrt(offsetX * offsetX + offsetY * offsetY);
        float normalizedX = offsetX / distanceToPoint;
        float normalizedY = offsetY / distanceToPoint;

        // move towards the point at a certain speed
        position[0] += normalizedX * speed;
        position[1] += normalizedY * speed;

        //position[0] = chassis->getState()[0];
        //position[1] = chassis->getState()[1];

        //Drive motors
    }

    /**
     * Returns the coordinates of the follower.
     *
     * @return A float[2], with arr[0] being the x value and arr[1] being the y value.
     */
    float *getFollowerPosition() {
        return position;
    }

    //Sets the status of null
    void setStat(bool input){
        null = input;
    }
    
    //Returns null
    bool getStat(){
        return null;
    }
}

class PurePursuit {
    public:
    // list of the points of the path
    std::vector<float[2]> path;

    // list of the points of the path drawn by the follower
    std::vector<float[2]> followerPath;

    // a PathFollower object and its variables
    PathFollower follower;
    float followerSpeed = 2.5f;
    float followerStopDistance = 2;

    // the value by which the stop distance changes
    float lookaheadDistanceDelta = 2.5f;

    // size of the points
    float pointSize = 4;

    // the lookahead distance
    float lookaheadDistance = 45;

    /**
     * Resets the simulation by setting the follower and all of the paths to equal null.
     */
    void reset() {
        path.clear();

        follower.setStat(true);
        followerPath.clear();
    }

    void setPath(std::vector<float[2]> inputPath){
        path = inputPath;
    }

    void drive() {
        // draw and potentially move the PathFollower
        if (!follower.getStat()) {
            float position[2] = {0,0};
            position[0] = float(follower.getFollowerPosition()[0]);
            position[1] = float(follower.getFollowerPosition()[1]);
            float lookahead[2] = {0,0};
            lookahead[0] = getLookaheadPoint(position[0], position[1], lookaheadDistance)[0];
            lookahead[1] = getLookaheadPoint(position[0], position[1], lookaheadDistance)[0];

            // if lookahead exists
            if (true) { //!lookahead.empty()

                // calculate the distance to the lookahead point
                double deltaX = lookahead[0] - position[0];
                double deltaY = lookahead[1] - position[1];
                float distance = 2 * (float) sqrt(pow(deltaX, 2) + pow(deltaY, 2));

                // if the follower reached the destination, delete the follower
                if (distance < followerStopDistance) {
                    follower.setStat(true);
                } else {
                    // move the follower upon pressing 'f'
                    if (true) {
                        // add the follower's current position to its path
                        float followerPosition[2] = {0,0};
                        position[0] = float(follower.getFollowerPosition()[0]);
                        position[1] = float(follower.getFollowerPosition()[1]);
                        followerPath.push_back({float(followerPosition[0]), float(followerPosition[1])});

                        // move it
                        follower.moveFollowerTowardsPoint(lookahead[0], lookahead[1]);
                    }
                }
            }
        }
    }

    /**
     * Returns the sign of the input number n. Note that the function returns 1 for n = 0 to satisfy the requirements
     * set forth by the line-circle intersection formula.
     *
     * @param n The number to return the sign of.
     * @return A float value of the sign of the number (-1.0f for n < 0, else 1.0f).
     */
    float signum(float n) {
        if (n == 0) return 1;
        else return signum(n);
    }

    /**
     * Generate the furthest lookahead point on the path that is distance r from the point (x, y).
     *
     * @param x The x of the origin.
     * @param y The y of the origin.
     * @param r The lookahead distance.
     * @return A float[] coordinate pair if the lookahead point exists, or null.
     * @see <a href="http://mathworld.wolfram.com/Circle-LineIntersection.html">Circle-Line Intersection</a>
     */
    float *getLookaheadPoint(float x, float y, float r) {
        float lookahead[2];
        bool lookaheadNull = true;

        // iterate through all pairs of points
        for (int i = 0; i < path.size() - 1; i++) {
            // form a segment from each two adjacent points
            float segmentStart[2] = {0,0};
            segmentStart[0] = path.at(i)[0];
            segmentStart[1] = path.at(i)[1];
            float segmentEnd[2] = {0,0};
            segmentEnd[0] = path.at(i + 1)[0];
            segmentEnd[1] = path.at(i + 1)[1];

            // translate the segment to the origin
            float p1[] = {float(segmentStart[0] - x), float(segmentStart[1] - y)};
            float p2[] = {float(segmentEnd[0] - x), float(segmentEnd[1] - y)};

            // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
            float dx = p2[0] - p1[0];
            float dy = p2[1] - p1[1];
            float d = (float) sqrt(dx * dx + dy * dy);
            float D = p1[0] * p2[1] - p2[0] * p1[1];

            // if the discriminant is zero or the points are equal, there is no intersection
            float discriminant = r * r * d * d - D * D;
            if ((discriminant <= 0) || (p1 == p2)) continue;

            // the x components of the intersecting points
            float x1 = (float) (D * dy + signum(dy) * dx * sqrt(discriminant)) / (d * d);
            float x2 = (float) (D * dy - signum(dy) * dx * sqrt(discriminant)) / (d * d);

            // the y components of the intersecting points
            float y1 = (float) (-D * dx + abs(dy) * sqrt(discriminant)) / (d * d);
            float y2 = (float) (-D * dx - abs(dy) * sqrt(discriminant)) / (d * d);

            // whether each of the intersections are within the segment (and not the entire line)
            bool validIntersection1 = std::min(p1[0], p2[0]) < x1 && x1 < std::max(p1[0], p2[0])
                    || std::min(p1[1], p2[1]) < y1 && y1 < std::max(p1[1], p2[1]);
            bool validIntersection2 = std::min(p1[0], p2[0]) < x2 && x2 < std::max(p1[0], p2[0])
                    || std::min(p1[1], p2[1]) < y2 && y2 < std::max(p1[1], p2[1]);

            // remove the old lookahead if either of the points will be selected as the lookahead
            if (validIntersection1 || validIntersection2) lookaheadNull = true;

            // select the first one if it's valid
            if (validIntersection1) {
                lookahead[0] = float(x1 + x);
                lookahead[1] = float(y1 + y);
                lookaheadNull = false;
            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookaheadNull || abs(x1 - p2[0]) > abs(x2 - p2[0]) || abs(y1 - p2[1]) > abs(y2 - p2[1])) {
                    lookahead[0] = float(x2 + x);
                    lookahead[1] = float(y2 + y);
                    lookaheadNull = false;
                }
            }
        }

        // special case for the very last point on the path
        if (path.size() > 0) {
            float lastPoint[2] = {path.at(path.size() - 1)[0], path.at(path.size() - 1)[1]};

            float endX = lastPoint[0];
            float endY = lastPoint[1];

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= r) {
                float returnArr[2] = {float(endX), float(endY)};
                return returnArr;
            }
        }

        return lookahead;
    }

    /**
     * Is called when a key is pressed; controls most of the program controls.
     */
    void start() {
        if (path.size() > 0) {
            float firstPointCoordinates[2] = {0,0};
            firstPointCoordinates[0] = path.at(0)[0];
            firstPointCoordinates[1] = path.at(0)[1];

            std::vector<float> followerPath;
            follower = PathFollower(firstPointCoordinates[0], firstPointCoordinates[1], followerSpeed);
        }
    }

}