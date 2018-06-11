//
// Created by malintha on 9/30/17.
//

#ifndef PROJECT_RRTIMPL_H
#define PROJECT_RRTIMPL_H

#endif

#pragma once

#include <iostream>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <random>

struct Node {
public:
    Node() {}

    Node(geometry_msgs::Point p) : point(p) {}

    int id;
    geometry_msgs::Point point;
    std::vector<Node*> neighbors;
};

class RRT {
public:
    RRT(Node init, Node goal, float sigma, int x_max, int x_min, int y_max, int y_min) : _init(init), _goal(goal),
                                                                                         sigma(sigma), x_max(x_max),
                                                                                         x_min(x_min), y_max(y_max),
                                                                                         y_min(y_min) {
        nodes.push_back(init);
        nodes.push_back(goal);
    }

    void generateRandomPoints(std::vector<visualization_msgs::Marker> obsVec) {
        int total = 0;
        while(total < 30)
        {
            geometry_msgs::Point point;

            std::random_device rand_dev;
            std::mt19937 generator(rand_dev());
            std::uniform_int_distribution<int> distr(x_min, x_max);

            Node p;
            p.point.y = distr(generator);
            p.point.x = distr(generator);
            p.point.z = 0;
            p.id = total;

            if (!intersectsObs(p.point, p.point, obsVec) && isWithinWorld(p.point)) {
                nodes.push_back(p);
                total++;
            }
        }
    }

    void computeNeighborGraph(std::vector<visualization_msgs::Marker> obsVec) {
        // decide whether to extend toward the goal or a random point
        ROS_INFO("Size: %d", (int)nodes.size());
        for (auto& i: nodes)
        {   
            for (auto& j: nodes)
            {
                if (i.id!=j.id && !intersectsObs(i.point, j.point, obsVec)) {
                    i.neighbors.push_back(&j);
                }
            }
        }
    }

    float getEuclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
    }

    bool isWithinWorld(geometry_msgs::Point p) {
        return (p.x > this->x_min && p.x < this->x_max && p.y > this->y_min && p.y < this->y_max);
    }

    bool
    intersectsObs(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obsVec) {

        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;

        for (int i = 0; i < obsVec.size(); i++) {
            visualization_msgs::Marker obs = obsVec[i];

            float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5;
            float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5;
            float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5;
            float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5;

            //check for the bottom intersection
            bool bottom = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);
            //left intersect
            bool left = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);
            //right intersect
            bool right = lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);
            //top intersect
            bool top = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

            if ((x1 > obs_xl &&  x1 < obs_xr) && (y1 > obs_yb &&  y1 < obs_yt)) {
                return true;
            }

            if ((x2 > obs_xl &&  x2 < obs_xr) && (y2 > obs_yb &&  y2 < obs_yt)) {
                return true;
            }

            if (bottom || left || right || top) {
                return true;
            }
        }
        return false;
    }

    bool lineIntersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

        // calculate the distance to intersection point
        float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {

            float intersectionX = x1 + (uA * (x2 - x1));
            float intersectionY = y1 + (uA * (y2 - y1));

            return true;
        }
        return false;
    }

    std::vector<Node> getNodes() {
        return this->nodes;
    }

    Node getById(int id) {
        for (auto& i: nodes)
        {   
            if (i.id == id){
                return i;
            }
        }
    }

private:
    Node _init, _goal;
    float sigma;
    int x_max;
    int x_min;
    int y_max;
    int y_min;
    std::vector<Node> nodes;
};
