//
// Created by malintha on 9/30/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "prmImpl.h"

Node runRRT(ros::Publisher, int);

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher);

void populateRviz(ros::Publisher marker_pub);

void populateObstacles(ros::Publisher marker_pub);

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub);

void computeNeighborGraph(ros::Publisher marker_pub, int frameid);

static int numNodes = 200;
static RRT rrt = RRT(20, 0, 20, 0, numNodes);


static std::vector<visualization_msgs::Marker> obsVec;
static int path_node_index;
static bool pn_index_initialized = false;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_rrt");
    static ros::NodeHandle n;
    static ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
    ros::Rate loop_rate(20);
    int frame_count = 0;

    populateRviz(marker_pub);
    rrt.generateRandomPoints(obsVec);
    rrt.computeNeighborGraph(obsVec);

    bool first =true;
    while (ros::ok()) {
        ROS_INFO("Frame: %d", frame_count);
        populateRviz(marker_pub);


        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please run Rviz in another terminal.");
            sleep(1);
        }

        if (first){
            // Visualize PRM in Red
            auto nodes = rrt.getNodes();
            for (auto& i: nodes)
            {   
                for (auto& j: i.neighbors)
                {
                    addEdge(i.point, j->point, marker_pub);
                }
            }
            first =false;            
        }

        if (frame_count > 10)
        {
            // Plan on the PRM from start to goal and visualize in yell
            auto path = rrt.solveShortestPath();
            for (int i=0; i< path.size()-1;i++) {
                drawFinalPath(rrt.getById(path[i]).point, rrt.getById(path[i+1]).point, marker_pub);
            }
        }

        //iterate ROS
        ros::spinOnce();
        loop_rate.sleep();
        ++frame_count;
    }

    return 0;
}

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge, vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.02;
    edge.color.r = 1.0;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "finalPath";
    edge.id = 4;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.04;

    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}


void populateRviz(ros::Publisher marker_pub) {
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    v_start.header.frame_id = v_end.header.frame_id = "map";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    v_start.scale.x = v_start.scale.y = 0.2;
    v_end.scale.x = v_end.scale.y = 0.2;

    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;

    geometry_msgs::Point ps, pe;
    ps.x = 0;
    ps.y = 0;
    pe.x = 18;
    pe.y = 18;
    v_start.points.push_back(ps);
    v_end.points.push_back(pe);

    //publish edge and vertices
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);

    populateObstacles(marker_pub);
};

void populateObstacles(ros::Publisher marker_pub) {
    visualization_msgs::Marker obs1, obs2, obs3, obs4, obs5, obs6;

    obs1.type = obs2.type = obs3.type = obs4.type = obs5.type = obs6.type = visualization_msgs::Marker::CUBE;
    obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = obs5.header.frame_id = obs6.header.frame_id = "map";
    obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = obs5.header.stamp = obs6.header.stamp = ros::Time::now();
    obs1.ns = obs2.ns = obs3.ns = obs4.ns = obs5.ns = obs6.ns = "obstacles";
    obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = obs5.lifetime = obs6.lifetime = ros::Duration();
    obs1.action = obs2.action = obs3.action = obs4.action = obs5.action = obs6.action = visualization_msgs::Marker::ADD;

    obs1.id = 0;
    obs2.id = 1;
    obs3.id = 2;
    obs4.id = 3;
    obs5.id = 4;
    obs6.id = 5;

    obs1.scale.x = obs2.scale.x = 2;
    obs1.scale.y = obs2.scale.y = 12;
    obs3.scale.y = 4;
    obs3.scale.x = 7;
    obs4.scale.x = obs4.scale.y = 2;
    obs5.scale.x = 5;
    obs5.scale.y = 3;
    obs6.scale.x = 3;
    obs6.scale.y = 7;

    obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = obs5.scale.z = obs6.scale.z = 0.25;


    obs1.pose.position.x = 8;
    obs1.pose.position.y = 6;
    obs1.pose.position.z = 0.25;
    obs1.pose.orientation.x = 0.0;
    obs1.pose.orientation.y = 0.0;
    obs1.pose.orientation.z = 0.0;
    obs1.pose.orientation.w = 1;
    obs1.color.a = 1;
    obs1.color.r = obs1.color.g = obs1.color.b = 6.6f;

    obs2.pose.position.x = 14;
    obs2.pose.position.y = 14;
    obs2.pose.position.z = 0.25;
    obs2.pose.orientation.x = 0.0;
    obs2.pose.orientation.y = 0.0;
    obs2.pose.orientation.z = 0.0;
    obs2.pose.orientation.w = 1;
    obs2.color.a = 1;
    obs2.color.r = obs2.color.g = obs2.color.b = 6.6f;

    obs3.pose.position.x = 16.5;
    obs3.pose.position.y = 2;
    obs3.pose.position.z = 0.25;
    obs3.pose.orientation.x = 0.0;
    obs3.pose.orientation.y = 0.0;
    obs3.pose.orientation.z = 0.0;
    obs3.pose.orientation.w = 1;
    obs3.color.a = 1;
    obs3.color.r = obs3.color.g = obs3.color.b = 6.6f;

    obs4.pose.position.x = 16;
    obs4.pose.position.y = 9;
    obs4.pose.position.z = 0.25;
    obs4.pose.orientation.x = 0.0;
    obs4.pose.orientation.y = 0.0;
    obs4.pose.orientation.z = 0.0;
    obs4.pose.orientation.w = 1;
    obs4.color.a = 1;
    obs4.color.r = obs4.color.g = obs4.color.b = 6.6f;

    obs5.pose.position.x = 2.5;
    obs5.pose.position.y = 18.5;
    obs5.pose.position.z = 0.25;
    obs5.pose.orientation.x = 0.0;
    obs5.pose.orientation.y = 0.0;
    obs5.pose.orientation.z = 0.0;
    obs5.pose.orientation.w = 1;
    obs5.color.a = 1;
    obs5.color.r = obs5.color.g = obs5.color.b = 6.6f;

    obs6.pose.position.x = 1.5;
    obs6.pose.position.y = 13.5;
    obs6.pose.position.z = 0.25;
    obs6.pose.orientation.x = 0.0;
    obs6.pose.orientation.y = 0.0;
    obs6.pose.orientation.z = 0.0;
    obs6.pose.orientation.w = 1;
    obs6.color.a = 1;
    obs6.color.r = obs6.color.g = obs6.color.b = 6.6f;

    marker_pub.publish(obs1);
    marker_pub.publish(obs2);
    marker_pub.publish(obs3);
    marker_pub.publish(obs4);
    marker_pub.publish(obs5);
    marker_pub.publish(obs6);

    obsVec.push_back(obs1);
    obsVec.push_back(obs2);
    obsVec.push_back(obs3);
    obsVec.push_back(obs4);
    obsVec.push_back(obs5);
    obsVec.push_back(obs6);
}