/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Mark Moll */

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <omplapp/apps/DynamicCarPlanning.h>
#include <omplapp/config.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_broadcaster.h>

class MarkerPublisher
{
public:
    MarkerPublisher():
        marker_list_(),
        marker_(),
        br(),
        nh_()
    {
        marker_.header.frame_id = "/world";
        marker_.type = visualization_msgs::Marker::ARROW;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.scale.x = 0.4;
        marker_.scale.y = 0.2;
        marker_.scale.z = 0.2;
        marker_.color.a = 1.0;
        marker_.color.r = 1.0;
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>( "poses_array", 0 );
    }

    void addMarker(const double x, const double y, const double pitch)
    {
        marker_.id = counter_++;
        //marker.header.stamp = trafos[i].stamp_;
        marker_.pose.position.x = x;
        marker_.pose.position.y = y;
        marker_.pose.position.z = 0.0;
        marker_.pose.orientation.x = cos(pitch/2);
        marker_.pose.orientation.y = sin(pitch/2);
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 0.0;
        ROS_INFO_STREAM("orientation " << marker_.pose.orientation.x << " "
                        << marker_.pose.orientation.y << " "
                        << marker_.pose.orientation.z << " "
                        << marker_.pose.orientation.w << " ");
        marker_list_.markers.push_back(marker_);
    }

    void publish()
    {
        br.sendTransform(tf::StampedTransform(tf::Transform(),ros::Time::now(),"world","path"));
        pub_markers_.publish(marker_list_);
    }

private:
    visualization_msgs::MarkerArray marker_list_;
    visualization_msgs::Marker marker_;
    tf::TransformBroadcaster br;
    ros::Publisher pub_markers_;
    ros::NodeHandle nh_;
    int counter_;
};

using namespace ompl;

bool isStateValid(const base::State *state)
{
//    const base::SE2StateSpace::StateType& s0 =
//        *(state->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0));
//    ROS_INFO_STREAM("[" << s0.getX() << "," << s0.getY() << "]");
//    if(s0.getX() > 1 && s0.getX() < 7 && s0.getY() > 1 && s0.getY() < 7)
//        return false;
//    else
        return false;
}

void dynamicCarSetup(app::DynamicCarPlanning& setup)
{
    // plan for dynamic car in SE(2)
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE2StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<> start(stateSpace);
    start[0] = start[1] = start[2] = start[3] = start[4] = 0.;

    // define goal state
    base::ScopedState<> goal(stateSpace);
    goal[0] = goal[1] = 10.;
    goal[2] = 0;
    goal[3] = goal[4] = 0.;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .5);

    setup.setStateValidityChecker(boost::bind(&isStateValid, _1));
    setup.setStateValidityCheckerType(app::FCL);
}

void dynamicCarDemo(app::DynamicCarPlanning& setup)
{
    MarkerPublisher marker_pub;

    std::cout<<"\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    //setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    setup.setPlanner(base::PlannerPtr(new control::EST(setup.getSpaceInformation())));
    //setup.setPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
    std::vector<double> cs(2);
    cs[0] = cs[1] = 0.1;
    setup.setup();
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    // try to solve the problem
    if (setup.solve(40))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl& path(setup.getSolutionPath());
        //path.interpolate(); // uncomment if you want to plot the path
        for (unsigned int i=0; i<path.getStateCount(); ++i)
        {
            const base::SE2StateSpace::StateType& s0 =
                *path.getState(i)->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType& s1 =
                *path.getState(i)->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
            std::cout << s0.getX() <<' '<< s0.getY() << ' ' << s0.getYaw() << ' ';
            std::cout << s1[0] <<' '<< s1[1] << ' ';

            marker_pub.addMarker(s0.getX(),s0.getY(),s0.getYaw());

            if (i==0)
                // null controls applied for zero seconds to get to start state
                std::cout << "0 0 0";
            else
            {
                // print controls and control duration needed to get from state i-1 to state i
                const double* c = path.getControl(i-1)->as<control::RealVectorControlSpace::ControlType>()->values;
                std::cout << c[0] << ' ' << c[1] << ' ' << path.getControlDuration(i-1);
            }
            std::cout << std::endl;
        }
        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is " <<
                setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
    ROS_INFO("done");
    marker_pub.publish();
    ros::spinOnce ();
}
void dynamicCarBenchmark(app::DynamicCarPlanning& setup)
{
    tools::Benchmark::Request request(100., 10000., 10); // runtime (s), memory (MB), run count

    setup.setup ();

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char* argv[])
{
    ros::init (argc, argv, "ompl_car");
    app::DynamicCarPlanning car;
    dynamicCarSetup(car);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once and print the path.
    if (argc>1)
        dynamicCarBenchmark(car);
    else
        dynamicCarDemo(car);

    ros::spin();
    return 0;
}
