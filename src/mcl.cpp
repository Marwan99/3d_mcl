#include <random>

#include <mcl/mcl.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#define NUM_PARTICLES 100

MCL::MCL(ros::NodeHandle &nh): motion_model(nh), measurement_model(nh)
{
    nh_ = nh;
    vis_pub_  = nh.advertise<visualization_msgs::MarkerArray>("/particles", 10, true);
    pose_pub_ = nh.advertise<nav_msgs::Odometry>("/mcl_odom", 1, true);

    // Initializing particles
    for(int i = 0; i < NUM_PARTICLES; i++)
        // particles.push_back(pose((double)std::rand() / RAND_MAX * 1.0 -0.5, (double)std::rand() / RAND_MAX  * 1.0 - 0.5, (((double)std::rand() / (RAND_MAX))*6.24) - 3.14));
        particles.push_back(pose(
            (double)std::rand() / RAND_MAX * 1.0 -0.5,
            (double)std::rand() / RAND_MAX  * 1.0 - 0.5, 
            0));
        // particles.push_back(pose(0, 0, 0));
    
    publish_markers();
}

void MCL::filter()
{
    if(measurement_model.scan_available && motion_model.odom_initialized)
    {
        // ros::Time time = ros::Time::now();
        measurement_model.scan_available = false;
        ROS_INFO("Filtering*************************************");

        // ros::Time motion_time = ros::Time::now();
        motion_model.update_pose(particles);
        // ROS_INFO("Motion time: %f", (ros::Time::now() - motion_time).toSec());

        if(!motion_model.moved)
            return;

        // ros::Time measurement_time = ros::Time::now();
        measurement_model.calculate_weights(particles);
        // ROS_INFO("Measurement time: %f", (ros::Time::now() - measurement_time).toSec());

        ROS_INFO("Resampling");
        low_var_respampling();
        publish_markers();
        publish_estimated_pose();

        // motion_model.reset_pre_integration();
        // ROS_INFO("Total time: %f", (ros::Time::now() - time).toSec());
        ROS_INFO("Iteration complete-----------------------------");
    }
}

void MCL::publish_markers()
{
    visualization_msgs::Marker temp_marker;
    visualization_msgs::MarkerArray markers_list;
    tf2::Quaternion quaternion;

    temp_marker.header.frame_id = "odom";
    temp_marker.header.stamp = ros::Time();
    temp_marker.ns = "particles_space";
    temp_marker.type = visualization_msgs::Marker::ARROW;
    temp_marker.action = visualization_msgs::Marker::MODIFY;
    temp_marker.pose.position.z = 0;
    temp_marker.scale.x = 0.2;
    temp_marker.scale.y = 0.025;
    temp_marker.scale.z = 0.025;
    temp_marker.color.a = 1.0; // Don't forget to set the alpha!
    temp_marker.color.r = 0.0;
    temp_marker.color.g = 1.0;
    temp_marker.color.b = 0.0;

    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        temp_marker.id = i;

        temp_marker.pose.position.x = particles[i].x;
        temp_marker.pose.position.y = particles[i].y;

        quaternion.setRPY(0, 0, particles[i].yaw);
        temp_marker.pose.orientation = tf2::toMsg(quaternion);

        markers_list.markers.push_back(temp_marker);
    }

    vis_pub_.publish(markers_list);
}

void MCL::publish_estimated_pose()
{
    normalise_weights();
    nav_msgs::Odometry odom_msg;

    odom_msg.header.frame_id = "odom";
    
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;

    double estimated_yaw = 0;

    for(auto particle : particles)
    {
        odom_msg.pose.pose.position.x += particle.x * particle.weight;
        odom_msg.pose.pose.position.y += particle.y * particle.weight;
        estimated_yaw += particle.yaw * particle.weight;
    }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, estimated_yaw);
    tf2::convert(quaternion, odom_msg.pose.pose.orientation);

    pose_pub_.publish(odom_msg);
}

void MCL::low_var_respampling()
{
    normalise_weights();
    // std::vector<pose> sampled_particles;
    
    // std::random_device device;	
    // std::mt19937 generator(device());
    // std::uniform_real_distribution<double> uni_dist(0.0, 1/NUM_PARTICLES);

    // double r = uni_dist(generator);
    // double c = particles[0].weight;
    // double u;

    // for(int i = 0; i < NUM_PARTICLES; i++)
    // {
    //     u = r + (i/NUM_PARTICLES);
    //     ROS_ERROR_STREAM(i << " " << u << " " << c << " " << particles[i].weight);
    //     while(u > c)
    //     {
    //         i++;
    //         c += particles[i].weight;
    //     }
    //     sampled_particles.push_back(particles[i]);
    // }

    // particles = sampled_particles;

    std::vector<pose> x_t;
    std::vector<double> weights;

    // extract all the weights into a vector
    for(auto particle : particles)
        weights.push_back(particle.weight);


    std::random_device device;
    std::mt19937 generator(device());
    std::discrete_distribution<int> weights_dist(weights.begin(), weights.end());

    for(int i=0; i<particles.size(); i++)
    {
        int drawn_particle_index = weights_dist(generator);
        x_t.push_back(particles[drawn_particle_index]);
    }

    particles = x_t;
}

void MCL::normalise_weights()
{
    double sum = 0;

    for(auto particle : particles)
        sum += particle.weight;

    if(sum == 0)
        return;

    for(auto & particle : particles)
        particle.weight /= sum;
}
