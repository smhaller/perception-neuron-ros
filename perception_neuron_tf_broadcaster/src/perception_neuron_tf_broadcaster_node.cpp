/* perception_neuron_tf_broadcaster.cpp
 *
 * Copyright (C) 2015 Alexander Rietzler, Simon Haller
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

class NeuronBroadcaster
{
public:
    NeuronBroadcaster(ros::NodeHandle & nh)
        :nh_(nh)
    {        

         link_children_names_=std::vector<std::string>{"HipsPosition","Hips","RightUpLeg","RightLeg","RightFoot",
                                                     "LeftUpLeg","LeftLeg","LeftFoot","Spine","Spine1","Spine2","Spine3","Neck","Head",
                                                     "RightShoulder","RightArm","RightForeArm","RightHand",
                                                     "RightHandThumb1","RightHandThumb2","RightHandThumb3",
                                                     "RightInHandIndex","RightHandIndex1","RightHandIndex2","RightHandIndex3",
                                                     "RightInHandMiddle","RightHandMiddle1","RightHandMiddle2","RightHandMiddle3",
                                                     "RightInHandRing","RightHandRing1","RightHandRing2","RightHandRing3",
                                                     "RightInHandPinky","RightHandPinky1","RightHandPinky2","RightHandPinky3",
                                                     "LeftShoulder","LeftArm","LeftForeArm","LeftHand",
                                                     "LeftHandThumb1","LeftHandThumb2","LeftHandThumb3",
                                                     "LeftInHandIndex","LeftHandIndex1","LeftHandIndex2","LeftHandIndex3",
                                                     "LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2","LeftHandMiddle3",
                                                     "LeftInHandRing","LeftHandRing1","LeftHandRing2","LeftHandRing3",
                                                     "LeftInHandPinky","LeftHandPinky1","LeftHandPinky2","LeftHandPinky3"
                                                     };

        link_parents_names_=std::vector<std::string>{"WorldPerceptionNeuron","HipsPosition","Hips","RightUpLeg","RightLeg",
                                                     "Hips","LeftUpLeg","LeftLeg","Hips","Spine","Spine1","Spine2","Spine3","Neck",
                                                     "Spine3","RightShoulder","RightArm","RightForeArm",
                                                     "RightHand","RightHandThumb1","RightHandThumb2",
                                                     "RightHand","RightInHandIndex","RightHandIndex1","RightHandIndex2",
                                                     "RightHand","RightInHandMiddle","RightHandMiddle1","RightHandMiddle2",
                                                     "RightHand","RightInHandRing","RightHandRing1","RightHandRing2",
                                                     "RightHand","RightInHandPinky","RightHandPinky1","RightHandPinky2",
                                                     "Spine3","LeftShoulder","LeftArm","LeftForeArm",
                                                     "LeftHand","LeftHandThumb1","LeftHandThumb2",
                                                     "LeftHand","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
                                                     "LeftHand","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
                                                     "LeftHand","LeftInHandRing","LeftHandRing1","LeftHandRing2",
                                                     "LeftHand","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2"
                                                    };

        std::vector<std::string> topic_names={"/perception_neuron/data_1",
                                              "/perception_neuron/data_2",
                                              "/perception_neuron/data_3"};

        subscribers_.resize(3);

        for(int i=0; i <subscribers_.size(); i++){
            subscribers_.at(i)=nh_.subscribe<std_msgs::Float64MultiArray>(topic_names.at(i), 5, boost::bind(&NeuronBroadcaster::callback_i,this, _1,i));
        }

    }

    void sendStaticTransform(){
        //Sending Static transformation to ROS World
        tf::Transform world_frame;
        world_frame.setOrigin(tf::Vector3(0,0,0));
        world_frame.setRotation(tf::Quaternion(0.70711,0,0,0.70711));
        tf_broadcaster_.sendTransform(tf::StampedTransform(world_frame, ros::Time::now(), "world", "WorldPerceptionNeuron"));
    }

    ~NeuronBroadcaster(){};

private:

     ros::NodeHandle nh_;
     std::vector<ros::Subscriber> subscribers_;
     std::vector<std::string> link_children_names_, link_parents_names_;
     tf::TransformBroadcaster tf_broadcaster_;

    void eulerToQuaternion(float eulerY, float eulerX, float eulerZ, tf::Quaternion & q){

        Eigen::Matrix3f rxyz,rx,ry,rz;

        rx=Eigen::AngleAxisf(eulerX*M_PI/180, Eigen::Vector3f::UnitX());
        ry=Eigen::AngleAxisf(eulerY*M_PI/180, Eigen::Vector3f::UnitY());
        rz=Eigen::AngleAxisf(eulerZ*M_PI/180, Eigen::Vector3f::UnitZ());

        //Check Ordering in Axis Neuron! Here = YXZ
        rxyz =  ry*rx*rz;

        Eigen::Quaternionf qf(rxyz);

        q.setW(qf.w());
        q.setX(qf.x());
        q.setY(qf.y());
        q.setZ(qf.z());
    }

    void callback_i(const std_msgs::Float64MultiArrayConstPtr & bone_data, int i){

        uint startIdx=0;
        uint link_index=0;

        for(uint j=0; j < bone_data->data.size()/6; j++){

            startIdx=j*6;

            tf::Transform pose;
            float eulerY,eulerX,eulerZ;
            tf::Quaternion rotation;
            tf::Vector3 position;

            position.setX(0.01*bone_data->data[startIdx]); //conversion to meters
            position.setY(0.01*bone_data->data[startIdx+1]);
            position.setZ(0.01*bone_data->data[startIdx+2]);

            eulerY=bone_data->data[startIdx+3];
            eulerX=bone_data->data[startIdx+4];
            eulerZ=bone_data->data[startIdx+5];

            eulerToQuaternion(eulerY,eulerX,eulerZ,rotation);

            pose.setOrigin(position);
            pose.setRotation(rotation);

            link_index=j + i*20;

            tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), link_parents_names_.at(link_index), link_children_names_.at(link_index)));

        }
    }

};

int main(int argc, char** argv){

    ros::init( argc, argv, "perception_neuron_tf_broadcaster_node", ros::init_options::AnonymousName );

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle nh;
    NeuronBroadcaster neuronBroadcaster(nh);

    ros::Rate rate(20.0);
    while (nh.ok()){
        neuronBroadcaster.sendStaticTransform();
        rate.sleep();
    }

    ROS_INFO_STREAM("This is a utility to publish perception neuron bone data to tf");
    ROS_INFO_STREAM("Type any key when you are done");
    std::string mode;
    std::cin >> mode;

    ROS_INFO_STREAM("Bye!");
    return 0;
}
