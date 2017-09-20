//
// Created by glawless on 23.05.17.
//

#include <target_tracker_distributed_kf/DistributedKF3D.h>

int main(int argc, char* argv[]){

  ros::init(argc, argv, "target_tracker_kf");
  target_tracker_distributed_kf::DistributedKF3D tracker;
  ros::spin();

  return EXIT_SUCCESS;
}