#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "ros_utils.h"
#include "utils.h"
#include "read_configs.h"
#include "map.h"
#include "map_refiner.h"

int main(int argc, char **argv) {
  ros_init(argc, argv, "air_slam");
  auto node = make_ros_node("air_slam");

  int breakpoint;
  ros_get_parameter(node, "breakpoint", breakpoint, 0);

  std::string config_path, model_dir;
  ros_get_parameter(node, "config_path", config_path, std::string(""));
  ros_get_parameter(node, "model_dir", model_dir, std::string(""));

  MapRefinementConfigs configs(config_path, model_dir);
  MapRefiner map_refiner(configs, node);
  
  std::string map_root;
  ros_get_parameter(node, "map_root", map_root, std::string(""));
  std::cout << "Loading map and vocabulary..." << std::endl;
  map_refiner.LoadMap(map_root);

  std::string voc_path;
  ros_get_parameter(node, "voc_path", voc_path, std::string(""));
  map_refiner.LoadVocabulary(voc_path);
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Building covisibility graph..." << std::endl;
  map_refiner.UpdateCovisibilityGraph();
  std::cout << "Done." << std::endl;

  std::cout << "Loop detection..." << std::endl;
  int loop_num = map_refiner.LoopDetection();
  std::cout << "Done, " << loop_num << " loop pairs are found." << std::endl;

  std::cout << "Optimizing pose graph..." << std::endl;
  map_refiner.PoseGraphRefinement();
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Merging mappoints..." << std::endl;
  map_refiner.MergeMap();
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Optimizing global map..." << std::endl;
  map_refiner.GlobalMapOptimization();
  map_refiner.UpdateCovisibilityGraph();
  std::cout << "Done." << std::endl;

  std::cout << "Build junction database..." << std::endl;
  map_refiner.BuildJunctionDatabase();
  std::cout << "Done." << std::endl;

  std::string trajectory_global_ba_path = ConcatenateFolderAndFileName(map_root, "trajectory_v1.txt");
  map_refiner.SaveTrajectory(trajectory_global_ba_path);

  map_refiner.Wait(breakpoint);

  std::cout << "Saving final map..." << std::endl;
  map_refiner.SaveFinalMap(map_root);
  std::cout << "Done." << std::endl;

  exit(0);
  map_refiner.StopVisualization();
  ros_shutdown();

  return 0;
}
