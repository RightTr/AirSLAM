#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "utils.h"
#include "read_configs.h"
#include "map.h"
#include "map_refiner.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("air_slam");

  int breakpoint;
  node->declare_parameter<int>("breakpoint", 0);
  node->get_parameter("breakpoint", breakpoint);

  std::string config_path, model_dir;
  node->declare_parameter<std::string>("config_path", "");
  node->declare_parameter<std::string>("model_dir", "");

  node->get_parameter("config_path", config_path);
  node->get_parameter("model_dir", model_dir);

  MapRefinementConfigs configs(config_path, model_dir);
  MapRefiner map_refiner(configs, node);
  
  node->declare_parameter<std::string>("map_root", "");
  node->declare_parameter<std::string>("voc_path", "");
  std::string map_root;
  node->get_parameter("map_root", map_root);
  std::cout << "Loading map and vocabulary..." << std::endl;
  map_refiner.LoadMap(map_root);

  std::string voc_path;
  node->get_parameter("voc_path", voc_path);
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
  rclcpp::shutdown();

  return 0;
}
