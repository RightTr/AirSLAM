#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include <thread>

#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("air_slam");

  std::string config_path, model_dir;
  node->declare_parameter("config_path", "");
  node->declare_parameter("model_dir", "");
  node->declare_parameter("dataroot", "");
  node->declare_parameter("camera_config_path", "");
  node->declare_parameter("saving_dir", "");

  node->get_parameter("config_path", config_path);
  node->get_parameter("model_dir", model_dir);

  VisualOdometryConfigs configs(config_path, model_dir);
  std::cout << "config done" << std::endl;

  node->get_parameter("dataroot", configs.dataroot);
  RCLCPP_INFO(node->get_logger(), "dataroot: %s", configs.dataroot.c_str());

  node->get_parameter("camera_config_path", configs.camera_config_path);
  RCLCPP_INFO(node->get_logger(), "camera_config_path: %s", configs.camera_config_path.c_str());

  node->get_parameter("saving_dir", configs.saving_dir);
  RCLCPP_INFO(node->get_logger(), "saving_dir: %s", configs.saving_dir.c_str());

  MapBuilder map_builder(configs, node);
  std::cout << "map_builder done" << std::endl;

  Dataset dataset(configs.dataroot, map_builder.UseIMU());
  size_t dataset_length = dataset.GetDatasetLength();
  std::cout << "dataset done" << std::endl;

  double sum_time = 0;
  int image_num = 0;
  for(size_t i = 0; i < dataset_length && rclcpp::ok(); ++i){
    std::cout << "i ====== " << i << std::endl;
    cv::Mat image_left, image_right;
    double timestamp;
    ImuDataList batch_imu_data;
    if(!dataset.GetData(i, image_left, image_right, batch_imu_data, timestamp)) continue;

    InputDataPtr data = std::shared_ptr<InputData>(new InputData());
    data->index = i;
    data->time = timestamp;
    data->image_left = image_left;
    data->image_right = image_right;
    data->batch_imu_data = batch_imu_data;

    auto before_infer = std::chrono::high_resolution_clock::now();   
    map_builder.AddInput(data);
    auto after_infer = std::chrono::high_resolution_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
    sum_time += (double)cost_time;
    image_num++;
    std::cout << "One Frame Processinh Time: " << cost_time << " ms." << std::endl;
  }
  std::cout << "Average FPS = " << image_num / (sum_time / 1000.0) << std::endl;


  std::cout << "Waiting to stop..." << std::endl; 
  map_builder.Stop();
  while(!map_builder.IsStopped()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Map building has been stopped" << std::endl; 

  std::string trajectory_path = ConcatenateFolderAndFileName(configs.saving_dir, "trajectory_v0.txt");
  map_builder.SaveTrajectory(trajectory_path);
  map_builder.SaveMap(configs.saving_dir);
  rclcpp::shutdown();
  
  return 0;
}
