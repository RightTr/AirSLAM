#include <fstream>
#include <math.h>

#include "dataset.h"
#include "ros2_publisher.h"
#include "utils.h"
#include "imu.h"

#include <regex>


double extractTimestamp(const std::string& filename) {
  size_t dot_pos = filename.rfind('.');
  if (dot_pos == std::string::npos) {
      throw std::runtime_error("Invalid filename: no extension found -> " + filename);
  }
  std::string ts_str = filename.substr(0, dot_pos);
  try {
      return std::stod(ts_str);
  } catch (...) {
      throw std::runtime_error("Invalid timestamp format in filename: " + filename);
  }
}

void sortByTimestamp(std::vector<std::string>& files) {
  std::sort(files.begin(), files.end(),
      [](const std::string& a, const std::string& b) {
          return extractTimestamp(a) < extractTimestamp(b);
      });
}

Dataset::Dataset(const std::string& dataroot, const bool use_imu): _use_imu(use_imu){
  if(!PathExists(dataroot)){
    std::cout << "dataroot : " << dataroot << " doesn't exist" << std::endl;
    exit(0);
  }
  else
  {
    std::cout << "dataroot : " << dataroot << std::endl;
  }
  std::string imu_file = ConcatenateFolderAndFileName(dataroot, "imu0/data.csv");
  if(use_imu && !FileExists(imu_file)){
    std::cout << "use_imu is set to true, however the imu file : " << imu_file << " doesn't exist" << std::endl;
    exit(0);
  }

  // std::string left_image_dir = ConcatenateFolderAndFileName(dataroot, "pose_interp_left_fs");
  // std::string right_image_dir = ConcatenateFolderAndFileName(dataroot, "pose_interp_right_fs");

  // std::string left_image_dir = ConcatenateFolderAndFileName(dataroot, "left_thermal/left_motion");
  // std::string right_image_dir = ConcatenateFolderAndFileName(dataroot, "right_thermal/right_motion");

  std::string left_image_dir = ConcatenateFolderAndFileName(dataroot, "left_thermal/image");
  std::string right_image_dir = ConcatenateFolderAndFileName(dataroot, "right_thermal/image");

  std::vector<std::string> image_names_left;
  std::vector<std::string> image_names_right;
  GetFileNames(left_image_dir, image_names_left);
  GetFileNames(right_image_dir, image_names_right);
  if(image_names_left.size() < 1 || image_names_right.size() < 1) return;

  ImuDataList all_imu_data;
  if(use_imu){
    ReadImuData(imu_file, all_imu_data);
  }
  size_t num_imu_data = all_imu_data.size();

  sortByTimestamp(image_names_left);
  sortByTimestamp(image_names_right);

  std::cout << "images_left size: " << image_names_left.size() << std::endl;
  std::cout << "images_right size: " << image_names_right.size() << std::endl;

  std::vector<std::string> image_names = image_names_left;
  for(size_t i = 0; i < image_names.size(); ++i){
    // double image_time = atof(image_names[i].substr(0, 10).c_str()) + atof(image_names[i].substr(10, image_names[i].find_last_of('.')-10).c_str()) / 1e9;
    double image_time = ImageNameToTime(image_names[i]);
    if(num_imu_data > 0){
      // discard images without imu data 
      if(image_time < all_imu_data[0].timestamp) continue;
      if(image_time > all_imu_data[num_imu_data-1].timestamp) break;
    }

    _left_images.emplace_back(ConcatenateFolderAndFileName(left_image_dir, image_names_left[i]));
    _right_images.emplace_back(ConcatenateFolderAndFileName(right_image_dir, image_names_right[i]));
    _timestamps.emplace_back(image_time);  
  }

  if(num_imu_data > 0){
    size_t imu_idx = 0;
    double last_image_time = -1;
    for(double image_time : _timestamps){
      ImuDataList mini_batch_imu_data;
      for(; imu_idx < all_imu_data.size()-1; imu_idx++){
        if(all_imu_data[imu_idx+1].timestamp < last_image_time) continue;
        mini_batch_imu_data.emplace_back(all_imu_data[imu_idx]);
        if(all_imu_data[imu_idx].timestamp > image_time) break;
      }
      imu_idx--;
      
      last_image_time = image_time;
      _imu_data.emplace_back(mini_batch_imu_data);
    }
  }
  std::cout << "Dataset loaded successfully!" << std::endl;
}

void Dataset::ReadImuData(const std::string& imu_file_path, ImuDataList& all_imu_data){
  if(!FileExists(imu_file_path)){
    std::cout << "imu file : " << imu_file_path << " doesn't exist" << std::endl;
    exit(0);
  }

  std::vector<std::vector<std::string> > lines;
  ReadTxt(imu_file_path, lines, ",");
  all_imu_data.resize((lines.size()-1));
  for(size_t i = 1; i < lines.size(); ++i){
    all_imu_data[i-1].timestamp = StringTimeToDouble(lines[i][0]);
    all_imu_data[i-1].gyr << atof(lines[i][1].c_str()), atof(lines[i][2].c_str()), atof(lines[i][3].c_str()); 
    all_imu_data[i-1].acc << atof(lines[i][4].c_str()), atof(lines[i][5].c_str()), atof(lines[i][6].c_str()); 
  }
}

size_t Dataset::GetDatasetLength(){
  return _left_images.size();
}

bool Dataset::GetData(size_t idx, cv::Mat& left_image, cv::Mat& right_image, ImuDataList& batch_imu_data, double& timestamp){
  batch_imu_data.clear();
  if(idx >= _left_images.size()) return false;
  if(!FileExists(_left_images[idx]) || !FileExists(_right_images[idx])) return false;
  std::cout << _left_images[idx] << std::endl;
  std::cout << _right_images[idx] << std::endl;
  left_image = cv::imread(_left_images[idx], 0);
  right_image = cv::imread(_right_images[idx], 0);
  timestamp = _timestamps[idx];
  if(_imu_data.size() > idx){
    std::copy(_imu_data[idx].begin(), _imu_data[idx].end(), std::back_inserter(batch_imu_data));
  }
  return true;
}