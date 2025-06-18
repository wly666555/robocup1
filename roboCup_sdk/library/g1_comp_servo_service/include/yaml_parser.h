// Copyright 2023 Unitree. All rights reserved.
#ifndef YAML_PARSER_
#define YAML_PARSER_

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>
#include <string>
#include <iostream>
#include <fstream>

class YamlParser {
 public:
  YamlParser(){}
  void setup(const std::string &yaml_file){yaml_file_ = yaml_file;node_ = YAML::LoadFile(yaml_file);}

  explicit YamlParser(const std::string &yaml_file) : yaml_file_(yaml_file) { node_ = YAML::LoadFile(yaml_file); }

  float ReadFloatFromYaml(const std::string &node_name) { return node_[node_name].as<float>(); }

  float ReadFloatFromYaml(const std::string &node_name1,const std::string &node_name2) { return node_[node_name1][node_name2].as<float>(); }
  
  float ReadFloatFromYaml(const std::string &node_name1,const std::string &node_name2,const std::string &node_name3) { return node_[node_name1][node_name2][node_name3].as<float>(); }

  int ReadIntFromYaml(const std::string &node_name) { return node_[node_name].as<int>(); }

  bool ReadBoolFromYaml(const std::string &node_name) { return node_[node_name].as<bool>(); }

  std::string ReadStringFromYaml(const std::string &node_name) { return node_[node_name].as<std::string>(); }

  Eigen::VectorXf ReadVectorFromYaml(const std::string &node_name, int rows) {
    Eigen::VectorXf vec(rows);
    for (size_t i = 0; i < node_[node_name].size(); ++i) {
      vec(i) = node_[node_name][i].as<float>();
    }
    return vec;
  }

  Eigen::VectorXf ReadVectorFromYaml(const std::string &node_name) {
    Eigen::VectorXf vec(node_[node_name].size());
    for (size_t i = 0; i < node_[node_name].size(); ++i) {
      vec(i) = node_[node_name][i].as<float>();
    }
    return vec;
  }

  Eigen::VectorXf ReadVectorFromYaml(const std::string &node_name1,const std::string &node_name2,int rows) {
    Eigen::VectorXf vec(rows);
    vec.setZero(rows);
    std::vector<float> values;
    const YAML::Node& nodes = node_[node_name1][node_name2];
    for (const auto& node : nodes) 
    {
      float value = node.second.as<float>();     
      values.push_back(value);
    }
    for (int i = 0; i < rows; ++i) {
      vec(i) = values[i]; 
    }
    return vec;
  }

  Eigen::ArrayXf ReadArrayVectorFromYaml(const std::string &node_name, int rows) {
    Eigen::ArrayXf vec(rows);
    for (size_t i = 0; i < node_[node_name].size(); ++i) {
      vec(i) = node_[node_name][i].as<float>();
    }
    return vec;
  }

  Eigen::MatrixXf ReadMatrixFromYaml(const std::string &node_name, int rows, int cols) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat(rows, cols);
    int count = 0;
    for (YAML::const_iterator it = node_[node_name].begin(); it != node_[node_name].end(); ++it) {
      for (size_t j = 0; j < it->size(); ++j) {
        mat(count) = (*it)[j].as<float>();
        count++;
      }
    }
    return mat;
  }

  Eigen::ArrayXXf ReadArrayMatrixFromYaml(const std::string &node_name, int rows, int cols) {
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat(rows, cols);
    int count = 0;
    for (YAML::const_iterator it = node_[node_name].begin(); it != node_[node_name].end(); ++it) {
      for (size_t j = 0; j < it->size(); ++j) {
        mat(count) = (*it)[j].as<float>();
        count++;
      }
    }
    return mat;
  }

  void WriteToYaml(const std::string &node_name, const float &value) 
  {
    node_[node_name] = value;
  }

  bool SaveToFile(const std::string &output_file = "") 
  {
    std::string file = output_file.empty() ? yaml_file_ : output_file;
    try {
        std::ofstream fout(file);
        fout << node_;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Error saving YAML file: " << e.what() << std::endl;
        return false;
    }
  }

 private:
  std::string yaml_file_;
  YAML::Node node_;
};

#endif  // YAML_PARSER_
