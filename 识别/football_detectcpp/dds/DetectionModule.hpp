/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: DetectionModule.idl
  Source: DetectionModule.hpp
  Cyclone DDS: v0.10.5

*****************************************************************/
#ifndef DDSCXX_DETECTIONMODULE_HPP
#define DDSCXX_DETECTIONMODULE_HPP

#include <array>
#include <vector>
#include <string>

namespace DetectionModule
{
class DetectionResult
{
private:
 std::string class_id_;
 std::string class_name_;
 std::array<float, 4> box_ = { };
 float score_ = 0.0f;
 std::array<float, 3> xyz_ = { };
 std::array<float, 2> offset_ = { };
 std::array<float, 2> offset_fov_ = { };

public:
  DetectionResult() = default;

  explicit DetectionResult(
    const std::string& class_id,
    const std::string& class_name,
    const std::array<float, 4>& box,
    float score,
    const std::array<float, 3>& xyz,
    const std::array<float, 2>& offset,
    const std::array<float, 2>& offset_fov) :
    class_id_(class_id),
    class_name_(class_name),
    box_(box),
    score_(score),
    xyz_(xyz),
    offset_(offset),
    offset_fov_(offset_fov) { }

  const std::string& class_id() const { return this->class_id_; }
  std::string& class_id() { return this->class_id_; }
  void class_id(const std::string& _val_) { this->class_id_ = _val_; }
  void class_id(std::string&& _val_) { this->class_id_ = _val_; }
  const std::string& class_name() const { return this->class_name_; }
  std::string& class_name() { return this->class_name_; }
  void class_name(const std::string& _val_) { this->class_name_ = _val_; }
  void class_name(std::string&& _val_) { this->class_name_ = _val_; }
  const std::array<float, 4>& box() const { return this->box_; }
  std::array<float, 4>& box() { return this->box_; }
  void box(const std::array<float, 4>& _val_) { this->box_ = _val_; }
  void box(std::array<float, 4>&& _val_) { this->box_ = _val_; }
  float score() const { return this->score_; }
  float& score() { return this->score_; }
  void score(float _val_) { this->score_ = _val_; }
  const std::array<float, 3>& xyz() const { return this->xyz_; }
  std::array<float, 3>& xyz() { return this->xyz_; }
  void xyz(const std::array<float, 3>& _val_) { this->xyz_ = _val_; }
  void xyz(std::array<float, 3>&& _val_) { this->xyz_ = _val_; }
  const std::array<float, 2>& offset() const { return this->offset_; }
  std::array<float, 2>& offset() { return this->offset_; }
  void offset(const std::array<float, 2>& _val_) { this->offset_ = _val_; }
  void offset(std::array<float, 2>&& _val_) { this->offset_ = _val_; }
  const std::array<float, 2>& offset_fov() const { return this->offset_fov_; }
  std::array<float, 2>& offset_fov() { return this->offset_fov_; }
  void offset_fov(const std::array<float, 2>& _val_) { this->offset_fov_ = _val_; }
  void offset_fov(std::array<float, 2>&& _val_) { this->offset_fov_ = _val_; }

  bool operator==(const DetectionResult& _other) const
  {
    (void) _other;
    return class_id_ == _other.class_id_ &&
      class_name_ == _other.class_name_ &&
      box_ == _other.box_ &&
      score_ == _other.score_ &&
      xyz_ == _other.xyz_ &&
      offset_ == _other.offset_ &&
      offset_fov_ == _other.offset_fov_;
  }

  bool operator!=(const DetectionResult& _other) const
  {
    return !(*this == _other);
  }

};

class DetectionResults
{
private:
 std::vector<::DetectionModule::DetectionResult> results_;

public:
  DetectionResults() = default;

  explicit DetectionResults(
    const std::vector<::DetectionModule::DetectionResult>& results) :
    results_(results) { }

  const std::vector<::DetectionModule::DetectionResult>& results() const { return this->results_; }
  std::vector<::DetectionModule::DetectionResult>& results() { return this->results_; }
  void results(const std::vector<::DetectionModule::DetectionResult>& _val_) { this->results_ = _val_; }
  void results(std::vector<::DetectionModule::DetectionResult>&& _val_) { this->results_ = _val_; }

  bool operator==(const DetectionResults& _other) const
  {
    (void) _other;
    return results_ == _other.results_;
  }

  bool operator!=(const DetectionResults& _other) const
  {
    return !(*this == _other);
  }

};

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::DetectionModule::DetectionResult>::getTypeName()
{
  return "DetectionModule::DetectionResult";
}

template <> constexpr bool TopicTraits<::DetectionModule::DetectionResult>::isSelfContained()
{
  return false;
}

template <> constexpr bool TopicTraits<::DetectionModule::DetectionResult>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::DetectionModule::DetectionResult>::type_map_blob_sz() { return 574; }
template<> constexpr unsigned int TopicTraits<::DetectionModule::DetectionResult>::type_info_blob_sz() { return 100; }
template<> inline const uint8_t * TopicTraits<::DetectionModule::DetectionResult>::type_map_blob() {
  static const uint8_t blob[] = {
 0xca,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf1,  0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2, 
 0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf,  0x97,  0x00,  0xb2,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0xa2,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00, 
 0x0c,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x30,  0x1e,  0x3c,  0x17, 
 0x0c,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x38,  0xc9,  0xbd,  0x9e, 
 0x16,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x04,  0x09,  0x34,  0xbe,  0x95,  0x8a,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x03,  0x00,  0x00,  0x00,  0x01,  0x00,  0x09,  0xca,  0x1c,  0xd3,  0xc3,  0x00,  0x16,  0x00,  0x00,  0x00, 
 0x04,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x03,  0x09,  0xd1,  0x6f,  0xb3,  0x6f,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x7a,  0x86, 
 0xc1,  0x57,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x59,  0x02,  0x43,  0x3d,  0x00,  0x00, 
 0x41,  0x01,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83, 
 0xc8,  0xad,  0x28,  0xcb,  0x14,  0x20,  0xd7,  0x00,  0x29,  0x01,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x29,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00,  0x44,  0x65,  0x74,  0x65, 
 0x63,  0x74,  0x69,  0x6f,  0x6e,  0x4d,  0x6f,  0x64,  0x75,  0x6c,  0x65,  0x3a,  0x3a,  0x44,  0x65,  0x74, 
 0x65,  0x63,  0x74,  0x69,  0x6f,  0x6e,  0x52,  0x65,  0x73,  0x75,  0x6c,  0x74,  0x00,  0x00,  0x00,  0x00, 
 0xf1,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00,  0x17,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x70,  0x00,  0x09,  0x00,  0x00,  0x00,  0x63,  0x6c,  0x61,  0x73,  0x73,  0x5f,  0x69,  0x64, 
 0x00,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00, 
 0x0b,  0x00,  0x00,  0x00,  0x63,  0x6c,  0x61,  0x73,  0x73,  0x5f,  0x6e,  0x61,  0x6d,  0x65,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x1e,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x04,  0x09,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x62,  0x6f,  0x78,  0x00,  0x00,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x09,  0x00,  0x06,  0x00,  0x00,  0x00,  0x73,  0x63,  0x6f,  0x72,  0x65,  0x00,  0x00,  0x00, 
 0x1e,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x03,  0x09,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x78,  0x79,  0x7a,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00, 
 0x6f,  0x66,  0x66,  0x73,  0x65,  0x74,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00, 
 0x06,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x02,  0x09,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x6f,  0x66,  0x66,  0x73,  0x65,  0x74,  0x5f,  0x66, 
 0x6f,  0x76,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x22,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb,  0x14,  0x20,  0xd7,  0xf1, 
 0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2,  0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf,  0x97, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::DetectionModule::DetectionResult>::type_info_blob() {
  static const uint8_t blob[] = {
 0x60,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2,  0xca,  0x8a,  0x8d,  0xb9, 
 0xc8,  0xcf,  0x97,  0x00,  0xb6,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb, 
 0x14,  0x20,  0xd7,  0x00,  0x2d,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

template <> constexpr const char* TopicTraits<::DetectionModule::DetectionResults>::getTypeName()
{
  return "DetectionModule::DetectionResults";
}

template <> constexpr allowable_encodings_t TopicTraits<::DetectionModule::DetectionResults>::allowableEncodings()
{
  return 0xFFFFFFFEu;
}

template <> constexpr bool TopicTraits<::DetectionModule::DetectionResults>::isSelfContained()
{
  return false;
}

template <> constexpr bool TopicTraits<::DetectionModule::DetectionResults>::isKeyless()
{
  return true;
}

template <> constexpr extensibility TopicTraits<::DetectionModule::DetectionResults>::getExtensibility()
{
  return extensibility::ext_appendable;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::DetectionModule::DetectionResults>::type_map_blob_sz() { return 808; }
template<> constexpr unsigned int TopicTraits<::DetectionModule::DetectionResults>::type_info_blob_sz() { return 148; }
template<> inline const uint8_t * TopicTraits<::DetectionModule::DetectionResults>::type_map_blob() {
  static const uint8_t blob[] = {
 0x16,  0x01,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0xf1,  0xf2,  0x0c,  0x73,  0x8d,  0x33,  0x83,  0xc7, 
 0xc8,  0x22,  0x99,  0xb2,  0x6a,  0xa0,  0x9e,  0x00,  0x36,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x02,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x26,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x1e,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x80,  0xf1,  0x01,  0x00,  0x00,  0xf1, 
 0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2,  0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf,  0x97,  0x53,  0xe6, 
 0x13,  0x36,  0xf1,  0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2,  0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf, 
 0x97,  0x00,  0x00,  0x00,  0xb2,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0xa2,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00,  0x0c,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x30,  0x1e,  0x3c,  0x17,  0x0c,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x38,  0xc9,  0xbd,  0x9e,  0x16,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x04,  0x09,  0x34,  0xbe,  0x95,  0x8a,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x09,  0xca,  0x1c,  0xd3,  0xc3,  0x00,  0x16,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x03,  0x09,  0xd1,  0x6f, 
 0xb3,  0x6f,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x7a,  0x86,  0xc1,  0x57,  0x00,  0x00, 
 0x16,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x59,  0x02,  0x43,  0x3d,  0x00,  0x00,  0xc1,  0x01,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0xf2,  0x62,  0xa5,  0x4e,  0xb7,  0x11,  0xf6,  0xfb,  0x92,  0x61,  0xce,  0x05, 
 0x0c,  0xa0,  0xaf,  0x00,  0x6a,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x02,  0x00,  0x2a,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x22,  0x00,  0x00,  0x00,  0x44,  0x65,  0x74,  0x65,  0x63,  0x74,  0x69,  0x6f, 
 0x6e,  0x4d,  0x6f,  0x64,  0x75,  0x6c,  0x65,  0x3a,  0x3a,  0x44,  0x65,  0x74,  0x65,  0x63,  0x74,  0x69, 
 0x6f,  0x6e,  0x52,  0x65,  0x73,  0x75,  0x6c,  0x74,  0x73,  0x00,  0x00,  0x00,  0x32,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x2a,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x80,  0xf2, 
 0x01,  0x00,  0x00,  0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb,  0x14, 
 0x20,  0xd7,  0x00,  0x00,  0x08,  0x00,  0x00,  0x00,  0x72,  0x65,  0x73,  0x75,  0x6c,  0x74,  0x73,  0x00, 
 0x00,  0x00,  0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb,  0x14,  0x20, 
 0xd7,  0x00,  0x00,  0x00,  0x29,  0x01,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x29,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00,  0x44,  0x65,  0x74,  0x65,  0x63,  0x74,  0x69,  0x6f, 
 0x6e,  0x4d,  0x6f,  0x64,  0x75,  0x6c,  0x65,  0x3a,  0x3a,  0x44,  0x65,  0x74,  0x65,  0x63,  0x74,  0x69, 
 0x6f,  0x6e,  0x52,  0x65,  0x73,  0x75,  0x6c,  0x74,  0x00,  0x00,  0x00,  0x00,  0xf1,  0x00,  0x00,  0x00, 
 0x07,  0x00,  0x00,  0x00,  0x17,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00, 
 0x09,  0x00,  0x00,  0x00,  0x63,  0x6c,  0x61,  0x73,  0x73,  0x5f,  0x69,  0x64,  0x00,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x63,  0x6c,  0x61,  0x73,  0x73,  0x5f,  0x6e,  0x61,  0x6d,  0x65,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x1e,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x04,  0x09,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x62,  0x6f,  0x78,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x01,  0x00,  0x09,  0x00, 
 0x06,  0x00,  0x00,  0x00,  0x73,  0x63,  0x6f,  0x72,  0x65,  0x00,  0x00,  0x00,  0x1e,  0x00,  0x00,  0x00, 
 0x04,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x03,  0x09,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x78,  0x79,  0x7a,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x21,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00,  0x6f,  0x66,  0x66,  0x73, 
 0x65,  0x74,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x02,  0x09,  0x00,  0x00, 
 0x0b,  0x00,  0x00,  0x00,  0x6f,  0x66,  0x66,  0x73,  0x65,  0x74,  0x5f,  0x66,  0x6f,  0x76,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x40,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0xf2,  0x62,  0xa5,  0x4e, 
 0xb7,  0x11,  0xf6,  0xfb,  0x92,  0x61,  0xce,  0x05,  0x0c,  0xa0,  0xaf,  0xf1,  0xf2,  0x0c,  0x73,  0x8d, 
 0x33,  0x83,  0xc7,  0xc8,  0x22,  0x99,  0xb2,  0x6a,  0xa0,  0x9e,  0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40, 
 0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb,  0x14,  0x20,  0xd7,  0xf1,  0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24, 
 0xd2,  0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf,  0x97, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::DetectionModule::DetectionResults>::type_info_blob() {
  static const uint8_t blob[] = {
 0x90,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x40,  0x00,  0x00,  0x00,  0x3c,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0xf2,  0x0c,  0x73,  0x8d,  0x33,  0x83,  0xc7,  0xc8,  0x22,  0x99,  0xb2, 
 0x6a,  0xa0,  0x9e,  0x00,  0x3a,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x1c,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf1,  0x79,  0xbc,  0x4c,  0x32,  0x6b,  0x24,  0xd2, 
 0xca,  0x8a,  0x8d,  0xb9,  0xc8,  0xcf,  0x97,  0x00,  0xb6,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40, 
 0x40,  0x00,  0x00,  0x00,  0x3c,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf2,  0x62,  0xa5,  0x4e, 
 0xb7,  0x11,  0xf6,  0xfb,  0x92,  0x61,  0xce,  0x05,  0x0c,  0xa0,  0xaf,  0x00,  0x6e,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x1c,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf2,  0x7d,  0x9e,  0x90,  0xe6,  0x40,  0xc8,  0x83,  0xc8,  0xad,  0x28,  0xcb,  0x14,  0x20,  0xd7,  0x00, 
 0x2d,  0x01,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::DetectionModule::DetectionResult>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::DetectionModule::DetectionResult>::getTypeName();
    }
};

template <>
struct topic_type_name<::DetectionModule::DetectionResults>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::DetectionModule::DetectionResults>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::DetectionModule::DetectionResult)
REGISTER_TOPIC_TYPE(::DetectionModule::DetectionResults)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::DetectionModule::DetectionResult>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::DetectionModule::DetectionResult& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write_string(streamer, instance.class_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!write_string(streamer, instance.class_name(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.box()[0], instance.box().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.score()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.xyz()[0], instance.xyz().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.offset()[0], instance.offset().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.offset_fov()[0], instance.offset_fov().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::DetectionModule::DetectionResult& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResult>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::DetectionModule::DetectionResult& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read_string(streamer, instance.class_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!read_string(streamer, instance.class_name(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.box()[0], instance.box().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.score()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.xyz()[0], instance.xyz().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.offset()[0], instance.offset().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.offset_fov()[0], instance.offset_fov().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::DetectionModule::DetectionResult& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResult>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::DetectionModule::DetectionResult& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move_string(streamer, instance.class_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!move_string(streamer, instance.class_name(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.box()[0], instance.box().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.score()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.xyz()[0], instance.xyz().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.offset()[0], instance.offset().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.offset_fov()[0], instance.offset_fov().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::DetectionModule::DetectionResult& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResult>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::DetectionModule::DetectionResult& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max_string(streamer, instance.class_id(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!max_string(streamer, instance.class_name(), 0))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.box()[0], instance.box().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.score()))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.xyz()[0], instance.xyz().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.offset()[0], instance.offset().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 6:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.offset_fov()[0], instance.offset_fov().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::DetectionModule::DetectionResult& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResult>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

template<>
propvec &get_type_props<::DetectionModule::DetectionResults>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::DetectionModule::DetectionResults& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(false, false))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.results().size());
      if (!write(streamer, se_1))
        return false;
      for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
      if (!write(streamer, instance.results()[i_1], prop))
        return false;
      }  //i_1
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::DetectionModule::DetectionResults& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResults>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::DetectionModule::DetectionResults& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(false, false))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.results().size());
      if (!read(streamer, se_1))
        return false;
      instance.results().resize(se_1);
      for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
      if (!read(streamer, instance.results()[i_1], prop))
        return false;
      }  //i_1
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::DetectionModule::DetectionResults& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResults>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::DetectionModule::DetectionResults& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(false, false))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.results().size());
      if (!move(streamer, se_1))
        return false;
      for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
      if (!move(streamer, instance.results()[i_1], prop))
        return false;
      }  //i_1
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::DetectionModule::DetectionResults& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResults>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::DetectionModule::DetectionResults& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(false, false))
        return false;
      {
      uint32_t se_1 = 0;
      if (!max(streamer, se_1))
        return false;
      for (uint32_t i_1 = 0; i_1 < se_1; i_1++) {
      if (!max(streamer, instance.results()[i_1], prop))
        return false;
      }  //i_1
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
      streamer.position(SIZE_MAX);
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::DetectionModule::DetectionResults& instance, bool as_key) {
  auto &props = get_type_props<::DetectionModule::DetectionResults>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_DETECTIONMODULE_HPP
