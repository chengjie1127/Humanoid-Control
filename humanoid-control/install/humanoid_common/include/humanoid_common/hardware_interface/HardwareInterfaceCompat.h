#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface {

class HardwareInterfaceException : public std::runtime_error {
 public:
  explicit HardwareInterfaceException(const std::string& message) : std::runtime_error(message) {}
};

class JointStateHandle {
 public:
  JointStateHandle() = default;

  JointStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff)
      : name_(name), pos_(pos), vel_(vel), eff_(eff) {
    if (pos_ == nullptr || vel_ == nullptr || eff_ == nullptr) {
      throw HardwareInterfaceException("Cannot create handle '" + name_ + "'. Joint state data pointer is null.");
    }
  }

  const std::string& getName() const { return name_; }

  double getPosition() const {
    if (pos_ == nullptr) {
      throw HardwareInterfaceException("Cannot get position for handle '" + name_ + "'. Position pointer is null.");
    }
    return *pos_;
  }

  double getVelocity() const {
    if (vel_ == nullptr) {
      throw HardwareInterfaceException("Cannot get velocity for handle '" + name_ + "'. Velocity pointer is null.");
    }
    return *vel_;
  }

  double getEffort() const {
    if (eff_ == nullptr) {
      throw HardwareInterfaceException("Cannot get effort for handle '" + name_ + "'. Effort pointer is null.");
    }
    return *eff_;
  }

 private:
  std::string name_;
  const double* pos_ = nullptr;
  const double* vel_ = nullptr;
  const double* eff_ = nullptr;
};

struct ClaimResources {};
struct DontClaimResources {};

template <typename HandleType, typename ClaimPolicy>
class HardwareResourceManager {
 public:
  void registerHandle(const HandleType& handle) { handles_[handle.getName()] = handle; }

  HandleType& getHandle(const std::string& name) {
    auto it = handles_.find(name);
    if (it == handles_.end()) {
      throw HardwareInterfaceException("Could not find resource '" + name + "'.");
    }
    return it->second;
  }

  const HandleType& getHandle(const std::string& name) const {
    auto it = handles_.find(name);
    if (it == handles_.end()) {
      throw HardwareInterfaceException("Could not find resource '" + name + "'.");
    }
    return it->second;
  }

  std::vector<std::string> getNames() const {
    std::vector<std::string> names;
    names.reserve(handles_.size());
    for (const auto& pair : handles_) {
      names.push_back(pair.first);
    }
    return names;
  }

 protected:
  std::unordered_map<std::string, HandleType> handles_;
};

}  // namespace hardware_interface
