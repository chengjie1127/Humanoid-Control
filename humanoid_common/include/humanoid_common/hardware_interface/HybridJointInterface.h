//
// Created by qiayuan on 2021/11/5.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once
#include <cassert>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace ocs2 {
namespace humanoid
{
class JointStateHandle
{
public:
  JointStateHandle() = default;

  JointStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff)
    : name_(name), pos_(pos), vel_(vel), eff_(eff)
  {
    if (pos_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + name + "'. Position pointer is null.");
    }
    if (vel_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + name + "'. Velocity pointer is null.");
    }
    if (eff_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + name + "'. Effort pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }

  double getPosition() const
  {
    assert(pos_);
    return *pos_;
  }

  double getVelocity() const
  {
    assert(vel_);
    return *vel_;
  }

  double getEffort() const
  {
    assert(eff_);
    return *eff_;
  }

private:
  std::string name_;
  const double* pos_ = { nullptr };
  const double* vel_ = { nullptr };
  const double* eff_ = { nullptr };
};

class HybridJointHandle : public JointStateHandle
{
public:
  HybridJointHandle() = default;

  HybridJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff)
    : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff)
  {
    if (posDes_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + js.getName() + "'. Position desired data pointer is null.");
    }
    if (velDes_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + js.getName() + "'. Velocity desired data pointer is null.");
    }
    if (kp_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
    }
    if (kd_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
    }
    if (ff_ == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + js.getName() + "'. Feedforward data pointer is null.");
    }
  }
  void setPositionDesired(double cmd)
  {
    assert(posDes_);
    *posDes_ = cmd;
  }
  void setVelocityDesired(double cmd)
  {
    assert(velDes_);
    *velDes_ = cmd;
  }
  void setKp(double cmd)
  {
    assert(kp_);
    *kp_ = cmd;
  }
  void setKd(double cmd)
  {
    assert(kd_);
    *kd_ = cmd;
  }
  void setFeedforward(double cmd)
  {
    assert(ff_);
    *ff_ = cmd;
  }
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff)
  {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }
  double getPositionDesired()
  {
    assert(posDes_);
    return *posDes_;
  }
  double getVelocityDesired()
  {
    assert(velDes_);
    return *velDes_;
  }
  double getKp()
  {
    assert(kp_);
    return *kp_;
  }
  double getKd()
  {
    assert(kd_);
    return *kd_;
  }
  double getFeedforward()
  {
    assert(ff_);
    return *ff_;
  }

private:
  double* posDes_ = { nullptr };
  double* velDes_ = { nullptr };
  double* kp_ = { nullptr };
  double* kd_ = { nullptr };
  double* ff_ = { nullptr };
};

class HybridJointInterface
{
public:
  void registerHandle(const HybridJointHandle& handle)
  {
    const auto [_, inserted] = handles_.emplace(handle.getName(), handle);
    if (!inserted)
    {
      throw std::runtime_error("Cannot register handle '" + handle.getName() + "'. Name already exists.");
    }
  }

  HybridJointHandle& getHandle(const std::string& name)
  {
    auto it = handles_.find(name);
    if (it == handles_.end())
    {
      throw std::runtime_error("Cannot get handle '" + name + "'. Name does not exist.");
    }
    return it->second;
  }

  const HybridJointHandle& getHandle(const std::string& name) const
  {
    auto it = handles_.find(name);
    if (it == handles_.end())
    {
      throw std::runtime_error("Cannot get handle '" + name + "'. Name does not exist.");
    }
    return it->second;
  }

  std::vector<std::string> getNames() const
  {
    std::vector<std::string> names;
    names.reserve(handles_.size());
    for (const auto& kv : handles_)
    {
      names.push_back(kv.first);
    }
    return names;
  }

private:
  std::unordered_map<std::string, HybridJointHandle> handles_;
};

}  // namespace humanoid
}  // namespace ocs2