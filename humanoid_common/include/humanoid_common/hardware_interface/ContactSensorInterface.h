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
class ContactSensorHandle
{
public:
  ContactSensorHandle() = default;

  ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact)
  {
    if (isContact == nullptr)
    {
      throw std::runtime_error("Cannot create handle '" + name + "'. isContact pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }

  bool isContact() const
  {
    assert(isContact_);
    return *isContact_;
  }

private:
  std::string name_;

  const bool* isContact_ = { nullptr };
};

class ContactSensorInterface
{
public:
  void registerHandle(const ContactSensorHandle& handle)
  {
    const auto [_, inserted] = handles_.emplace(handle.getName(), handle);
    if (!inserted)
    {
      throw std::runtime_error("Cannot register handle '" + handle.getName() + "'. Name already exists.");
    }
  }

  ContactSensorHandle& getHandle(const std::string& name)
  {
    auto it = handles_.find(name);
    if (it == handles_.end())
    {
      throw std::runtime_error("Cannot get handle '" + name + "'. Name does not exist.");
    }
    return it->second;
  }

  const ContactSensorHandle& getHandle(const std::string& name) const
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
  std::unordered_map<std::string, ContactSensorHandle> handles_;
};

}  // namespace humanoid
}  // namespace ocs2