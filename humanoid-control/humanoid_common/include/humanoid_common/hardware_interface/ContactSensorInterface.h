#pragma once
#include <cassert>
#include <humanoid_common/hardware_interface/HardwareInterfaceCompat.h>

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
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. isContact pointer is null.");
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
  : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace humanoid
}  // namespace ocs2