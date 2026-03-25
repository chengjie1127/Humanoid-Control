#pragma once

#include <string>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_interface/common/Types.h"

namespace ocs2 {
namespace humanoid {

class FootSafetyConstraint final : public StateConstraint {
 public:
  struct Config {
    scalar_t minimumLateralSeparation = 0.06;
    scalar_t minimumHeight = 0.005;
    scalar_t maximumHeight = 0.25;
  };

  FootSafetyConstraint(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, std::vector<std::string> contactNames,
                       Config config);
  ~FootSafetyConstraint() override = default;

  FootSafetyConstraint* clone() const override { return new FootSafetyConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 10; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  FootSafetyConstraint(const FootSafetyConstraint& other) = default;

  vector_t evaluateConstraint(const vector_t& state) const;
  feet_array_t<vector3_t> computeFootPositions(const vector_t& state) const;

  mutable PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::vector<std::string> contactNames_;
  Config config_;
};

}  // namespace humanoid
}  // namespace ocs2
