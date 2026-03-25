#include "humanoid_interface/constraint/FootSafetyConstraint.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>

namespace ocs2 {
namespace humanoid {

FootSafetyConstraint::FootSafetyConstraint(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           std::vector<std::string> contactNames, Config config)
    : StateConstraint(ConstraintOrder::Linear),
      pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      contactNames_(std::move(contactNames)),
      config_(std::move(config)) {}

vector_t FootSafetyConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  return evaluateConstraint(state);
}

VectorFunctionLinearApproximation FootSafetyConstraint::getLinearApproximation(scalar_t /*time*/, const vector_t& state,
                                                                               const PreComputation& /*preComp*/) const {
  VectorFunctionLinearApproximation approximation;
  approximation.f = evaluateConstraint(state);
  approximation.dfdx = finiteDifferenceDerivative([this](const vector_t& x) { return evaluateConstraint(x); }, state, 1e-5, true);
  return approximation;
}

vector_t FootSafetyConstraint::evaluateConstraint(const vector_t& state) const {
  const auto footPositions = computeFootPositions(state);

  vector_t constraint(10);
  constraint << footPositions[0].y() - footPositions[1].y() - config_.minimumLateralSeparation,
      footPositions[2].y() - footPositions[3].y() - config_.minimumLateralSeparation,
      footPositions[0].z() - config_.minimumHeight, footPositions[1].z() - config_.minimumHeight,
      footPositions[2].z() - config_.minimumHeight, footPositions[3].z() - config_.minimumHeight,
      config_.maximumHeight - footPositions[0].z(), config_.maximumHeight - footPositions[1].z(),
      config_.maximumHeight - footPositions[2].z(), config_.maximumHeight - footPositions[3].z();
  return constraint;
}

feet_array_t<vector3_t> FootSafetyConstraint::computeFootPositions(const vector_t& state) const {
  const auto q = centroidal_model::getGeneralizedCoordinates(state, info_);
  auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  feet_array_t<vector3_t> positions;
  for (size_t i = 0; i < positions.size(); ++i) {
    const auto frameId = model.getBodyId(contactNames_[i]);
    positions[i] = data.oMf[frameId].translation();
  }
  return positions;
}

}  // namespace humanoid
}  // namespace ocs2
