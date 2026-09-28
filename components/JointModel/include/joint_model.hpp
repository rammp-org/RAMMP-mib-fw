#pragma once

/// @file joint_model.hpp
/// @brief The mapping between a joint's engineering units and its actuator's travel.
///
/// An Actuator moves in encoder counts and exposes its calibrated travel as a fraction
/// from 0 (the low calibrated end) to 1 (the high end). A JointModel says what that
/// travel means for the joint: metres along a rail, degrees of a linkage, whatever the
/// mechanism does. It is pure arithmetic with no hardware dependency, so each model can
/// be unit-tested on the host against the leg's geometry.
///
/// One model class per kinematic family; each leg's ActuatorConfig names the instance it
/// gets. The real models (main legs, casters, carriages) are written once the base has
/// been measured. LinearJointModel covers any joint whose value is proportional to
/// actuator travel, and stands in until then.

namespace mib {

class JointModel {
public:
  virtual ~JointModel() = default;

  /// Joint value in engineering units -> fraction of the actuator's calibrated travel.
  /// Values outside the travel map outside [0, 1]; the Actuator clamps.
  virtual float to_fraction(float units) const = 0;

  /// Fraction of travel -> joint value in engineering units.
  virtual float to_units(float fraction) const = 0;

  /// Name of the unit, for logs and status ("m", "deg").
  virtual const char *unit() const = 0;
};

/// A joint whose value is proportional to actuator travel: two constants, what the joint
/// reads at each calibrated end.
class LinearJointModel : public JointModel {
public:
  constexpr LinearJointModel(float units_at_min, float units_at_max, const char *unit)
      : units_at_min_(units_at_min)
      , units_at_max_(units_at_max)
      , unit_(unit) {}

  float to_fraction(float units) const override {
    const float span = units_at_max_ - units_at_min_;
    return span == 0.0f ? 0.0f : (units - units_at_min_) / span;
  }

  float to_units(float fraction) const override {
    return units_at_min_ + fraction * (units_at_max_ - units_at_min_);
  }

  const char *unit() const override { return unit_; }

private:
  float units_at_min_;
  float units_at_max_;
  const char *unit_;
};

} // namespace mib
