#include "unicycle.hpp"

/*=== Unicycle Model ========================================================*/

UnicycleModel::UnicycleModel(double l, double r, double wheelsRotSpeedMax,
                             double dt, const State &state)
    : _l(l), _r(r), _wheelsRotSpeedMax(abs(wheelsRotSpeedMax)), _dt(dt),
      _state(state) {
  if (l <= 0) {
    throw std::invalid_argument("Parameter 'l' must be greater than 0.");
  }
  if (r <= 0) {
    throw std::invalid_argument("Parameter 'r' must be greater than 0.");
  }
  if (dt <= 0) {
    throw std::invalid_argument("Parameter 'dt' must be greater than 0.");
  }
}

std::ostream &operator<<(std::ostream &os, const UnicycleModel::Input &input) {
  os << "(wr=" << input.wr << ", wl=" << input.wl << ")";
  return os;
}

std::ostream &operator<<(std::ostream &os, const UnicycleModel::State &state) {
  os << "(x=" << state.x << ", y=" << state.y << ", yaw=" << state.yaw << ")";
  return os;
}

bool UnicycleModel::isInputFeasible(const Input &input) const {
  return std::abs(input.wr) <= _wheelsRotSpeedMax &&
         std::abs(input.wl) <= _wheelsRotSpeedMax;
}

bool UnicycleModel::forwardKin(const Input &input) {
  if (!isInputFeasible(input)) {
    return false;
  }
  _state.x =
      _state.x + (_r / 2 * (input.wr + input.wl) * cos(_state.yaw)) * _dt;
  _state.y =
      _state.y + (_r / 2 * (input.wr + input.wl) * sin(_state.yaw)) * _dt;
  _state.yaw = angularSum(_state.yaw, (_r / _l * (input.wr - input.wl)) * _dt);

  return true;
}

UnicycleModel::Input UnicycleModel::inverseKin(const HighLevelInput &input) {
  const double yaw_diff =
      angularDiff(_state.yaw + input.yawRate * _dt, _state.yaw);
  const double yaw_avg = angularSum(_state.yaw, yaw_diff / 2);

  double wr, wl;
  // Use x or y component based on the average orientation
  if (abs(cos(yaw_avg)) >= 0.5) {
    const double x_next = _state.x + input.v * cos(yaw_avg) * _dt;
    const double cos_yaw = cos(yaw_avg);
    wr = 0.5 * ((x_next - _state.x) / (_dt * (_r / 2) * cos_yaw) +
                (yaw_diff / (_dt * (_r / _l))));
    wl = 0.5 * ((x_next - _state.x) / (_dt * (_r / 2) * cos_yaw) -
                (yaw_diff / (_dt * (_r / _l))));
  } else {
    const double y_next = _state.y + input.v * sin(yaw_avg) * _dt;
    const double sin_yaw = sin(yaw_avg);
    wr = 0.5 * ((y_next - _state.y) / (_dt * (_r / 2) * sin_yaw) +
                (yaw_diff / (_dt * (_r / _l))));
    wl = 0.5 * ((y_next - _state.y) / (_dt * (_r / 2) * sin_yaw) -
                (yaw_diff / (_dt * (_r / _l))));
  }

  return Input(wr, wl);
}

/*=== Unicycle ==============================================================*/

Unicycle::Unicycle(double l, double r, double wheelsRotSpeedMax, double dt,
                   const State &state)
    : UnicycleModel(l, r, wheelsRotSpeedMax, dt, state) {}

std::ostream &operator<<(std::ostream &os, const Unicycle::Input &input) {
  os << "(v=" << input.v << ", yawRate=" << input.yawRate << ")";
  return os;
}

bool Unicycle::applyInput(const Input &input) {
  return UnicycleModel::forwardKin(UnicycleModel::inverseKin(input));
}
