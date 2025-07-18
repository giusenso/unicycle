#pragma once
#include <iostream>

#include "../utils/utils.hpp"

class UnicycleModel {

public:
  struct Input {
    double wr;
    double wl;
    Input() : wr(0.0), wl(0.0) {}
    Input(double wr_, double wl_) : wr(wr_), wl(wl_) {}
    friend std::ostream &operator<<(std::ostream &os, const Input &input);
  };
  inline static const Input UnfeasibleInput = Input(std::nan(""), std::nan(""));

  struct HighLevelInput {
    double v;
    double yawRate;
    HighLevelInput() : v(0.0), yawRate(0.0) {}
    HighLevelInput(double v_, double yawRate_) : v(v_), yawRate(yawRate_) {}
    friend std::ostream &operator<<(std::ostream &os, const Input &input);
  };

  struct State {
    double x;
    double y;
    double yaw;

    State() : x(0), y(0), yaw(0) {}
    State(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}

    bool operator==(const State &other) const {
      return std::abs(x - other.x) < 1e-9 && std::abs(y - other.y) < 1e-9 &&
             abs(angularDiff(yaw, other.yaw)) < 1e-9;
    }

    friend std::ostream &operator<<(std::ostream &os, const State &state);
  };
  inline static const State UninitializedState =
      State(std::nan(""), std::nan(""), std::nan(""));

  using Output = State;

  UnicycleModel(double l, double r, double wheelsRotSpeedMax, double dt,
                const State &state = UninitializedState);

  State state() const { return _state; }
  double samplingTime() const { return _dt; }
  double maxWheelRotSpeed() const { return _wheelsRotSpeedMax; }
  double wheelBase() const { return _l; }
  double wheelRadius() const { return _r; }

  /**
   * @brief Checks if the given input is feasible for the unicycle model.
   *
   * @param input The wheel speeds input (wr, wl) [rad/s].
   * @return true if both wheel speeds are within allowed limits.
   * @return false otherwise.
   */
  bool isInputFeasible(const Input &input) const;

  /**
   * @brief Performs forward kinematics using the given wheel speeds input.
   *
   * @param input The wheel speeds input (wr, wl) [rad/s].
   * @return true if the state was updated successfully.
   * @return false if the input is not feasible.
   */
  bool forwardKin(const Input &input);

  /**
   * @brief Computes the wheel speeds required to achieve the given linear and
   * angular velocities.
   *
   * @param v Desired linear velocity [m/s].
   * @param input High level input (v [m/s], yawRate [rad/s]).
   * @return Input The corresponding wheel speeds (wr, wl) [rad/s].
   */
  Input inverseKin(const HighLevelInput &input);

protected:
  double _l;                 // wheelbase [m]
  double _r;                 // wheel radius [m]
  double _wheelsRotSpeedMax; // max wheel rotational speed [rad/s]
  double _dt;                // sampling time [s]
  State _state;
};

class Unicycle : private UnicycleModel {
public:
  using Input = UnicycleModel::HighLevelInput;
  using UnicycleModel::Output;
  using UnicycleModel::State;

  Unicycle(double l, double r, double wheelsRotSpeedMax, double dt,
           const State &state = UninitializedState);

  using UnicycleModel::maxWheelRotSpeed;
  using UnicycleModel::samplingTime;
  using UnicycleModel::state;
  using UnicycleModel::wheelBase;
  using UnicycleModel::wheelRadius;

  /**
   * @brief Applies a high-level input (linear and angular velocity) to the
   * unicycle.
   *
   * This function computes the corresponding wheel speeds using inverse
   * kinematics, then updates the state using forward kinematics if the input is
   * feasible.
   *
   * @param input High-level input containing desired linear velocity [m/s] and
   * angular velocity [rad/s].
   * @return true if the input was feasible and the state was updated.
   * @return false if the input was not feasible and the state was not updated.
   */
  bool applyInput(const Input &input);

protected:
  using UnicycleModel::forwardKin;
  using UnicycleModel::inverseKin;
};