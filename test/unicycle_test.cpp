#include "unicycle.hpp"
#include <gtest/gtest.h>

#include <iostream>

TEST(UnicycleModelTest, forwardKin) {
  // Test forward and inverse kinematics on straight line motions
  {
    const double wheelBase = 0.5;          // m
    const double wheelRadius = 0.25;       // m
    const double wheelsRotSpeedMax = 10.0; // rad/s
    const double dt = 0.02;                // s

    // Maximum velocity based on wheel radius and maximum wheel rotation speed
    const double vMax = wheelsRotSpeedMax * wheelRadius; // m/s

    const std::vector<double> velocities = {
        0.0, 1e-5, 1e-2, 0.5, 1.0, 0.999 * vMax, 1.001 * vMax}; // m/s
    const double yawRate = 0.0;                                 // rad
    const double duration = 3.0;                                // s

    for (const double v : velocities) {
      UnicycleModel unicycle(wheelBase, wheelRadius, wheelsRotSpeedMax, dt,
                             UnicycleModel::State(0., 0., M_PI_2));
      UnicycleModel::State initialState = unicycle.state();

      double t = 0.0;
      for (; t < duration; t += unicycle.samplingTime()) {
        // INVERSE KINEMATICS
        UnicycleModel::Input input =
            unicycle.inverseKin(UnicycleModel::HighLevelInput(v, yawRate));

        // FORWARD KINEMATICS
        bool succ = unicycle.forwardKin(input);
        // If the input is not feasible, the state should not change
        // Else, we expect it to change
        EXPECT_EQ(succ, unicycle.isInputFeasible(input));
      }

      EXPECT_NEAR(unicycle.state().x, initialState.x, 1e-6);
      // If the velocity is violated, the y position should not change
      // Else, we expect it to change
      EXPECT_NEAR(unicycle.state().y,
                  initialState.y + (v > vMax ? 0 : v * duration), 1e-6);
      EXPECT_NEAR(unicycle.state().yaw, initialState.yaw, 1e-6);
    }
  }

  //  Test forward and inverse kinematics on pure rotation
  {
    const double wheelBase = 0.5;          // m
    const double wheelRadius = 0.25;       // m
    const double wheelsRotSpeedMax = 10.0; // rad/s
    const double dt = 0.02;                // s

    // Maximum rotational speed based on radius and maximum wheel rotation speed
    const double yawRateMax =
        (wheelRadius / wheelBase) * 2 * wheelsRotSpeedMax; // rad/s

    const std::vector<double> yawRates = {0.0,
                                          1e-5,
                                          1e-2,
                                          0.25,
                                          1.0,
                                          0.999 * yawRateMax,
                                          1.001 * yawRateMax}; // rad/s
    const double duration = 2.0;                               // s

    for (const double yawRate : yawRates) {
      UnicycleModel unicycle(wheelBase, wheelRadius, wheelsRotSpeedMax, dt,
                             UnicycleModel::State(1., 1., M_PI_2));
      UnicycleModel::State initialState = unicycle.state();

      double t = 0.0;
      for (; t < duration; t += unicycle.samplingTime()) {
        // INVERSE KINEMATICS
        UnicycleModel::Input input =
            unicycle.inverseKin(UnicycleModel::HighLevelInput(0., yawRate));

        // FORWARD KINEMATICS
        bool succ = unicycle.forwardKin(input);
        // If the input is not feasible, the state should not change
        // Else, we expect it to change
        EXPECT_EQ(succ, unicycle.isInputFeasible(input));
      }

      EXPECT_NEAR(unicycle.state().x, initialState.x, 1e-6);
      EXPECT_NEAR(unicycle.state().y, initialState.y, 1e-6);
      // If the yaw rate is not violated, the yaw should not change
      // Else, we expect it to change
      EXPECT_NEAR(unicycle.state().yaw,
                  angularSum(initialState.yaw,
                             (yawRate > yawRateMax ? 0.0 : yawRate * duration)),
                  1e-6);
    }
  }

  // Test forward and inverse kinematics on a circular path
  {
    const double wheelBase = 0.5;          // m
    const double wheelRadius = 0.25;       // m
    const double wheelsRotSpeedMax = 10.0; // rad/s
    const double dt = 0.02;                // s

    std::vector<double> velocities = {0.0, 0.1, 0.5, 2.0}; // m/s
    std::vector<double> yawRates = {0.1, 1.0, 0.5, 0.03};  // m/s

    for (unsigned i = 0; i < velocities.size(); ++i) {

      const double vDes = velocities.at(i);
      const double yawRateDes = yawRates.at(i);

      UnicycleModel unicycle(wheelBase, wheelRadius, wheelsRotSpeedMax, dt,
                             UnicycleModel::State(0., 0., 0.));
      UnicycleModel::State initialState = unicycle.state();
      UnicycleModel::State prev_state = initialState;

      double t = 0.0;
      for (;; t += unicycle.samplingTime()) {
        // INVERSE KINEMATICS
        UnicycleModel::Input input = unicycle.inverseKin(
            UnicycleModel::HighLevelInput(vDes, yawRateDes));

        prev_state = unicycle.state();

        // FORWARD KINEMATICS
        bool succ = unicycle.forwardKin(input);
        // If the input is not feasible, the state should not change
        // Else, we expect it to change
        EXPECT_EQ(succ,
                  unicycle.isInputFeasible(input)); // Ensure input feasibility
        EXPECT_NE(prev_state.yaw, unicycle.state().yaw); // Ensure state changes

        // When we have done a full circular trajectory, we should stop
        if (prev_state.yaw > unicycle.state().yaw) {
          break;
        }
      }

      // Are we in the proximity of the initial state?
      EXPECT_NEAR(unicycle.state().x, initialState.x,
                  1.1 * vDes * cos(unicycle.state().yaw) *
                      unicycle.samplingTime());
      EXPECT_NEAR(unicycle.state().y, initialState.y,
                  1.1 * vDes * sin(unicycle.state().yaw) *
                      unicycle.samplingTime());
      EXPECT_NEAR(unicycle.state().yaw, initialState.yaw,
                  1.1 * yawRateDes * unicycle.samplingTime());
    }
  }

  // Test invalid parameters
  {
    // Invalid wheel base
    EXPECT_THROW(
        {
          UnicycleModel model(0.0, 0.1, 1.0, 0.01,
                              UnicycleModel::State(0., 0., 0.));
        },
        std::invalid_argument);

    // Invalid wheel radius
    EXPECT_THROW(
        {
          UnicycleModel model(0.5, -0.2, 1.0, 0.01,
                              UnicycleModel::State(0., 0., 0.));
        },
        std::invalid_argument);

    // Invalid dt
    EXPECT_THROW(
        {
          UnicycleModel model(0.5, 0.1, 1.0, 0.0,
                              UnicycleModel::State(0., 0., 0.));
        },
        std::invalid_argument);
  }
}

TEST(UnicycleTest, applyInput) {
  // Mimic the circular path done for the UnicycleModel
  {
    const double wheelBase = 0.5;          // m
    const double wheelRadius = 0.25;       // m
    const double wheelsRotSpeedMax = 10.0; // rad/s
    const double dt = 0.02;                // s

    std::vector<double> velocities = {0.0, 0.1, 0.5, 2.0}; // m/s
    std::vector<double> yawRates = {0.1, 1.0, 0.5, 0.03};  // m/s

    for (unsigned i = 0; i < velocities.size(); ++i) {

      const double vDes = velocities.at(i);
      const double yawRateDes = yawRates.at(i);

      Unicycle unicycle(wheelBase, wheelRadius, wheelsRotSpeedMax, dt,
                        UnicycleModel::State(0., 0., 0.));
      Unicycle::State initialState = unicycle.state();
      Unicycle::State prev_state = initialState;

      double t = 0.0;
      for (;; t += unicycle.samplingTime()) {

        prev_state = unicycle.state();

        bool succ = unicycle.applyInput(Unicycle::Input(vDes, yawRateDes));
        EXPECT_EQ(
            succ,
            true); // Ensure input feasibility (we know this input is feasible)
        EXPECT_NE(prev_state.yaw, unicycle.state().yaw); // Ensure state changes

        // When we have done a full circular trajectory, we should stop
        if (prev_state.yaw > unicycle.state().yaw) {
          break;
        }
      }

      // Are we in the proximity of the initial state?
      EXPECT_NEAR(unicycle.state().x, initialState.x,
                  1.1 * vDes * cos(unicycle.state().yaw) *
                      unicycle.samplingTime());
      EXPECT_NEAR(unicycle.state().y, initialState.y,
                  1.1 * vDes * sin(unicycle.state().yaw) *
                      unicycle.samplingTime());
      EXPECT_NEAR(unicycle.state().yaw, initialState.yaw,
                  1.1 * yawRateDes * unicycle.samplingTime());
    }
  }
}