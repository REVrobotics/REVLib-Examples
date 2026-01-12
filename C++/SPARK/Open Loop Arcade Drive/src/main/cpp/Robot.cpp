// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

Robot::Robot() {
  /*
   * Create new SPARK MAX configuration objects. These will store the
   * configuration parameters for the SPARK MAXes that we will set below.
   */
  SparkMaxConfig globalConfig;
  SparkMaxConfig rightLeaderConfig;
  SparkMaxConfig leftFollowerConfig;
  SparkMaxConfig rightFollowerConfig;

  /*
   * Set parameters that will apply to all SPARKs. We will also use this as
   * the left leader config.
   */
  globalConfig.SmartCurrentLimit(50).SetIdleMode(
      SparkMaxConfig::IdleMode::kBrake);

  // Apply the global config and invert since it is on the opposite side
  rightLeaderConfig.Apply(globalConfig).Inverted(true);

  // Apply the global config and set the leader SPARK for follower mode
  leftFollowerConfig.Apply(globalConfig).Follow(m_leftLeader);

  // Apply the global config and set the leader SPARK for follower mode
  rightFollowerConfig.Apply(globalConfig).Follow(m_rightLeader);

  /*
   * Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
  m_leftLeader.Configure(globalConfig,
                         rev::ResetMode::kResetSafeParameters,
                         rev::PersistMode::kPersistParameters);
  m_leftFollower.Configure(leftFollowerConfig,
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
  m_rightLeader.Configure(rightLeaderConfig,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
  m_rightFollower.Configure(rightFollowerConfig,
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
}
void Robot::RobotPeriodic() {
  // Display the applied output of the left and right side onto the dashboard
  frc::SmartDashboard::PutNumber("Left Out", m_leftLeader.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Right Out", m_rightLeader.GetAppliedOutput());
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /**
   * Get forward and rotation values from the joystick. Invert the joystick's
   * Y value because its forward direction is negative.
   */
  double forward = -joystick.GetLeftY();
  double rotation = joystick.GetRightX();

  /*
   * Apply values to left and right side. We will only need to set the leaders
   * since the other motors are in follower mode.
   */
  m_leftLeader.Set(forward + rotation);
  m_rightLeader.Set(forward - rotation);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
