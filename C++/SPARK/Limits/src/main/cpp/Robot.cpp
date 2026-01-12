// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

Robot::Robot() {
  /*
   * Create a new SPARK MAX configuration object. This will store the
   * configuration parameters for the SPARK MAX that we will set below.
   */
  SparkMaxConfig motorConfig;

  // Set the idle mode to brake to stop immediately when reaching a limit
  motorConfig.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);

  // Enable limit switches to stop the motor when they are closed
  motorConfig.limitSwitch
      .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
      .ForwardLimitSwitchTriggerBehavior(LimitSwitchConfig::Behavior::kStopMovingMotor)
      .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchTriggerBehavior(LimitSwitchConfig::Behavior::kStopMovingMotor);

  // Set the soft limits to stop the motor at -50 and 50 rotations
  motorConfig.softLimit.ForwardSoftLimit(50)
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(-50)
      .ReverseSoftLimitEnabled(true);

  /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
  m_motor.Configure(motorConfig, rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_encoder.SetPosition(0);

  // Initialize dashboard values
  frc::SmartDashboard::SetDefaultBoolean("Direction", true);
}

void Robot::RobotPeriodic() {
  // Display data from SPARK onto the dashboard
  frc::SmartDashboard::PutBoolean("Forward Limit Reached",
                                  m_forwardLimitSwitch.Get());
  frc::SmartDashboard::PutBoolean("Reverse Limit Reached",
                                  m_reverseLimitSwitch.Get());
  frc::SmartDashboard::PutNumber("Applied Output", m_motor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Position", m_encoder.GetPosition());
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // Set the motor setpoint based on the direction from the dashboard
  if (frc::SmartDashboard::GetBoolean("Direction", true)) {
    m_motor.Set(0.2);
  } else {
    m_motor.Set(-0.2);
  }
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
