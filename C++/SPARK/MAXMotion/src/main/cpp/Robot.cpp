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

  /*
   * Configure the encoder. For this specific example, we are using the
   * integrated encoder of the NEO, and we don't need to configure it. If
   * needed, we can adjust values like the position or velocity conversion
   * factors.
   */
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

  /*
   * Configure the closed loop controller. We want to make sure we set the
   * feedback sensor as the primary encoder.
   */
  motorConfig.closedLoop
      .SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.1)
      .I(0)
      .D(0)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, ClosedLoopSlot::kSlot1)
      .I(0, ClosedLoopSlot::kSlot1)
      .D(0, ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, ClosedLoopSlot::kSlot1)
      .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot::kSlot1);

  motorConfig.closedLoop
      .maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .CruiseVelocity(1000)
      .MaxAcceleration(1000)
      .AllowedProfileError(1)
      // Set MAXMotion parameters for velocity control in slot 1
      .MaxAcceleration(500, ClosedLoopSlot::kSlot1)
      .CruiseVelocity(6000, ClosedLoopSlot::kSlot1)
      .AllowedProfileError(1, ClosedLoopSlot::kSlot1);

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
                    rev::PersistMode::kPersistParameters);

  // Initialize dashboard values
  frc::SmartDashboard::SetDefaultNumber("Target Position", 0);
  frc::SmartDashboard::SetDefaultNumber("Target Velocity", 0);
  frc::SmartDashboard::SetDefaultBoolean("Control Mode", false);
  frc::SmartDashboard::SetDefaultBoolean("Reset Encoder", false);
}

void Robot::RobotPeriodic() {
  // Display encoder position and velocity
  frc::SmartDashboard::PutNumber("Actual Position", m_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Actual Velocity", m_encoder.GetVelocity());

  if (frc::SmartDashboard::GetBoolean("Reset Encoder", false)) {
    frc::SmartDashboard::PutBoolean("Reset Encoder", false);
    // Reset the encoder position to 0
    m_encoder.SetPosition(0);
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  if (frc::SmartDashboard::GetBoolean("Control Mode", false)) {
    /*
     * Get the target velocity from SmartDashboard and set it as the setpoint
     * for the closed loop controller with MAXMotionVelocityControl as the
     * control type.
     */
    double targetVelocity =
        frc::SmartDashboard::GetNumber("Target Velocity", 0);
    m_closedLoopController.SetSetpoint(
        targetVelocity, SparkMax::ControlType::kMAXMotionVelocityControl,
        ClosedLoopSlot::kSlot1);
  } else {
    /*
     * Get the target position from SmartDashboard and set it as the setpoint
     * for the closed loop controller with MAXMotionPositionControl as the
     * control type.
     */
    double targetPosition =
        frc::SmartDashboard::GetNumber("Target Position", 0);
    m_closedLoopController.SetSetpoint(
        targetPosition, SparkMax::ControlType::kMAXMotionPositionControl,
        ClosedLoopSlot::kSlot0);
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
