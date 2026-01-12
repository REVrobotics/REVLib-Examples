// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkLimitSwitch forwardLimitSwitch;
  private SparkLimitSwitch reverseLimitSwitch;
  private RelativeEncoder encoder;

  public Robot() {
    /*
     * Initialize the SPARK MAX and get its limit switch and encoder objects for
     * later use.
     */
    motor = new SparkMax(1, MotorType.kBrushless);
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    reverseLimitSwitch = motor.getReverseLimitSwitch();
    encoder = motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    // Set the idle mode to brake to stop immediately when reaching a limit
    motorConfig.idleMode(IdleMode.kBrake);

    // Enable limit switches to stop the motor when they are closed
    motorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    // Set the soft limits to stop the motor at -50 and 50 rotations
    motorConfig.softLimit
        .forwardSoftLimit(50)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-50)
        .reverseSoftLimitEnabled(true);

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
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Reset the position to 0 to start within the range of the soft limits
    encoder.setPosition(0);

    // Initialize dashboard values
    SmartDashboard.setDefaultBoolean("Direction", true);
  }

  @Override
  public void robotPeriodic() {
    // Display data from SPARK onto the dashboard
    SmartDashboard.putBoolean("Forward Limit Reached", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Reached", reverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Applied Output", motor.getAppliedOutput());
    SmartDashboard.putNumber("Position", encoder.getPosition());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Set the motor setpoint based on the direction from the dashboard
    if (SmartDashboard.getBoolean("Direction", true)) {
      motor.set(0.2);
    } else {
      motor.set(-0.2);
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
