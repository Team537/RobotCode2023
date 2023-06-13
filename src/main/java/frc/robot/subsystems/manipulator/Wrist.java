// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax wristMotor = new CANSparkMax(Constants.WristConstants.WRIST, MotorType.kBrushless);
  private SparkMaxPIDController wristPIDController = wristMotor.getPIDController();
  private RelativeEncoder wristEncoder = wristMotor.getEncoder();
  private String wristState = "Default";

  /** Creates a new ArmPivot. */
  public Wrist() {
    wristPIDController.setP(Constants.SparkPIDFConstants.P);
    wristPIDController.setI(Constants.SparkPIDFConstants.I);
    wristPIDController.setD(Constants.SparkPIDFConstants.D);
    wristPIDController.setIZone(Constants.SparkPIDFConstants.IZONE);
    wristPIDController.setFF(Constants.SparkPIDFConstants.FF);
    wristPIDController.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    wristPIDController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    wristPIDController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    wristPIDController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    wristPIDController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);

  }

  public void WristPidDefaults() {
    m_WristPidController.setP(Constants.SparkPIDFConstants.kP);
    m_WristPidController.setI(Constants.SparkPIDFConstants.kI);
    m_WristPidController.setD(Constants.SparkPIDFConstants.kD);
    m_WristPidController.setIZone(Constants.SparkPIDFConstants.kIz);
    m_WristPidController.setFF(Constants.SparkPIDFConstants.kFF);
    m_WristPidController.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_WristPidController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxVelocityWrist, 0);
    m_WristPidController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_WristPidController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelWrist, 0);
    m_WristPidController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
  }

  public void WristPositionMidGoal() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionMidGoal,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Mid Goal";

  }

  public void WristPositionHighGoal() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionHighGoal,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "High Goal";

  }

  public void WristPositionShelfHumanPL() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionShelfHumanPL,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ShelfMid";
  }

  public void WristPositionZero() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionZero,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Zero";
  }

  public void WristPositionGroundForward() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionGroundForward,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "GroundForward";
  }

  public void WristPositionManualUp() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionManualUp,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ManualUp";
  }

  public void WristPositionManualDown() {
    WristPidDefaults();
    m_WristPidController.setReference(Constants.WristConstants.kWristPositionManualDown,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ManualUp";
  }

  /*
   * public void WristPositionGroundBack() {
   * WristPidDefaults();
   * m_WristPidController.setReference(Constants.WristConstants.
   * kWristPositionGroundBack,
   * CANSparkMax.ControlType.kSmartMotion);
   * wristState = "GroundBack";
   * }
   */

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Wrist Position", m_WristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Velocity", m_WristEncoder.getVelocity());
    SmartDashboard.putString("Wrist State", wristState);
    // This method will be called once per scheduler run
    wristPositionEnter = SmartDashboard.getNumber("wrist get set pos", wristPositionEnter);
    SmartDashboard.putNumber("wrist get set pos", wristPositionEnter);
  }

  public void WristSetSmartDash() {
    WristPidDefaults();
    m_WristPidController.setReference(wristPositionEnter,
        CANSparkMax.ControlType.kSmartMotion);
    wristUpdatePosition();

  }

  public void wristUpdatePosition() {
    wristPositionEnter = SmartDashboard.getNumber("wrist get set pos", wristPositionEnter);

  }
}
