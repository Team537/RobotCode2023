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
  public CANSparkMax wristMotor = new CANSparkMax(Constants.WristConstants.WRIST, MotorType.kBrushless);
  private SparkMaxPIDController wristPIDController = wristMotor.getPIDController();
  private RelativeEncoder wristEncoder = wristMotor.getEncoder();
  private String wristState = "Default";
  double wristPositionEnter = 0;

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
    wristPIDController.setP(Constants.SparkPIDFConstants.P);
    wristPIDController.setI(Constants.SparkPIDFConstants.I);
    wristPIDController.setD(Constants.SparkPIDFConstants.D);
    wristPIDController.setIZone(Constants.SparkPIDFConstants.IZONE);
    wristPIDController.setFF(Constants.SparkPIDFConstants.FF);
    wristPIDController.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    wristPIDController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    wristPIDController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    wristPIDController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL_WRIST, 0);
    wristPIDController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
  }

  public void WristPositionMidGoal() {
    WristPidDefaults();
    wristPIDController.setReference(Constants.WristConstants.WRIST_POS_MID_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Mid Goal";

  }

  public void WristPositionHighGoal() {
    WristPidDefaults();
    wristPIDController.setReference(Constants.WristConstants.WRIST_POS_HIGH_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "High Goal";

  }

  public void WristPositionShelfHumanPL() {
    WristPidDefaults();
    wristPIDController.setReference(Constants.WristConstants.WRIST_POS_SHELF_HUMAN_PL,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ShelfMid";
  }

  public void WristPositionGroundForward() {
    WristPidDefaults();
    wristPIDController.setReference(Constants.WristConstants.WRIST_POS_GROUND,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "GroundForward";
  }

  public void WristPositionManualUp() {
    WristPidDefaults();
    wristPIDController.setReference(Constants.WristConstants.WRIST_POS_MANUAL_UP,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ManualUp";
  }

  // public void WristPositionManualDown() {
  // WristPidDefaults();
  // wristPIDController.setReference(Constants.WristConstants.WRIST_POS_MANUAL_DOWN,
  // CANSparkMax.ControlType.kSmartMotion);
  // wristState = "ManualUp";
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Wrist Position", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Velocity", wristEncoder.getVelocity());
    SmartDashboard.putString("Wrist State", wristState);
    // This method will be called once per scheduler run
    wristPositionEnter = SmartDashboard.getNumber("wrist get set pos", wristPositionEnter);
    SmartDashboard.putNumber("wrist get set pos", wristPositionEnter);
  }

  // used for testing, sets wrist position from smart dashboard
  public void WristSetSmartDash() {
    WristPidDefaults();
    wristPIDController.setReference(wristPositionEnter,
        CANSparkMax.ControlType.kSmartMotion);
    wristUpdatePosition();

  }

  public void wristUpdatePosition() {
    wristPositionEnter = SmartDashboard.getNumber("wrist get set pos",
        wristPositionEnter);

  }
}
