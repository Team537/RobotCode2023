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
  private CANSparkMax m_Wrist = new CANSparkMax(Constants.WristConstants.kWrist, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerPivot1 = m_Wrist.getPIDController();
  private RelativeEncoder m_encoderPivot1 = m_Wrist.getEncoder();
  private String wristState = "Default";

  /** Creates a new ArmPivot. */
  public Wrist() {

  }

  public void WristPositionMidGoal() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionMidGoal,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Mid Goal";

  }

  public void WristPositionHighGoal() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionHighGoal,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "High Goal";

  }

  public void WristPositionShelfHumanPL() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionShelfHumanPL,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ShelfMid";
  }

  public void WristPositionZero() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionZero,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Zero";
  }

  public void WristPositionGround() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionGround,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "Ground";
  }

  public void WristPositionManualUp() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionManualUp,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ManualUp";
  }

  public void WristPositionManualDown() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionManualDown,
        CANSparkMax.ControlType.kSmartMotion);
    wristState = "ManualUp";
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Wrist Position", m_encoderPivot1.getPosition());
    SmartDashboard.putNumber("Wrist Velocity", m_encoderPivot1.getVelocity());
    SmartDashboard.putString("Wrist State", wristState);
    // This method will be called once per scheduler run
  }
}
