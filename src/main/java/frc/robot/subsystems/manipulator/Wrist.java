// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private CANSparkMax m_Wrist = new CANSparkMax(Constants.WristConstants.kWrist, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerPivot1 = m_Wrist.getPIDController();
  // private ArmPivot m_ArmPivot =  new ArmPivot();
  private RelativeEncoder m_encoderPivot1 = m_Wrist.getEncoder();
  // public double armAngle = m_ArmPivot.getArmAngle();
  private String wristState = "Default";


  /** Creates a new ArmPivot. */
  public  Wrist() {

  }

  public void WristPositionHighGoal() {
    m_pidControllerPivot1.setP(Constants.WristConstants.kP);
    m_pidControllerPivot1.setI(Constants.WristConstants.kI);
    m_pidControllerPivot1.setD(Constants.WristConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.WristConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.WristConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.WristConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.WristConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.WristConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionHighGoal, CANSparkMax.ControlType.kSmartMotion);
    wristState = "High Goal";
   
  }

  public void WristPositionMidGoal() {
    m_pidControllerPivot1.setP(Constants.WristConstants.kP);
    m_pidControllerPivot1.setI(Constants.WristConstants.kI);
    m_pidControllerPivot1.setD(Constants.WristConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.WristConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.WristConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.WristConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.WristConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.WristConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionMidGoal, CANSparkMax.ControlType.kSmartMotion);
    wristState = "Mid Goal";

  }

  public void WristPositionLowGoal() {
    m_pidControllerPivot1.setP(Constants.WristConstants.kP);
    m_pidControllerPivot1.setI(Constants.WristConstants.kI);
    m_pidControllerPivot1.setD(Constants.WristConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.WristConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.WristConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.WristConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.WristConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.WristConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionLowGoal, CANSparkMax.ControlType.kSmartMotion);
    wristState = "Low Goal";
  }

  public void WristPositionShelf() {
    m_pidControllerPivot1.setP(Constants.WristConstants.kP);
    m_pidControllerPivot1.setI(Constants.WristConstants.kI);
    m_pidControllerPivot1.setD(Constants.WristConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.WristConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.WristConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.WristConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.WristConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.WristConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionShelf, CANSparkMax.ControlType.kSmartMotion);
    wristState = "Shelf";
  }

  public void WristPositionZero() {
    m_pidControllerPivot1.setP(Constants.WristConstants.kP);
    m_pidControllerPivot1.setI(Constants.WristConstants.kI);
    m_pidControllerPivot1.setD(Constants.WristConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.WristConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.WristConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.WristConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.WristConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.WristConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.WristConstants.kWristPositionZero, CANSparkMax.ControlType.kSmartMotion);
    wristState = "Zero";
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Wrist Position", m_encoderPivot1.getPosition());
    SmartDashboard.putNumber("Wrist Velocity",m_encoderPivot1.getVelocity());
    SmartDashboard.putString("Wrist State", wristState);
    // This method will be called once per scheduler run
  }
}