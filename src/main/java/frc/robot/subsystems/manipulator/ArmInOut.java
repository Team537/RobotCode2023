// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmInOut extends SubsystemBase {
  /** Creates a new ArmInOut. */
  public ArmInOut() {}

  private CANSparkMax m_extension = new CANSparkMax(Constants.ArmInOutConstants.kArmInOut, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerExtension = m_extension.getPIDController();
  private RelativeEncoder m_encoderExtension = m_extension.getEncoder();
  private double m_incrementUp;
  private double m_incrementDown;
  private String armInOutState = "Default";
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Extend Position", m_encoderExtension.getPosition());
    SmartDashboard.putNumber("Extension Velocity",m_encoderExtension.getVelocity());
    SmartDashboard.putString("Arm In Out State", armInOutState);
    
    // This method will be called once per scheduler run
  }

  public void armMidGoal() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionMidGoal, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Mid Goal";
  }

  public void armHighGoal() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionHighGoal, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "High Goal";

  }

  public void armShelfMid() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionShelfMid, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfMid";
  }

  public void armShelfHigh() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionShelfHigh, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfHigh";

  }

  public void armZero() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionZero, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Zero";
  }

  public void armGround() {
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionGround, CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Ground";

  }

  /*public void armTest() {
      m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
      m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
      m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
      m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
      m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
      m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
      m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
      m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
      m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
      m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
      m_pidControllerExtension.setReference(Constants.ArmInOutConstants.kArmInOutPositionTest, CANSparkMax.ControlType.kSmartMotion);
      armInOutState = "Test";
    }*/

  /*public void armIncrementUp() {

    m_incrementUp = Constants.ArmInOutConstants.kIncrementUp + m_encoderExtension.getPosition();

    if(m_incrementUp < -350){
      m_incrementUp = m_encoderExtension.getPosition();
    }

    
    
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(m_incrementUp, CANSparkMax.ControlType.kSmartMotion);

  }
  public void armIncrementDown() {

    m_incrementDown = Constants.ArmInOutConstants.kIncrementDown + m_encoderExtension.getPosition();

    if(m_incrementDown > 0){
      m_incrementDown= m_encoderExtension.getPosition();
    }

    
    
    m_pidControllerExtension.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerExtension.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerExtension.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerExtension.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerExtension.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerExtension.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerExtension.setReference(m_incrementDown, CANSparkMax.ControlType.kSmartMotion);

  }*/
}
