// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmInOut extends SubsystemBase {
  /** Creates a new ArmInOut. */
  public ArmInOut() {}

  private CANSparkMax m_climb = new CANSparkMax(Constants.ArmInOutConstants.kArmInOut, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerClimb = m_climb.getPIDController();
  private RelativeEncoder m_encoderExtension = m_climb.getEncoder();
  private double m_incrementUp;
  private double m_incrementDown;
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position", m_encoderExtension.getPosition());
    SmartDashboard.putNumber("Extension Velocity",m_encoderExtension.getVelocity());
    
    // This method will be called once per scheduler run
  }

  public void armIn() {
    m_pidControllerClimb.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerClimb.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerClimb.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerClimb.setReference(Constants.ArmInOutConstants.kRotationsUp, CANSparkMax.ControlType.kSmartMotion);

  }

  public void armOut() {
    m_pidControllerClimb.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerClimb.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerClimb.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerClimb.setReference(Constants.ArmInOutConstants.kRotationsDown, CANSparkMax.ControlType.kSmartMotion);

  }

  public void armIncrementUp() {

    m_incrementUp = Constants.ArmInOutConstants.kIncrementUp + m_encoderExtension.getPosition();

    if(m_incrementUp < -350){
      m_incrementUp = m_encoderExtension.getPosition();
    }

    
    
    m_pidControllerClimb.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerClimb.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerClimb.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerClimb.setReference(m_incrementUp, CANSparkMax.ControlType.kSmartMotion);

  }
  public void armIncrementDown() {

    m_incrementDown = Constants.ArmInOutConstants.kIncrementDown + m_encoderExtension.getPosition();

    if(m_incrementDown > 0){
      m_incrementDown= m_encoderExtension.getPosition();
    }

    
    
    m_pidControllerClimb.setP(Constants.ArmInOutConstants.kP);
    m_pidControllerClimb.setI(Constants.ArmInOutConstants.kI);
    m_pidControllerClimb.setD(Constants.ArmInOutConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ArmInOutConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ArmInOutConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ArmInOutConstants.kMinOutput, Constants.ArmInOutConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ArmInOutConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ArmInOutConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ArmInOutConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ArmInOutConstants.kAllE, 0);
    m_pidControllerClimb.setReference(m_incrementDown, CANSparkMax.ControlType.kSmartMotion);

  }
}
