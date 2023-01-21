// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm_In_Out extends SubsystemBase {
  WPI_TalonFX m_ArmInOut = new WPI_TalonFX(Constants.ArmInOutConstants.kArmInOut);


  /** Creates a new Arm_In_Out. */
  public Arm_In_Out() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp() {
    m_pidControllerClimb.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb.setReference(Constants.ClimberConstants.kLeftRotationsUp, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerClimb2.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb2.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb2.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb2.setReference(Constants.ClimberConstants.kRightRotationsUp, CANSparkMax.ControlType.kSmartMotion);
  }









}
