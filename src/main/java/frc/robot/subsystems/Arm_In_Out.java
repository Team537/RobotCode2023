// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm_In_Out extends SubsystemBase {
  private WPI_TalonFX m_ArmInOut = new WPI_TalonFX(Constants.ArmInOutConstants.kArmInOut);

  /** Creates a new Arm_In_Out. */
   public void ArmInOut() {
    m_ArmInOut.configFactoryDefault();
    m_ArmInOut.setNeutralMode(NeutralMode.Brake);
    m_ArmInOut.configMotionAcceleration(0,Constants.kTimeoutMs);
    m_ArmInOut.configMotionCruiseVelocity(2000,Constants.kTimeoutMs);
    m_ArmInOut.configPeakOutputForward(1.0,Constants.kTimeoutMs);
    m_ArmInOut.configPeakOutputReverse(-1.0,Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // Issues with slotIDX, and configure pid 
 public void ArmOut() {
  m_ArmInOut.set(ControlMode.MotionMagic, Constants.ArmInOutConstants.kOutPos);
  m_ArmInOut.config_kP(0, Constants.ArmInOutConstants.kP);
	m_ArmInOut.config_kI(0, Constants.ArmInOutConstants.kI, Constants.kTimeoutMs);
	m_ArmInOut.config_kD(0, Constants.ArmInOutConstants.kD, Constants.kTimeoutMs);
	m_ArmInOut.config_kF(0, Constants.ArmInOutConstants.kFF, Constants.kTimeoutMs);
	m_ArmInOut.config_IntegralZone(0, Constants.ArmInOutConstants.kIz, Constants.kTimeoutMs);
	m_ArmInOut.configClosedLoopPeakOutput(0, Constants.ArmInOutConstants.kMaxOutput, Constants.kTimeoutMs);
 }

 public void ArmIn() {
  m_ArmInOut.set(ControlMode.MotionMagic, Constants.ArmInOutConstants.kInPos);
  m_ArmInOut.config_kP(0, Constants.ArmInOutConstants.kP);
	m_ArmInOut.config_kI(0, Constants.ArmInOutConstants.kI, Constants.kTimeoutMs);
	m_ArmInOut.config_kD(0, Constants.ArmInOutConstants.kD, Constants.kTimeoutMs);
	m_ArmInOut.config_kF(0, Constants.ArmInOutConstants.kFF, Constants.kTimeoutMs);
	m_ArmInOut.config_IntegralZone(0, Constants.ArmInOutConstants.kIz, Constants.kTimeoutMs);
	m_ArmInOut.configClosedLoopPeakOutput(0, Constants.ArmInOutConstants.kMaxOutput, Constants.kTimeoutMs);
 }
 








}
