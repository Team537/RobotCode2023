// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // Issues with slotIDX, and configure pid 
 public void ArmOut() {
  m_ArmInOut.config_kP(0, Constants.ClimberConstants.kP);
	m_ArmInOut.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
	m_ArmInOut.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
	m_ArmInOut.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
	m_ArmInOut.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
	m_ArmInOut.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);
  m_ArmInOut.set
 }
 








}
