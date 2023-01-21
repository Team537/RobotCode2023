// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperIntake extends SubsystemBase {
  WPI_TalonSRX m_Gripper = new WPI_TalonSRX(Constants.GripperConstants.kGripper);

  /** Creates a new GripperIntake. */
  public GripperIntake() {
  m_Gripper.configFactoryDefault(Constants.kTimeoutMs);
  m_Gripper.setNeutralMode(NeutralMode.Brake);


  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void GripperIn() {
    m_Gripper.set(0.5);
  }
  public void GripperStop() {
    m_Gripper.set(0);
  }
  public void GripperOut() {
    m_Gripper.set(-0.5);
  }

}
