// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GripperConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperIntake extends SubsystemBase {
  CANSparkMax m_Gripper = new CANSparkMax(GripperConstants.kGripper, MotorType.kBrushless);
  CANSparkMax m_Gripper2 = new CANSparkMax(GripperConstants.kGripper2, MotorType.kBrushless);
 
  private String gripperState = "Stopped";

  

  /** Creates a new GripperIntake. */
  public GripperIntake() {
   
  } 

  @Override
  public void periodic() {
    SmartDashboard.putString("Gripper State", gripperState);
    // This method will be called once per scheduler run
  }
  public void GripperIn() {

    m_Gripper.set(0.25);
    m_Gripper2.set(-0.25);
    gripperState = "Intaking";
  }
  public void GripperStop() {
    m_Gripper.set(0);
    m_Gripper2.set(0);
    gripperState = "Stopped";
  }
  public void GripperOut() {
    m_Gripper.set(-0.05);
    m_Gripper2.set(0.05);
    gripperState = "Outaking";
  }
  public void GripperFast() {
    m_Gripper.set(-0.8);
    m_Gripper2.set(0.8);
    gripperState = "SuperSpeed";
  }

}
