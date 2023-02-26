// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmInOut;
import frc.robot.subsystems.ArmPivot;


public class Manipulator extends SubsystemBase {
  private Wrist m_Wrist = new Wrist();
  private ArmInOut m_ArmInOut = new ArmInOut();
  private ArmPivot m_ArmPivot = new ArmPivot();
  private String manipulatorState = "Low Goal";

 

  /** Creates a new Manipulator. */
  public Manipulator() {}


  public void highGoal() {
    //button Y
    m_Wrist.WristPositionUp();
    m_ArmInOut.armIn();
    m_ArmPivot.ArmPositionDown();
     manipulatorState="High Goal";
     
  }

  public void midGoal() {
    
    manipulatorState="Mid Goal";
   
    //Button B
  }

  public void lowGoal() {
    manipulatorState="Low Goal";
    //Button A
  }

  public void shelf(){
    manipulatorState="Shelf";
    //Button X
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Manipulator State", manipulatorState);
    // This method will be called once per scheduler run
  }
  
}
