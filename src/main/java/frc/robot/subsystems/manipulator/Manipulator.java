// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import java.util.concurrent.Delayed;
import java.util.concurrent.TimeUnit;

import frc.robot.subsystems.GripperIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arminout.ArmInOutGround;
import frc.robot.commands.arminout.ArmInOutHighGoal;
// import frc.robot.commands.arminout.ArmInOutLowGoal;
import frc.robot.commands.arminout.ArmInOutMidGoal;
import frc.robot.commands.arminout.ArmInOutShelf;
import frc.robot.commands.armpivot.ArmPivotHighGoal;
import frc.robot.commands.armpivot.ArmPivotGround;
import frc.robot.commands.armpivot.ArmPivotMidGoal;
import frc.robot.commands.armpivot.ArmPivotShelf;
import frc.robot.commands.gripper.GripperOut;
import frc.robot.commands.wrist.WristGround;
import frc.robot.commands.wrist.WristHighGoal;
// import frc.robot.commands.wrist.WristLowGoal;
import frc.robot.commands.wrist.WristMidGoal;
import frc.robot.commands.wrist.WristShelf;


public class Manipulator extends SubsystemBase {
  // private GripperIntake m_Gripper = new GripperIntake();
  private Wrist m_Wrist = new Wrist();
  private ArmInOut m_ArmInOut = new ArmInOut();
  private ArmPivot m_ArmPivot = new ArmPivot();
  private String manipulatorState = "Low Goal";
  private ArmInOutHighGoal armInOutHighGoal = new ArmInOutHighGoal(m_ArmInOut);
  private ArmInOutMidGoal armInOutMidGoal = new ArmInOutMidGoal(m_ArmInOut);
  private ArmInOutGround armInOutLowGoal = new ArmInOutGround(m_ArmInOut);
  private ArmInOutShelf armInOutShelf = new ArmInOutShelf(m_ArmInOut);
  private ArmPivotHighGoal armPivotHighGoal = new ArmPivotHighGoal(m_ArmPivot);
  private ArmPivotMidGoal armPivotMidGoal = new ArmPivotMidGoal(m_ArmPivot);
  private ArmPivotGround armPivotLowGoal = new ArmPivotGround(m_ArmPivot);
  private ArmPivotShelf armPivotShelf = new ArmPivotShelf(m_ArmPivot);
  private WristGround wristLowGoal = new WristGround(m_Wrist);
  private WristMidGoal wristMidGoal = new WristMidGoal(m_Wrist);
  private WristHighGoal wristHighGoal = new WristHighGoal(m_Wrist);
  private WristShelf wristShelf = new WristShelf(m_Wrist);

 public SequentialCommandGroup highGoal = new SequentialCommandGroup(new ParallelCommandGroup(armPivotHighGoal,wristHighGoal),armInOutHighGoal);
 public SequentialCommandGroup midGoal = new SequentialCommandGroup(new ParallelCommandGroup(armPivotMidGoal,wristMidGoal),armInOutMidGoal);
 public SequentialCommandGroup lowGoal = new SequentialCommandGroup(armInOutLowGoal, new ParallelCommandGroup(armPivotLowGoal,wristLowGoal));
 public SequentialCommandGroup shelf = new SequentialCommandGroup(new ParallelCommandGroup(armPivotShelf,wristShelf),armInOutShelf);
  /** Creates a new Manipulator. */
  public Manipulator() {}


  public void highGoal() {
    //button Y
    
  
     manipulatorState="High Goal";
     
  }

  public void midGoal() {
    
    manipulatorState="Mid Goal";
   
    m_ArmInOut.armTest();
    m_Wrist.WristPositionTest();
    new WaitCommand(0.5);
    m_ArmPivot.ArmPositionTest();
    
    //Button B
  }

  public void ground(){
    manipulatorState="Ground";
    m_ArmPivot.ArmPositionGround();
    new WaitCommand(1);
    m_Wrist.WristPositionGround();
    m_ArmInOut.armGround();
    //Button A
  }

  public void shelf(){
    manipulatorState="Shelf";
    //Button X
  }
  public void zero(){
    manipulatorState="Zero";
    m_ArmPivot.ArmPositionZero();
    m_Wrist.WristPositionZero();
    m_ArmInOut.armZero();

    //right button
  }

  public void scoreMid(){

    m_Wrist.WristPositionMidGoal();
  }
/* 
  public void gripperManipulatorIn(){
    m_Gripper.GripperIn();

  }*/

  @Override
  public void periodic() {
    SmartDashboard.putString("Manipulator State", manipulatorState);
    // This method will be called once per scheduler run
  }
  
}
