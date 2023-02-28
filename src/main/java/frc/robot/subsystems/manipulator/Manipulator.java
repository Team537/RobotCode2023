// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arminout.ArmInOutHighGoal;
import frc.robot.commands.arminout.ArmInOutLowGoal;
import frc.robot.commands.arminout.ArmInOutMidGoal;
import frc.robot.commands.arminout.ArmInOutShelf;
import frc.robot.commands.armpivot.ArmPivotHighGoal;
import frc.robot.commands.armpivot.ArmPivotLowGoal;
import frc.robot.commands.armpivot.ArmPivotMidGoal;
import frc.robot.commands.armpivot.ArmPivotShelf;
import frc.robot.commands.wrist.WristHighGoal;
import frc.robot.commands.wrist.WristLowGoal;
import frc.robot.commands.wrist.WristMidGoal;
import frc.robot.commands.wrist.WristShelf;


public class Manipulator extends SubsystemBase {
  private Wrist m_Wrist = new Wrist();
  private ArmInOut m_ArmInOut = new ArmInOut();
  private ArmPivot m_ArmPivot = new ArmPivot();
  private String manipulatorState = "Low Goal";
  private ArmInOutHighGoal armInOutHighGoal = new ArmInOutHighGoal(m_ArmInOut);
  private ArmInOutMidGoal armInOutMidGoal = new ArmInOutMidGoal(m_ArmInOut);
  private ArmInOutLowGoal armInOutLowGoal = new ArmInOutLowGoal(m_ArmInOut);
  private ArmInOutShelf armInOutShelf = new ArmInOutShelf(m_ArmInOut);
  private ArmPivotHighGoal armPivotHighGoal = new ArmPivotHighGoal(m_ArmPivot);
  private ArmPivotMidGoal armPivotMidGoal = new ArmPivotMidGoal(m_ArmPivot);
  private ArmPivotLowGoal armPivotLowGoal = new ArmPivotLowGoal(m_ArmPivot);
  private ArmPivotShelf armPivotShelf = new ArmPivotShelf(m_ArmPivot);
  private WristLowGoal wristLowGoal = new WristLowGoal(m_Wrist);
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
