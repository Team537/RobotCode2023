// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.arminout.ArmInOutGround;
// import frc.robot.commands.arminout.ArmInOutHighGoal;
// import frc.robot.commands.arminout.ArmInOutMidGoal;
// import frc.robot.commands.arminout.ArmInOutShelfHumanPL;
// import frc.robot.commands.armpivot.ArmPivotGround;
// import frc.robot.commands.armpivot.ArmPivotHighGoal;
// import frc.robot.commands.armpivot.ArmPivotMidGoal;
// import frc.robot.commands.armpivot.ArmPivotShelfHumanPL;
// import frc.robot.commands.wrist.WristGround;
// import frc.robot.commands.wrist.WristHighGoal;
// import frc.robot.commands.wrist.WristMidGoal;
// import frc.robot.commands.wrist.WristShelfHumanPL;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */

  private Wrist m_Wrist = new Wrist();
  private ArmInOut m_ArmInOut = new ArmInOut();
  private ArmPivot m_ArmPivot = new ArmPivot();
  private String manipulatorState = "Low Goal";
  // private ArmInOutHighGoal armInOutHighGoal = new ArmInOutHighGoal(m_ArmInOut);
  // private ArmInOutMidGoal armInOutMidGoal = new ArmInOutMidGoal(m_ArmInOut);
  // private ArmInOutGround armInOutLowGoal = new ArmInOutGround(m_ArmInOut);
  // private ArmInOutShelfHumanPL armInOutShelf = new ArmInOutShelfHumanPL(m_ArmInOut);
  // private ArmPivotHighGoal armPivotHighGoal = new ArmPivotHighGoal(m_ArmPivot);
  // private ArmPivotMidGoal armPivotMidGoal = new ArmPivotMidGoal(m_ArmPivot);
  // private ArmPivotGround armPivotLowGoal = new ArmPivotGround(m_ArmPivot);
  // private ArmPivotShelfHumanPL armPivotShelf = new ArmPivotShelfHumanPL(m_ArmPivot);
  // private WristGround wristLowGoal = new WristGround(m_Wrist);
  // private WristMidGoal wristMidGoal = new WristMidGoal(m_Wrist);
  // private WristHighGoal wristHighGoal = new WristHighGoal(m_Wrist);
  // private WristShelfHumanPL wristShelf = new WristShelfHumanPL(m_Wrist);
  
  public Manipulator() {}


  public void ManipulatorMidGoal() {
    // new SequentialCommandGroup(m_Wrist.WristPositionMidGoal());
    m_Wrist.WristPositionMidGoal();
    m_ArmInOut.armMidGoal();
    m_ArmPivot.ArmPositionMidGoal();
    
  }
  
  public void ManipulatorHighGoal() {

  }

  public void ManipulatorShelfHumanPL() {

  }

  public void ManipulatorGround() {

  }

  public void ManipulatorZero() {

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
