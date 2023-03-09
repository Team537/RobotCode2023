// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manipulator.ManipulatorMidGoal;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperIntake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class ScoreMidDriveBack extends SequentialCommandGroup {
  /** Creates a new ScoreMidDriveBack. */

  //defineing the requirements for motors it needs to run. 
  public ScoreMidDriveBack(DriveSubsystem m_drive, FieldSim m_fieldSim,  ArmInOut m_ArmInOut, ArmPivot m_ArmPivot, GripperIntake m_Gripper, Wrist m_Wrist, LED m_LED) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //adds commands to run sequentially when executed
    addCommands(
      //run instantly
      new InstantCommand(m_LED::autoStart),

      
      new ManipulatorMidGoal(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED).withTimeout(2),
      new RunCommand(m_Gripper::GripperOut, m_Gripper).withTimeout(2),
      new RunCommand(m_Gripper::GripperStop, m_Gripper).withTimeout(1),
      //new FollowTrajectory(m_drive, m_fieldSim, "Drive Backward", m_ArmInOut, m_ArmPivot, m_Gripper, m_Wrist, m_LED),
      new RunCommand (() -> m_drive.drive(-0.5, 0, 0, false, true), m_drive).withTimeout(5),
      new InstantCommand(m_LED::autoEnd)
      );
  }
}
