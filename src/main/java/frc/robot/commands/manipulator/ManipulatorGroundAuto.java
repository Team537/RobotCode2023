// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManipulatorGroundAuto extends SequentialCommandGroup {

  /** Creates a new ManipulatorHighGoal. */
  public ManipulatorGroundAuto(ArmPivot m_ArmPivot, ArmInOut m_ArmInOut, Wrist m_Wrist, LED m_LED) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // schedules commands to run
    addCommands(new ParallelCommandGroup(

        // calls them from the subsystem class
        new StartEndCommand(m_ArmInOut::armGround, m_ArmInOut::armGround, m_ArmInOut),
        new StartEndCommand(m_ArmPivot::ArmPositionGround, m_ArmPivot::ArmPositionGround, m_ArmPivot),
        new StartEndCommand(m_Wrist::WristPositionZero, m_Wrist::WristPositionZero, m_Wrist)));
  }
}
