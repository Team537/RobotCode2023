// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.commands.manipulator.ManipulatorGround;
import frc.robot.commands.swerve.SetSwerveBrakeMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
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
public class FollowTrajectory extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final ArmInOut m_ArmInOut;
  private final ArmPivot m_ArmPivot;
  private final GripperIntake m_Gripper;
  private final Wrist m_Wrist;
  private final LED m_LED;
  private final FieldSim m_fieldSim;
  private final String pathName;

  public FollowTrajectory(DriveSubsystem m_drive, FieldSim m_fieldSim, String pathName, ArmInOut m_ArmInOut,
      ArmPivot m_ArmPivot, GripperIntake m_Gripper, Wrist m_Wrist, LED m_LED) {
    // setting the subsytem to the one in the drive subsystem, so no duplicates
    this.m_drive = m_drive;
    this.m_fieldSim = m_fieldSim;
    this.pathName = pathName;
    this.m_ArmInOut = m_ArmInOut;
    this.m_ArmPivot = m_ArmPivot;
    this.m_Gripper = m_Gripper;
    this.m_Wrist = m_Wrist;
    this.m_LED = m_LED;

    HashMap<String, Command> eventMap = new HashMap<>();
    // prints a status message once it completes a action
    eventMap.put("1", new PrintCommand("Passed marker 1"));
    eventMap.put("2", new PrintCommand("Passed marker 2"));
    eventMap.put("3", new PrintCommand("Passed marker 3"));
    eventMap.put("4", new PrintCommand("Passed marker 4"));

    // eventMap.put("autoStart", );
    // eventMap.put("autoEnd", new InstantCommand(m_LED::autoEnd));
    // eventMap.put("scoreMid", new ManipulatorMidGoal(m_ArmPivot, m_ArmInOut,
    // m_Wrist, m_LED).withTimeout(5));
    eventMap.put("returnGround", new ManipulatorGround(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED));
    // eventMap.put("gripperOut", new SequentialCommandGroup(
    // new RunCommand(m_Gripper::GripperOut, m_Gripper).withTimeout(2),
    // new RunCommand(m_Gripper::GripperStop, m_Gripper).withTimeout(1)));

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, 4.0, 3.0, false);

    FollowPathWithEvents command = new FollowPathWithEvents(
        // uses the drive method to follow the trajectory.
        m_drive.followTrajectoryCommand(trajectory),
        trajectory.getMarkers(),
        eventMap);

    addCommands(
        new PlotFieldTrajectory(m_fieldSim, trajectory),
        new SetSwerveOdometry(m_drive, trajectory.getInitialPose(), m_fieldSim),
        command,
        new SetSwerveBrakeMode(m_drive, NeutralMode.Brake)
            .andThen(() -> m_drive.slowDrive(0, 0, 0, false, false)));
  }

  // Add your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());

}
