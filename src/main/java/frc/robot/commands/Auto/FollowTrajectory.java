// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.lang.reflect.Field;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.swerve.SetSwerveBrakeMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends SequentialCommandGroup{
  private final DriveSubsystem m_drive;
  private final FieldSim m_fieldSim;
  private final String pathName;

  public FollowTrajectory(DriveSubsystem m_drive, FieldSim m_fieldSim, String pathName) {

   this.m_drive = m_drive;
   this.m_fieldSim = m_fieldSim;
   this.pathName = pathName;

   HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("1", new PrintCommand("Passed marker 1"));
eventMap.put("2",new PrintCommand("Passed marker 2"));
eventMap.put("3",new PrintCommand("Passed marker 3"));
eventMap.put("4",new PrintCommand("Passed marker 4"));

   PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, 1, 0.5, false);

   FollowPathWithEvents command = new FollowPathWithEvents(
    m_drive.followTrajectoryCommand(trajectory),
    trajectory.getMarkers(),
    eventMap
);

addCommands(
      new PlotFieldTrajectory(m_fieldSim, trajectory),
      new SetSwerveOdometry(m_drive, trajectory.getInitialPose(), m_fieldSim),
      command,
      new SetSwerveBrakeMode(m_drive, NeutralMode.Brake)
          .andThen(() -> m_drive.drive(0, 0, 0, false, false)));
  }

  



    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  
  }

