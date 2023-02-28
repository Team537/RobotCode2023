// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
public class ExampleTrajectory extends SequentialCommandGroup {
  /** Creates a new ExampleTrajectory. */
  public ExampleTrajectory(DriveSubsystem m_drive, FieldSim m_fieldSim) {
PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", 1, 0.5, false);

PPSwerveControllerCommand command =
new PPSwerveControllerCommand(
   trajectory,
   m_drive::getPoseMeters, SwerveConstants.kDriveKinematics, m_drive.getXPidController(), m_drive.getYPidController(),
    m_drive.getRotPidControllerAuto(), m_drive::setSwerveModuleStatesAuto, m_drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlotFieldTrajectory(m_fieldSim, trajectory),
      new SetSwerveOdometry(m_drive, trajectory.getInitialPose(), m_fieldSim),
      command,
      new SetSwerveBrakeMode(m_drive, NeutralMode.Brake)
          .andThen(() -> m_drive.drive(0, 0, 0, false, false)));
  }
}
