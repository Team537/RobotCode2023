// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.simulation.FieldSim;


//sends things to the field simulation
public class PlotFieldTrajectory extends CommandBase {

  FieldSim m_fieldSim;
  Trajectory m_trajectory;

  /** Creates a new PlotFieldTrajectory. */
  public PlotFieldTrajectory(FieldSim m_fieldSim, Trajectory m_traj) {

    this.m_fieldSim = m_fieldSim;
    m_trajectory = m_traj; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_fieldSim.setTrajectory(m_trajectory);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
