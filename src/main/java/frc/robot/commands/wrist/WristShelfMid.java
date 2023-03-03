// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.Wrist;


public class WristShelfMid extends CommandBase {

  private Wrist m_Wrist;
  /** Creates a new ArmPivotLowGoal. */
  public WristShelfMid(Wrist m_Wrist) {

    this.m_Wrist = m_Wrist;

    addRequirements(m_Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Wrist.WristPositionShelfMid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
