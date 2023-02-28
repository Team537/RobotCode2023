// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ArmPivot;

public class ArmPivotShelf extends CommandBase {

  private ArmPivot m_ArmPivot;
  /** Creates a new ArmPivotLowGoal. */
  public ArmPivotShelf(ArmPivot m_ArmPivot) {

    this.m_ArmPivot = m_ArmPivot;

    addRequirements(m_ArmPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ArmPivot.kArmPositionShelf();
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
