// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arminout;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ArmInOut;


public class ArmInOutShelfHumanPL extends CommandBase {

  private ArmInOut m_ArmInOut;
  /** Creates a new ArmPivotLowGoal. */
  public ArmInOutShelfHumanPL(ArmInOut m_ArmInOut) {

    this.m_ArmInOut = m_ArmInOut;

    addRequirements(m_ArmInOut);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ArmInOut.armShelfHumanPL();
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
