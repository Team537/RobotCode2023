// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperIntake;
import frc.robot.subsystems.LED;

public class GripperIn extends CommandBase {
   
  private GripperIntake m_GripperIntake;
  private LED m_LED;

  /** Creates a new GripperIn. */
  public GripperIn(GripperIntake m_GripperIntake, LED m_LED) {
    this.m_GripperIntake = m_GripperIntake;
    this.m_LED = m_LED;

    addRequirements(m_GripperIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_LED.setIntaking(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_GripperIntake.GripperIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LED.setIntaking(false);
    m_GripperIntake.GripperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
