// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

public class ManipulatorLowGoal extends CommandBase {

  private Manipulator m_Manipulator;
  private LED m_LED;
  /** Creates a new ManipulatorHighGoal. */
  public ManipulatorLowGoal(Manipulator m_Manipulator, LED m_LED) {

        this.m_Manipulator = m_Manipulator;
        this.m_LED = m_LED;

        addRequirements(m_Manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_LED.setLowGoal(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Manipulator.lowGoal(); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_LED.setLowGoal(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
