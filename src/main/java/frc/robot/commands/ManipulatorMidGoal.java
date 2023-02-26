// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

public class ManipulatorMidGoal extends CommandBase {

  private Manipulator m_Manipulator;
  private LED m_LED;
  /** Creates a new ManipulatorHighGoal. */
  public ManipulatorMidGoal(Manipulator m_Manipulator, LED m_LED) {

        this.m_Manipulator = m_Manipulator;
        this.m_LED = m_LED;

        addRequirements(m_Manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_LED.setMidGoal(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Manipulator.midGoal(); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_LED.setMidGoal(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
