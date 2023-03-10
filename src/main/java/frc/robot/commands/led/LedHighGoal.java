// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class LedHighGoal extends CommandBase {


  private LED m_LED;
 
  /** Creates a new ManipulatorHighGoal. */
  //links led to robotcontainer
  public LedHighGoal(LED m_LED) {

      
        this.m_LED = m_LED;

       
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // highGoal = m_Manipulator.highGoal(); 

    m_LED.setHighGoal(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_LED.setHighGoal(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
