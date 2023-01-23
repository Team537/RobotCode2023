/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SlowSwerveDriveCommand extends CommandBase {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private final boolean m_isFieldRelative;

  /**
   * Creates a new ExampleCommand.
   *
   * 
   */
  public SlowSwerveDriveCommand(DriveSubsystem m_drive, DoubleSupplier throttleInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, boolean isFieldRelative) {
    this.m_drive = m_drive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_isFieldRelative = isFieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.05 ? m_throttleInput.getAsDouble() : 0;
    double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.05 ? m_strafeInput.getAsDouble() : 0;
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;

    m_drive.slowDrive(throttle, strafe, rotation, m_isFieldRelative, false);    // Forward/Back Trottle, Left/Right Strafe, Left/Right Turn
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
