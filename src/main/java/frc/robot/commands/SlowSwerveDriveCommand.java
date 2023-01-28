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
  private final DoubleSupplier m_driveInput, m_strafeInput, m_rotationInput;
  private final boolean m_isFieldRelative;

  /**
   * Creates a new ExampleCommand.
   *
   * 
   */
  public SlowSwerveDriveCommand(DriveSubsystem m_drive, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, boolean isFieldRelative) {
    this.m_drive = m_drive;
    m_driveInput = driveInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_isFieldRelative = isFieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print( " \n Slow Drive Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = Math.abs(m_driveInput.getAsDouble()) > 0.05 ? m_driveInput.getAsDouble() : 0;
    double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.05 ? m_strafeInput.getAsDouble() : 0;
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;

    m_drive.slowDrive(drive, strafe, rotation, m_isFieldRelative, false);    // Forward/Back drive, Left/Right Strafe, Left/Right Turn
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print(" \n Slow Drive Started");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}