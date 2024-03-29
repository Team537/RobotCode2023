/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SwerveDriveCommand extends CommandBase {

  private final DriveSubsystem m_drive;
  private final LED m_LED;
  private final DoubleSupplier m_driveInput, m_strafeInput, m_rotationInput, m_triggerInput;
  private final boolean m_isFieldRelative;

  /**
   * Creates a new ExampleCommand.
   *
   * 
   */
  public SwerveDriveCommand(DriveSubsystem m_drive, DoubleSupplier driveInput, DoubleSupplier strafeInput,
      DoubleSupplier rotationInput, DoubleSupplier triggerInput, boolean isFieldRelative, LED m_LED) {
    this.m_drive = m_drive;
    m_driveInput = driveInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_triggerInput = triggerInput;
    m_isFieldRelative = isFieldRelative;

    this.m_LED = m_LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print(" \n Swerve Drive Started");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = (30 * m_triggerInput.getAsDouble() + 1) * (Math.abs(m_driveInput.getAsDouble())) > 0.05
        ? m_driveInput.getAsDouble()
        : 0;
    double strafe = (30 * m_triggerInput.getAsDouble() + 1) * Math.abs(m_strafeInput.getAsDouble()) > 0.05
        ? m_strafeInput.getAsDouble()
        : 0;
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;

    m_drive.drive(drive, strafe, rotation, m_isFieldRelative); // Forward/Back Drive, Left/Right Strafe,
                                                               // Left/Right Turn

    if ((Math.abs(m_drive.getVelocity()) > 0) && m_drive.driveState.equals("Boost Drive") && DriverStation.isTeleop()) {
      m_LED.setBoostDriving(true);
      m_LED.setDriving(false);
    } else {
      m_LED.setBoostDriving(false);
    }

    if (Math.abs(m_drive.getPitch()) > 75 || Math.abs(m_drive.getRoll()) > 75) {
      m_LED.setFallen(true);
    } else {
      m_LED.setFallen(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print(" \n Swerve Drive Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
