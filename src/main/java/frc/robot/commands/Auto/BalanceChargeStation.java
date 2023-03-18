// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceChargeStation extends CommandBase {

  private DriveSubsystem m_drive;

  /** Creates a new BalanceChargeStation. */
  public BalanceChargeStation(DriveSubsystem m_drive, boolean isReversed) {

    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Balance Auto");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var gyroPitch = m_drive.getGyroPitch();
    double speed = 0.08;
    double angleDeadband = 5;

    // If the angle of the robot is greater then the deadband, drive forward
    if (gyroPitch > angleDeadband) {

      speed = Math.abs(speed);

      // If the angle of the robot is less then the deadband, drive backward
    } else if (gyroPitch < -angleDeadband) {

      speed = -Math.abs(0.08);

      // If the robot is within the deadband, stay still and set the drivetrain to a
      // diamond shape to prevent movement
    } else if (gyroPitch < angleDeadband && gyroPitch > -angleDeadband) {

      speed = 0;
      m_drive.setDiamondShape();

    }

    // Drive with the speed previously set
    m_drive.drive(speed, 0, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setDiamondShape();
  }
}
