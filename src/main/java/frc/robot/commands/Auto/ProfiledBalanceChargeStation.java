// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ProfiledBalanceChargeStation extends CommandBase {

  private DriveSubsystem m_drive;
  private PIDController pitchController = new PIDController(1, 0, 0);

  /** Creates a new BalanceChargeStation. */
  public ProfiledBalanceChargeStation(DriveSubsystem m_drive, boolean isReversed) {

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
    double speed = 0;

    gyroPitch = MathUtil.applyDeadband(gyroPitch, 5);
    speed = MathUtil.clamp(pitchController.calculate(gyroPitch), -0.1, 0.1);

    // Drive with the speed previously set
    m_drive.drive(speed, 0, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setDiamondShape();
  }
}
