// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

enum State {
  Ground,
  Mounting,
  Balancing
}

public class BalanceChargeStation extends CommandBase {

  private DriveSubsystem m_drive;
  private State state;
  private double lastTimeWhenBalancing = 0;
  private double lastTimeWhenMounting = 0;

  /** Creates a new BalanceChargeStation. */
  public BalanceChargeStation(DriveSubsystem m_drive, boolean isReversed) {

    this.m_drive = m_drive;
    this.state = State.Ground;
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
    double speed = 0.1;

    boolean shouldUseSlowSpeed = true;
    double angleDeadband = 5;

    SmartDashboard.putString("Balance state", state.toString());

    if (Math.abs(gyroPitch) > angleDeadband && this.state == State.Ground) {
      this.state = State.Mounting;
      lastTimeWhenMounting = System.currentTimeMillis();
      speed = (gyroPitch < 0 ? -0.13 : 0.13);

      m_drive.drive(speed, 0, 0, true, true);
      return;
    }

    if (this.state == State.Mounting && gyroPitch < 10) {
      var hasElapsedMountingTimePassedThreshold = 2000 < System.currentTimeMillis() - lastTimeWhenMounting;
      if (hasElapsedMountingTimePassedThreshold) {
        this.state = State.Balancing;
      } else {

        shouldUseSlowSpeed = true;
        speed = (gyroPitch < 0 ? -0.125 : 0.125);
        m_drive.drive(speed, 0, 0, true, true);
      }
      return;
    }

    // State is balancing && we are stable
    if (this.state != State.Balancing || gyroPitch < angleDeadband) {

      speed = 0;
      m_drive.drive(speed, 0, 0, true, true);
      return;
    }

    // Check current time
    // WHen compared to last time, if greater than some threshold:
    // move forward
    // set new current time
    // else return

    var currentTime = System.currentTimeMillis();

    var diff = currentTime - lastTimeWhenBalancing;

    // half a second
    if (diff >= 500) {

      m_drive.drive(speed, 0, 0, true, true);
      lastTimeWhenBalancing = System.currentTimeMillis();
    }

    // if (lastTimeWhenBalancing == 0) {
    // // move
    // // set the time to a current time
    // lastTimeWhenBalancing = System.currentTimeMillis();
    // } else {
    // //
    // }

    // // If the angle of the robot is greater then the deadband, drive forward
    // if (gyroPitch > angleDeadband) {
    // slowSpeed = Math.abs(slowSpeed);
    // fastSpeed = Math.abs(fastSpeed);
    // // If the angle of the robot is less then the deadband, drive backward
    // } else if (gyroPitch < -angleDeadband) {
    // slowSpeed = -Math.abs(slowSpeed);
    // fastSpeed = -Math.abs(fastSpeed);
    // // If the robot is within the deadband, stay still and set the drivetrain to
    // a
    // // diamond shape to prevent movement
    // } else if (gyroPitch < angleDeadband && gyroPitch > -angleDeadband) {
    // slowSpeed = 0;
    // fastSpeed = 0;
    // m_drive.setDiamondShape();
    // }
    // if (Math.abs(gyroPitch) > 12) {
    // shouldUseSlowSpeed = true;
    // } else {
    // shouldUseSlowSpeed = false;
    // }

    // WPIUtilJNI.getSystemTime();

    // // Drive with the speed previously set
    // m_drive.drive(shouldUseSlowSpeed ? slowSpeed : fastSpeed, 0, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setDiamondShape();
    System.out.println("Balance Auto Ended");
  }
}
