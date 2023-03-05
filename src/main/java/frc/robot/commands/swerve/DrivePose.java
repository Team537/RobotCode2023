// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Conversions.GeoConversions;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivePose extends CommandBase {
  private final DriveSubsystem m_drive;
  private final Supplier<Pose2d> poseSupplier;
  
  private boolean active = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          1, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 0.5), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 0.5), 0.02);
  private double driveErrorAbs = 0.01;
  private double thetaErrorAbs =  0.0175;
  /** Creates a new DrivePose. */

  public DrivePose(DriveSubsystem m_drive, Supplier<Pose2d> pSupplier) {
    this.m_drive = m_drive;
    poseSupplier = pSupplier;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    var currentPose = m_drive.getPoseMeters();
    driveController.reset( currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    active = true;
    var currentPose = m_drive.getPoseMeters();
    var targetPose = poseSupplier.get();

    double currentDistance =
    currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    driveErrorAbs = currentDistance;

    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;

    double thetaVelocity =
    thetaController.calculate(
        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    var driveVelocity =
    new Pose2d(
            new Translation2d(),
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
        .transformBy(GeoConversions.translationToTransform2d(driveVelocityScalar, 0.0))
        .getTranslation();
      
        m_drive.drivePose(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    active = false;
    m_drive.drive(0, 0, 0, true, false);
  }
  public boolean atGoal() {
    return active && driveController.atGoal() && thetaController.atGoal();
  }

  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return active
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public boolean isRunning() {
    return active;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
