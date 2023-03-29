/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.swerve.Swerve;
import frc.robot.Constants.DriveSpeedConstants;
import frc.robot.Constants.GeoConversions;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AdvantageSwerveDriveCommand extends CommandBase {

  private final Swerve m_drive;
  private final DoubleSupplier m_driveInput, m_strafeInput, m_rotationInput, m_triggerInput;
  private final boolean m_isFieldRelative;
  private final LED m_LED;

  /**
   * Creates a new ExampleCommand.
   *
   * 
   */

  public AdvantageSwerveDriveCommand(Swerve m_drive, DoubleSupplier driveInput, DoubleSupplier strafeInput,
      DoubleSupplier rotationInput, DoubleSupplier triggerInput, boolean isFieldRelative, LED m_LED) {

    this.m_drive = m_drive;
    m_driveInput = driveInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_isFieldRelative = isFieldRelative;
    m_triggerInput = triggerInput;
    this.m_LED = m_LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print(" \n Drive Started");
  }

  public double calculateDrivePercentage(double JoystickPercent, double TriggerPercent) {
    double OutputSpeedPercent = 0.0;

    // deadband
    if (Math.abs(JoystickPercent) > 0.05) {
      if (JoystickPercent < 0) {
        TriggerPercent *= -1;
      }

      // OutputSpeedPercent = (JoystickPercent * DriveSpeedConstants.kBaseRobotSpeed)
      // + (JoystickPercent * (TriggerPercent * (DriveSpeedConstants.kMaxRobotSpeed -
      // DriveSpeedConstants.kMaxRobotSpeed)));
      OutputSpeedPercent = JoystickPercent + (Math.abs(JoystickPercent / 0.2) * TriggerPercent);
    } else {
      // when it is lower than the deadband, it sets it to zero
      OutputSpeedPercent = 0;
    }

    return OutputSpeedPercent;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 0.05 are the joystick deadbands. if the joystick is less than that value, is
    // makes it equal to 0
    double drive = calculateDrivePercentage(m_driveInput.getAsDouble(), m_triggerInput.getAsDouble());
    double strafe = calculateDrivePercentage(m_strafeInput.getAsDouble(), m_triggerInput.getAsDouble());
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;

    double linearMagnitude = Math.hypot(drive, strafe);
    Rotation2d linearDirection = new Rotation2d(drive, strafe);
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rotation = Math.copySign(rotation * rotation, rotation);

    SmartDashboard.putNumber("Drive ", drive);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);

    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(GeoConversions.translationToTransform2d(linearMagnitude, 0.0))
        .getTranslation();

    ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * m_drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * m_drive.getMaxLinearSpeedMetersPerSec(),
        rotation);

    var driveRotation = m_drive.getHeadingRotation2d();
    if (DriverStation.getAlliance() == Alliance.Red) {
      driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    }
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        driveRotation);

    m_drive.runVelocity(speeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
