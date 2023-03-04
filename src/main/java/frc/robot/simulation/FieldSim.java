// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.ModuleMap;

import java.util.Map;
/**
 * 
 * Simulates Field in Simulations
 */

public class FieldSim {
  private final DriveSubsystem m_drive;

  private final Field2d m_field2d = new Field2d();

  private final Map<ModulePosition, Pose2d> m_swerveModulePoses =
      ModuleMap.of(new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d());

  public FieldSim(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  public void setTrajectory(Trajectory trajectory) {
    m_field2d.getObject("trajectory").setTrajectory(trajectory);
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d.setRobotPose(pose);
  }
  
  
  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_drive.getPoseMeters());

    for (ModulePosition i : ModulePosition.values()) {
      Translation2d updatedPositions =
          SwerveConstants.kModuleTranslations
              .get(i)
              .rotateBy(m_drive.getPoseMeters().getRotation())
              .plus(m_drive.getPoseMeters().getTranslation());
      m_swerveModulePoses.put(
          i,
          new Pose2d(
              updatedPositions,
              m_drive
                  .getSwerveModule(i)
                  .getHeadingRotation2d()
                  .plus(m_drive.getHeadingRotation2d())));
    }

    m_field2d
        .getObject("Swerve Modules")
        .setPoses(ModuleMap.orderedValues(m_swerveModulePoses, new Pose2d[0]));
  }

  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
