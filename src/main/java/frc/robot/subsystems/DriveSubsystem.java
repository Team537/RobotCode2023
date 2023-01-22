// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  private final HashMap<ModulePosition, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              ModulePosition.FRONT_LEFT,
                  new SwerveModule(
                      ModulePosition.FRONT_LEFT,
                      new TalonFX(SwerveConstants.kFrontLeftTurn),
                      new TalonFX(SwerveConstants.kFrontLeftDrive),
                      new CANCoder(SwerveConstants.kFrontLeftCanCoder),
                      SwerveConstants.kFrontLeftCANCoderOffset),
              ModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      ModulePosition.FRONT_RIGHT,
                      new TalonFX(SwerveConstants.kFrontRightTurn),
                      new TalonFX(SwerveConstants.kFrontRightDrive),
                      new CANCoder(SwerveConstants.kFrontRightCanCoder),
                      SwerveConstants.kFrontRightCANCoderOffset),
              ModulePosition.BACK_LEFT,
                  new SwerveModule(
                      ModulePosition.BACK_LEFT,
                      new TalonFX(SwerveConstants.kBackLeftTurn),
                      new TalonFX(SwerveConstants.kBackLeftDrive),
                      new CANCoder(SwerveConstants.kBackLeftCanCoder),
                     SwerveConstants.kBackLeftCANCoderOffset),
              ModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      ModulePosition.BACK_RIGHT,
                      new TalonFX(SwerveConstants.kBackRightTurn),
                      new TalonFX(SwerveConstants.kBackRightDrive),
                      new CANCoder(SwerveConstants.kBackRightCanCoder),
                      SwerveConstants.kBackRightCANCoderOffset)));

  private final Pigeon2 m_gyro = new Pigeon2(9);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          SwerveConstants.kDriveKinematics,
          getHeadingRotation2d(),
          getModulePositions(),
          new Pose2d());

  private PIDController m_xController = new PIDController(SwerveConstants.kP_X, SwerveConstants.kI_X,  SwerveConstants.kD_X);
  private PIDController m_yController = new PIDController(SwerveConstants.kP_Y, SwerveConstants.kI_Y,  SwerveConstants.kD_Y);
  private ProfiledPIDController m_turnController =
      new ProfiledPIDController(SwerveConstants.kP_Theta, SwerveConstants.kI_Theta,SwerveConstants.kD_Theta , SwerveConstants.kThetaControllerConstraints);

  private double m_simYaw;

  public DriveSubsystem() {
    m_gyro.setYaw(0);
  }

  public void drive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    drive *= SwerveConstants.kMaxSpeedMetersPerSecond;
    strafe *= SwerveConstants.kMaxSpeedMetersPerSecond;
    rotation *= SwerveConstants.kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                drive, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(drive, strafe, rotation);

    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setOdometry(Pose2d pose) {
    m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    
    m_gyro.setYaw(pose.getRotation().getDegrees()); 
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_gyro.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModule getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : m_swerveModules.keySet()) {
      map.put(i, m_swerveModules.get(i).getState());
    }
    return map;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public ProfiledPIDController getThetaPidController() {
    return m_turnController;
  }

  public void setNeutralMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),
        getModulePositions());

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis =
          SwerveConstants.kModuleTranslations
              .get(module.getModulePosition())
              .rotateBy(getHeadingRotation2d())
              .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        SwerveConstants.kDriveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(2);
    m_gyro.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }



}
