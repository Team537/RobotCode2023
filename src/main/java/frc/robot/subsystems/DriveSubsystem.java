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
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  // Hash Map for All Swerve Modules

  private final HashMap<ModulePosition, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              ModulePosition.FRONT_LEFT,
                  new SwerveModule(
                      ModulePosition.FRONT_LEFT,
                      new WPI_TalonFX(SwerveConstants.kFrontLeftTurn),
                      new WPI_TalonFX(SwerveConstants.kFrontLeftDrive),
                      new CANCoder(SwerveConstants.kFrontLeftCanCoder),
                      SwerveConstants.kFrontLeftCANCoderOffset, false),
              ModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      ModulePosition.FRONT_RIGHT,
                      new WPI_TalonFX(SwerveConstants.kFrontRightTurn),
                      new WPI_TalonFX(SwerveConstants.kFrontRightDrive),
                      new CANCoder(SwerveConstants.kFrontRightCanCoder),
                      SwerveConstants.kFrontRightCANCoderOffset, false),
              ModulePosition.BACK_LEFT,
                  new SwerveModule(
                      ModulePosition.BACK_LEFT,
                      new WPI_TalonFX(SwerveConstants.kBackLeftTurn),
                      new WPI_TalonFX(SwerveConstants.kBackLeftDrive),
                      new CANCoder(SwerveConstants.kBackLeftCanCoder),
                     SwerveConstants.kBackLeftCANCoderOffset, false),
              ModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      ModulePosition.BACK_RIGHT,
                      new WPI_TalonFX(SwerveConstants.kBackRightTurn),
                      new WPI_TalonFX(SwerveConstants.kBackRightDrive),
                      new CANCoder(SwerveConstants.kBackRightCanCoder),
                      SwerveConstants.kBackRightCANCoderOffset, false)));


  //Gyro and Simulated Gyro  
                    
  private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kPigeonID);
//   private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);


  //Swerve Odometry 

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          SwerveConstants.kDriveKinematics,
          getHeadingRotation2d(),
          getModulePositions(),
          new Pose2d());

  //PID Controllers for X, Y, and Rotation
  //Rotation is Profiled to limit Uncontrollable Rotation

  private PIDController m_XController = new PIDController(SwerveConstants.kP_X, SwerveConstants.kI_X,  SwerveConstants.kD_X);
  private PIDController m_YController = new PIDController(SwerveConstants.kP_Y, SwerveConstants.kI_Y,  SwerveConstants.kD_Y);
  private ProfiledPIDController m_turnController =
      new ProfiledPIDController(SwerveConstants.kP_Rot, SwerveConstants.kI_Rot,SwerveConstants.kD_Rot , SwerveConstants.kRotControllerConstraints);
      private PIDController m_turnControllerAuto =
      new PIDController(SwerveConstants.kP_Rot, SwerveConstants.kI_Rot,SwerveConstants.kD_Rot);

  //Simulated Yaw, Only used in Sim
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

    //Chassis Speed
    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                drive, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(drive, strafe, rotation);

    //Module States
    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }
  public void slowDrive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
        // rotation *= SwerveConstants.kMaxRotationRadiansPerSecond;


    //Chassis Speed
    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                drive, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(drive, strafe, rotation);

    //Module States
    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }
/**
 * Sets States For Swerve Modules and Determines FeedbackLoop Type
 * 
 * @param states
 * @param isOpenLoop
 */
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  /**
   * Sets Odometry, Used in Auto
   * @param pose 
   */
  public void setOdometry(Pose2d pose) {
    m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    m_gyro.setYaw(pose.getRotation().getDegrees());
  }
  public void resetGyro() {
    m_gyro.setYaw(0);
    m_gyro.setAccumZAngle(0);
  }
   /**
   * Gets Drive Heading
   * @return Adjusted Gyro Heading
   */
  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_gyro.getYaw(), 360);
  }
  /**
   * Gets {@link Rotation2d} from Heading
   * @return {@link Rotation2d} from Drive Heading
   */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }
/**
   * Gets Robot Position on the Field in Meters
   * @return Position of the Robot in XY Coordinates 
   */
  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }
/**
   * Gets Swerve Module From Module Position
   * @param {@link ModulePosition}
   * @return Swerve Module
   */
  public SwerveModule getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }
/**
   * Gets Module States
   * @return {@link Map Map ofModule Position and SwerveModuleState}
   */
  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : m_swerveModules.keySet()) {
      map.put(i, m_swerveModules.get(i).getState());
    }
    return map;
  }
/**
   * Gets Module Positions
   * @return {@link SwerveModulePosition SwerveModulePosition Array}
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

// Returns PID Controllers, Used in Auto

  public PIDController getXPidController() {
    return m_XController;
  }

  public PIDController getYPidController() {
    return m_YController;
  }

  public ProfiledPIDController getRotPidController() {
    return m_turnController;
  }

  public PIDController getRotPidControllerAuto() {
    return m_turnControllerAuto;
  }

  public void setNeutralMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }
/**
 * Gets Drive Odometry
 * @return {@link SwerveDriveOdometry}
 */
  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }
/**
 * Updates Drive Odometry
 * 
 */
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
/**
 * Runs Periodically after Init
 * 
 * 
 * 
 */
  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }
 /**
 * Runs Periodically during Simulation
 * 
 * 
//  * 
//  */
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
