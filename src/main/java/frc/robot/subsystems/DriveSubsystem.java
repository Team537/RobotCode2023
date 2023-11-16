// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class DriveSubsystem extends SubsystemBase {

  // Hash Map for All Swerve Modules

  private final HashMap<ModulePosition, SwerveModule> swerveModules = new HashMap<>(
      Map.of(
          ModulePosition.FRONT_LEFT,
          new SwerveModule(
              ModulePosition.FRONT_LEFT,
              new WPI_TalonFX(SwerveConstants.FRONT_LEFT_TURN_MOTOR_ID),
              new WPI_TalonFX(SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID),
              SwerveConstants.FRONT_LEFT_SRX_MAG_ENCODER_ID,
              SwerveConstants.FRONT_LEFT_SRX_MAG_ENCODER_OFFSET, false),

          ModulePosition.FRONT_RIGHT,
          new SwerveModule(
              ModulePosition.FRONT_RIGHT,
              new WPI_TalonFX(SwerveConstants.FRONT_RIGHT_TURN_MOTOR_ID),
              new WPI_TalonFX(SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID),
              SwerveConstants.FRONT_RIGHT_SRX_MAG_ENCODER_ID,
              SwerveConstants.FRONT_RIGHT_SRX_MAG_ENCODER_OFFSET, true),

          ModulePosition.BACK_LEFT,
          new SwerveModule(
              ModulePosition.BACK_LEFT,
              new WPI_TalonFX(SwerveConstants.BACK_LEFT_TURN_MOTOR_ID),
              new WPI_TalonFX(SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID),
              SwerveConstants.BACK_LEFT_SRX_MAG_ENCODER_ID,
              SwerveConstants.BACK_LEFT_SRX_MAG_ENCODER_OFFSET, true),

          ModulePosition.BACK_RIGHT,
          new SwerveModule(
              ModulePosition.BACK_RIGHT,
              new WPI_TalonFX(SwerveConstants.BACK_RIGHT_TURN_MOTOR_ID),
              new WPI_TalonFX(SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID),
              SwerveConstants.BACK_RIGHT_SRX_MAG_ENCODER_ID,
              SwerveConstants.BACK_RIGHT_SRX_MAG_ENCODER_OFFSET, false)));

  // Gyro and Simulated Gyro

  private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
  public String driveState = "Drive";

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.SWERVE_KINEMATICS,
      getHeadingRotation2d(),
      getModulePositions(),
      new Pose2d());

  private PIDController xPIDController = new PIDController(SwerveConstants.P_X, SwerveConstants.I_X,
      SwerveConstants.D_X);
  private PIDController yPIDController = new PIDController(SwerveConstants.P_Y, SwerveConstants.I_Y,
      SwerveConstants.D_Y);
  private ProfiledPIDController profiledTurnController = new ProfiledPIDController(SwerveConstants.P_ROTATION,
      SwerveConstants.I_ROTATION, SwerveConstants.D_ROTATION, SwerveConstants.ROTATION_CONTROLLER_CONSTRAINTS);
  private PIDController turnController = new PIDController(SwerveConstants.P_ROTATION, SwerveConstants.I_ROTATION,
      SwerveConstants.D_ROTATION);

  private double simYaw;

  public DriveSubsystem() {
    // pigeon.setYaw(180);

  }

  public void drive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative) {
    drive *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    strafe *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    rotation *= SwerveConstants.MAX_ROTATION_RADIANS_PER_SECOND;

    // Chassis Speed
    ChassisSpeeds chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        drive, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(drive, strafe, rotation);

    // Module States
    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
        .of(SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));

    setSwerveModuleStatesMap(moduleStates);

    driveState = "Drive";
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {

    return new PPSwerveControllerCommand(
        traj,
        this::getPoseMeters,
        SwerveConstants.SWERVE_KINEMATICS,
        getXPidController(),
        getYPidController(),
        getRotPidControllerAuto(),
        this::setSwerveModuleStates,
        true,
        this);

  }

  public double[] getGyroVelocityXYZ() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz;
  }

  public double getGyroPitch() {

    return pigeon.getPitch();

  }

  public void setDiamondShape() {

    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap.of(new SwerveModuleState[] {

        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),

        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),

        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),

        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });

    setSwerveModuleStatesMap(moduleStates);

  }

  public void stop() {
    this.drive(0, 0, 0, true);
  }

  /**
   * Sets States For Swerve Modules and Determines FeedbackLoop Type
   * 
   * @param states
   * @param isOpenLoop
   */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()]);
  }

  public void setSwerveModuleStatesMap(Map<ModulePosition, SwerveModuleState> moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()));

  }

  /**
   * Sets Odometry, Used in Auto
   * 
   * @param pose
   */

  public void setOdometry(Pose2d pose) {
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    pigeon.setYaw(pose.getRotation().getDegrees());
  }

  public void resetGyro() {
    pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
  }

  /**
   * Gets Drive Heading
   * 
   * @return Adjusted Gyro Heading
   */
  public double getHeadingDegrees() {
    return Math.IEEEremainder(-(pigeon.getYaw()), 360);
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  /**
   * Gets {@link Rotation2d} from Heading
   * 
   * @return {@link Rotation2d} from Drive Heading
   */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Gets Robot Position on the Field in Meters
   * 
   * @return Position of the Robot in XY Coordinates
   */
  public Pose2d getPoseMeters() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Gets Swerve Module From Module Position
   * 
   * @param {@link ModulePosition}
   * @return Swerve Module
   */
  public SwerveModule getSwerveModule(ModulePosition modulePosition) {
    return swerveModules.get(modulePosition);
  }

  /**
   * Gets Module States
   * 
   * @return {@link Map Map ofModule Position and SwerveModuleState}
   */
  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : swerveModules.keySet()) {
      map.put(i, swerveModules.get(i).getState());
    }
    return map;
  }

  /**
   * Gets Module Positions
   * 
   * @return {@link SwerveModulePosition SwerveModulePosition Array}
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public PIDController getXPidController() {
    return xPIDController;
  }

  public PIDController getYPidController() {
    return yPIDController;
  }

  public ProfiledPIDController getRotPidController() {
    return profiledTurnController;
  }

  public PIDController getRotPidControllerAuto() {
    return turnController;
  }

  public void setBrakeMode(NeutralMode mode) {
    for (SwerveModule module : swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  /**
   * Gets Drive Odometry
   * 
   * @return {@link SwerveDriveOdometry}
   */
  public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
  }

  /**
   * Updates Drive Odometry
   * 
   */
  public void updateOdometry() {
    odometry.update(
        getHeadingRotation2d(),
        getModulePositions());

    for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules)) {
      Translation2d modulePositionFromChassis = SwerveConstants.MODULE_TRANSLATIONS
          .get(module.getModulePosition())
          .rotateBy(getHeadingRotation2d())
          .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  // public void setLeds(LED leds) {
  // this.m_LED = leds;
  // }
  private void updateSmartDashboard() {

    SmartDashboard.putNumber("Gyro Angle", getHeadingDegrees());
    SmartDashboard.putString("Drive State", driveState);
    SmartDashboard.putNumber("Gyro Pitch", getGyroPitch());
    SmartDashboard.putNumber("Odo X", odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Odo Y", odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odo Yaw", odometry.getEstimatedPosition().getRotation().getDegrees());
  }

  /**
   * s
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
   * // *
   * //
   */
  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = SwerveConstants.SWERVE_KINEMATICS.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02; // changed from .02

    Unmanaged.feedEnable(20);
    pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(simYaw));
  }

  public void resetEncoders() {

    for (SwerveModule module : swerveModules.values()) {
      module.resetAngleToAbsolute();
      ;
    }

  }

  public double getVelocity() {

    return swerveModules.get(ModulePosition.FRONT_LEFT).getDriveMetersPerSecond();
  }

  public void setCoast() {
    for (SwerveModule module : swerveModules.values()) {
      module.setDriveNeutralMode(NeutralMode.Coast);

    }

  }

}
