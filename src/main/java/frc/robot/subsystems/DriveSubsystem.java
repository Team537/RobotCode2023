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
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
  

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
                      SwerveConstants.kFrontLeftSRXMagCoder,
                      SwerveConstants.kFrontLeftSRXMagCoderOffset, false),
              ModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      ModulePosition.FRONT_RIGHT,
                      new WPI_TalonFX(SwerveConstants.kFrontRightTurn),
                      new WPI_TalonFX(SwerveConstants.kFrontRightDrive),
                      SwerveConstants.kFrontRightSRXMagCoder,
                      SwerveConstants.kFrontRightSRXMagCoderOffset, true),
              ModulePosition.BACK_LEFT,
                  new SwerveModule(
                      ModulePosition.BACK_LEFT,
                      new WPI_TalonFX(SwerveConstants.kBackLeftTurn),
                      new WPI_TalonFX(SwerveConstants.kBackLeftDrive),
                      SwerveConstants.kBackLeftSRXMagCoder,
                     SwerveConstants.kBackLeftSRXMagCoderOffset, true),
              ModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      ModulePosition.BACK_RIGHT,
                      new WPI_TalonFX(SwerveConstants.kBackRightTurn),
                      new WPI_TalonFX(SwerveConstants.kBackRightDrive),
                      SwerveConstants.kBackRightSRXMagCoder,
                      SwerveConstants.kBackRightSRXMagCoderOffset, false)));


  //Gyro and Simulated Gyro  
                    
  private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kPigeonID);
  public String driveState = "Drive";

  //old gyro
//   private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);



  //Swerve Odometry 

  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
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
    m_gyro.setYaw(180);
  
    // m_gyro.configMountPose(-90, -0.219727 , 0.615234);
  
    
    
    
  }

  public void drive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
        // d = d * s
    drive *= SwerveConstants.kMaxSpeedMetersPerSecond;
    strafe *= SwerveConstants.kMaxSpeedMetersPerSecond;
    rotation *= SwerveConstants.kMaxRotationRadiansPerSecond;

    //Chassis Speed
    ChassisSpeeds chassisSpeeds =
        isFieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                drive, strafe, rotation, getHeadingRotation2d())
                : new ChassisSpeeds(drive, strafe, rotation);
            //  new ChassisSpeeds(drive*Math.cos(Math.toRadians(m_gyro.getYaw())) + strafe*Math.sin(Math.toRadians(m_gyro.getYaw())),
            //  -drive*Math.sin(Math.toRadians(m_gyro.getYaw())) + strafe*Math.cos(Math.toRadians(m_gyro.getYaw())), 
            //  rotation);

    //Module States
    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
      driveState = "Drive";
  }
  public void slowDrive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
        rotation *= SwerveConstants.kSlowRotationRadiansPerSecond;


    //Chassis Speed
    ChassisSpeeds chassisSpeeds =
        isFieldRelative ?
             ChassisSpeeds.fromFieldRelativeSpeeds(
                drive, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(drive, strafe, rotation);

    //Module States
    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), SwerveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setSlowDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);

      driveState = "Slow Drive";
  }
  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
    
         return new PPSwerveControllerCommand(
             traj, 
             this::getPoseMeters, // Pose supplier
             SwerveConstants.kDriveKinematics, // SwerveDriveKinematics
             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.

             //refers to the class its in, calls the swerve module
             this::setSwerveModuleStatesAuto, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         );
     
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

   // calls from the setswerveodometry command
   //only used in auto
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
    return Math.IEEEremainder(-(m_gyro.getYaw()), 360);
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }
  
  public double getPitch() {
    return m_gyro.getPitch();
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
    return m_odometry.getEstimatedPosition();
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

  public void setBrakeMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }
/**
 * Gets Drive Odometry
 * @return {@link SwerveDriveOdometry}
 */
  public SwerveDrivePoseEstimator getOdometry() {
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
  // public void setLeds(LED leds) {
  //   this.m_LED = leds;
  // }
  private void updateSmartDashboard() {

    SmartDashboard.putNumber("Gyro Angle", getHeadingDegrees());
    SmartDashboard.putString("Drive State", driveState);
  }
/**s
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

    // m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.06; //changed from .02

    Unmanaged.feedEnable(2);
    m_gyro.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }


  // public void resetEncoders(){

  //   for (SwerveModule module : m_swerveModules.values()) {
  //     module.resetAngleToAbsolute();;
  //   }

  // }

  public double getVelocity(){

    return m_swerveModules.get(ModulePosition.FRONT_LEFT).getDriveMetersPerSecond();
  }

  public void teleOpGyroReset(){

    double gyroAngle = m_gyro.getYaw();

    m_gyro.setYaw(180);

  }

}
