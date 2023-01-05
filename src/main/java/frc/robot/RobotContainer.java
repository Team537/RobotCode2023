// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroPID;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.kGains;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.toggleFastMode;
import frc.robot.commands.toggleSlowMode; 

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.SwingConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  Camera camera = new Camera();
  

  // SlewRateLimiter for Joystick Motion Profiling

  private final SlewRateLimiter Left = new SlewRateLimiter(3);
  private final SlewRateLimiter Right = new SlewRateLimiter(3);

 //Drive PID Controllers

  PIDController forwardController = new PIDController(kGains.kP, kGains.kI, kGains.kD);
  PIDController turnController = new PIDController(GyroPID.kP, GyroPID.kI, GyroPID.kD);
  
// The driver controllers

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort);

   // Drive Speeds

   private double forwardSpeed;
   private double turnSpeed;

  //  private double leftSpeed =  -Left.calculate( m_driverController.getLeftY());
  //  private double rightSpeed =  -Right.calculate(m_driverController.getRightY());
    
  public RobotContainer() {

    //Toggle Booleans

    boolean toggleFastMode = true;
    boolean toggleSlowMode = true;
    

    //Joystick Buttons

    JoystickButton startButton = new JoystickButton(m_driverController, Button.kStart.value);
    JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);
    

   //Button Bindings

    if(toggleFastMode = true){
        startButton.onTrue
        (new toggleFastMode(m_robotDrive));
    }

    if(toggleSlowMode = true){
        backButton.onTrue
        (new toggleSlowMode(m_robotDrive));
    }

    
    if (m_driverController.getAButton()) {
      
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        
        double range = PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.CAMERA_HEIGHT_METERS, 
          VisionConstants.TARGET_HEIGHT_METERS, 
          VisionConstants.CAMERA_PITCH_RADIANS,  
          Units.degreesToRadians(result.getBestTarget().getPitch()));

          forwardSpeed = -forwardController.calculate(range, VisionConstants.GOAL_RANGE_METERS);

          turnSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

      } else {
          forwardSpeed = 0;
          turnSpeed = 0;
          
      }  

        
      
  }

  forwardSpeed = -Left.calculate( m_driverController.getLeftY());
  turnSpeed = -Right.calculate(m_driverController.getRightX());

   //Drive Commands

    m_robotDrive.setDefaultCommand(
      // new RunCommand(() ->
      //     m_robotDrive.tankDrive(
      //       leftSpeed,
      //        rightSpeed),
      //         m_robotDrive)
      
      new ArcadeDriveCommand(
        m_robotDrive,
        () -> forwardSpeed,
        () -> turnSpeed)
);
      

//Choose Which Drive Based on what is chosen by Drivers
    
  }
  public Command getAutonomousCommand() {
    // Voltage Restricition to Prevent Motor from Blowing itself
  
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);
  
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
  
    Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              // Pass config
              config);
  
  
    RamseteCommand ramseteCommand =
          new RamseteCommand(
              exampleTrajectory,
              m_robotDrive::getPose,
              new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
              new SimpleMotorFeedforward(
                  DriveConstants.ksVolts,
                  DriveConstants.kvVoltSecondsPerMeter,
                  DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics,
              m_robotDrive::getWheelSpeeds,
              new PIDController(DriveConstants.kPDriveVel, 0, 0),
              new PIDController(DriveConstants.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              m_robotDrive::tankDriveVolts,
              m_robotDrive);
  
  
              // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
  
  
      return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  
  }
}