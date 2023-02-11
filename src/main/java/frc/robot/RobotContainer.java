// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroPID;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.kGains;
import frc.robot.commands.ExampleTrajectory;
import frc.robot.commands.SlowSwerveDriveCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.simulation.FieldSim;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperIntake;
import frc.robot.Constants.limelight;
import frc.robot.subsystems.Camera;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import java.io.IOException;   
import java.nio.file.Path;
import java.util.List;

import javax.print.attribute.standard.JobHoldUntil;
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
  private final GripperIntake m_Gripper = new GripperIntake();
  PhotonCamera camera = new PhotonCamera("USB Camera 0");
  private FieldSim m_FieldSim = new FieldSim(m_robotDrive);
  
  private final Camera m_camera = new Camera();
  

  // SlewRateLimiter for Joystick Motion Profiling

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

 
 
 
  
// The driver controllers

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort1);
  


  JoystickButton starButton = new JoystickButton(m_driverController, Button.kStart.value);

   // Drive Speeds
  
  


   JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
   JoystickButton rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

  //  private double leftSpeed =  -Left.calculate( m_driverController.getLeftY());
  //  private double rightSpeed =  -Right.calculate(m_driverController.getRightY());

  //Buttons
  JoystickButton aButton = new JoystickButton(m_driverController2, Button.kA.value);
  JoystickButton bButton = new JoystickButton(m_driverController2, Button.kB.value);
    
  public RobotContainer() {


    aButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperIn,m_Gripper::GripperStop,m_Gripper));
    bButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperOut,m_Gripper::GripperStop,m_Gripper));
    
      leftBumper.toggleOnTrue(new StartEndCommand(m_camera::CameraToLimelight,m_camera::CameraPipeline,m_camera));
      rightBumper.toggleOnTrue(new StartEndCommand(m_camera::CameraToAprilTag,m_camera::CameraPipeline,m_camera));
    //Toggle Booleans

    
  
 starButton.toggleOnTrue(new SlowSwerveDriveCommand(
  m_robotDrive,
  ()-> -m_driverController.getLeftY(),
  ()->  m_driverController.getLeftX(),
  ()->  -m_driverController.getRightX()*0.7,
  false));
    


  m_robotDrive.setDefaultCommand( 
    new SwerveDriveCommand(
      m_robotDrive,
      ()-> -m_driverController.getLeftY(),
      ()->  m_driverController.getLeftX(),
      ()->  -m_driverController.getRightX()*0.7,
      true));
  
  
      m_FieldSim.initSim();
  
  
  
   
  }

  public void periodic() {
    m_FieldSim.periodic();
  }

  // public void robotInit() {
  //   m_robotDrive.resetEncoders();
  // }

  public Command getAutoCommand() {

   return new ExampleTrajectory(m_robotDrive, m_FieldSim);
  }


}
 