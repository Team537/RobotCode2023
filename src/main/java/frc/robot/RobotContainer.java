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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.camera;
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
  private final camera m_camera = new camera();
  // SlewRateLimiter for Joystick Motion Profiling
  private final SlewRateLimiter Left = new SlewRateLimiter(3);
  private final SlewRateLimiter Right = new SlewRateLimiter(3);
  //Toggle Booleans
  
  // The driver controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort);

     
    
  public RobotContainer() {
    boolean toggleFastMode = true;
    boolean toggleSlowMode = true;
    JoystickButton startButton = new JoystickButton(m_driverController, Button.kStart.value);
    JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);

    if(toggleFastMode = true){
        startButton.onTrue
        (new toggleFastMode(m_robotDrive));
    }

    if(toggleSlowMode = true){
        backButton.onTrue
        (new toggleSlowMode(m_robotDrive));
    }




   

    m_robotDrive.setDefaultCommand(
      new RunCommand(() ->
          m_robotDrive.tankDrive(
             -Left.calculate( m_driverController.getLeftY()),
             -Right.calculate(m_driverController.getRightY())),
              m_robotDrive)
      
      // new ArcadeDriveCommand(
      //   m_robotDrive,
      //   () -> -Left.calculate( m_driverController.getLeftY()),
      //   () -> -Right.calculate(m_driverController.getRightY()))
);
      

//Choose Which Drive Based on what is chosen by Drivers
      
    
   
  }
}