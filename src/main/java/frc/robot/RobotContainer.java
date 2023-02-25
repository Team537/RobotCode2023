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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ExampleTrajectory;
import frc.robot.commands.SlowSwerveDriveCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.simulation.FieldSim;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmInOut;
import frc.robot.subsystems.ArmPivot;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperIntake;
import frc.robot.subsystems.Wrist;
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
// import frc.robot.commands.ArcadeDriveCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;


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
  private final LED LEDAllianceStart = new LED();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final GripperIntake m_Gripper = new GripperIntake();
  private final ArmInOut m_ArmInOut = new ArmInOut();
  private final ArmPivot m_ArmPivot = new ArmPivot();
  private final Wrist m_Wrist = new Wrist(); 
  PhotonCamera camera = new PhotonCamera("USB Camera 0");
  private FieldSim m_FieldSim = new FieldSim(m_robotDrive);
  
  private final Camera m_camera = new Camera();
  

  // SlewRateLimiter for Joystick Motion Profiling

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(0.1);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(0.1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(30);

 
 
  
// The driver controllers

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort1);
  


  JoystickButton starButton = new JoystickButton(m_driverController, Button.kStart.value);
  JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);

   // Drive Speeds
  
  


   JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
   JoystickButton rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);

  //  private double leftSpeed =  -Left.calculate( m_driverController.getLeftY());
  //  private double rightSpeed =  -Right.calculate(m_driverController.getRightY());

  //Buttons
  JoystickButton aButton = new JoystickButton(m_driverController, Button.kA.value);
  JoystickButton bButton = new JoystickButton(m_driverController, Button.kB.value);

  JoystickButton yButton = new JoystickButton(m_driverController, Button.kY.value);
  JoystickButton xButton = new JoystickButton(m_driverController, Button.kX.value);

  POVButton dPadUpButton = new POVButton(m_driverController, 0);
  POVButton dPadDownButton = new POVButton(m_driverController, 180);
  POVButton dPadLeftButton = new POVButton(m_driverController, 90);
  POVButton dPadRightButton = new POVButton(m_driverController, 270);
  
    
  public RobotContainer() {

    dPadLeftButton.onTrue(new StartEndCommand(m_ArmInOut::armIn,m_ArmInOut::armOut,m_ArmInOut));
    dPadRightButton.onTrue(new StartEndCommand(m_ArmInOut::armOut,m_ArmInOut::armIn,m_ArmInOut));
    // dPadUpButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementUp, m_ArmInOut::armIncrementDown, m_ArmInOut));
    // dPadDownButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementDown, m_ArmInOut::armIncrementUp, m_ArmInOut));

    /*^^ for incrementing the position of the arm in-out */

    yButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPosition1,m_ArmPivot::ArmPosition2,m_ArmPivot));
    xButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPosition2,m_ArmPivot::ArmPosition1,m_ArmPivot));
    backButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPosition3,m_ArmPivot::ArmPosition1,m_ArmPivot));

    dPadDownButton.onTrue(new StartEndCommand(m_Wrist::WristPosition2,m_Wrist::WristPosition1,m_Wrist));
    dPadUpButton.onTrue(new StartEndCommand(m_Wrist::WristPosition1,m_Wrist::WristPosition2,m_Wrist));
    leftBumper.onTrue(new StartEndCommand(m_Wrist::WristPosition3,m_Wrist::WristPosition1,m_Wrist));

    aButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperIn,m_Gripper::GripperStop,m_Gripper));
    bButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperOut,m_Gripper::GripperStop,m_Gripper));
    
     final ChaseTagCommand chaseTagCommand = 
    new ChaseTagCommand(m_camera, m_robotDrive, m_camera :: getRobotPose2d);

      // leftBumper.toggleOnTrue(new StartEndCommand(m_camera::CameraToLimelight,m_camera::CameraPipeline,m_camera));
      // rightBumper.toggleOnTrue(new StartEndCommand(m_camera::CameraToAprilTag,m_camera::CameraPipeline,m_camera));
    //Toggle Booleans

    
  
 starButton.toggleOnTrue(new SlowSwerveDriveCommand(
  m_robotDrive,
  ()-> -(m_driverController.getLeftY()),
  ()->  m_driverController.getLeftX(),
  ()->  -m_driverController.getRightX()*0.7,
  false));
    

  //Drive without Slew
//  m_robotDrive.setDefaultCommand( 
//     new SwerveDriveCommand(
//       m_robotDrive,
//       ()-> -m_driverController.getLeftY(),
//       ()->  m_driverController.getLeftX(),
//       ()->  -m_driverController.getRightX()*0.7,
//       true)); 
  
    //Drive with Slew
    m_robotDrive.setDefaultCommand( 
      new SwerveDriveCommand(
        m_robotDrive,
        ()-> -m_ySpeedLimiter.calculate(m_driverController.getLeftY()),
        ()->  m_xSpeedLimiter.calculate(m_driverController.getLeftX()),
        ()->  -m_driverController.getRightX()*0.7,
        true));



      m_FieldSim.initSim();
  
  
  
   
  }

  private double time = 0;

  public void periodic() {
    m_FieldSim.periodic();
    // Already runs ever 5 miliseconds:
    // Add to a variable when joystick power > 0
    // Reset it to 0 when joystick = 0
    // dont let it go over 1
    SmartDashboard.putNumber("Left Joystick",m_driverController.getLeftY());
  }

  public void robotInit() {
    m_robotDrive.resetEncoders();
  }

  public Command getAutoCommand() {

   return new ExampleTrajectory(m_robotDrive, m_FieldSim);
  }


}
 