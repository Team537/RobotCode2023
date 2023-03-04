// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto.FollowTrajectory;
import frc.robot.commands.gripper.GripperIn;
import frc.robot.commands.gripper.GripperOut;
import frc.robot.commands.manipulator.ManipulatorHighGoal;
import frc.robot.commands.manipulator.ManipulatorGround;
import frc.robot.commands.manipulator.ManipulatorMidGoal;
import frc.robot.commands.manipulator.ManipulatorShelfHumanPL;
import frc.robot.commands.manipulator.ManipulatorZero;
import frc.robot.commands.signal.SignalCone;
import frc.robot.commands.signal.SignalCube;
import frc.robot.commands.swerve.SlowSwerveDriveCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.vision.ChaseTagCommand;
import frc.robot.commands.wrist.WristManualUp;
import frc.robot.simulation.FieldSim;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.GripperCamera;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperIntake;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.subsystems.Camera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.ArcadeDriveCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public static final LED m_LED = new LED();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final GripperIntake m_Gripper = new GripperIntake();
  // private final GripperCamera m_GripperCamera = new GripperCamera();
  private final ArmInOut m_ArmInOut = new ArmInOut();
  private final ArmPivot m_ArmPivot = new ArmPivot();
  private final Wrist m_Wrist = new Wrist(); 
    // private PhotonCamera camera = new PhotonCamera("USB Camera 0");
  private FieldSim m_FieldSim = new FieldSim(m_robotDrive);
  private SendableChooser<Command> m_Chooser = new SendableChooser<Command>();
  
  private final Camera m_camera = new Camera(m_robotDrive);


  
  Command high_goal = new ManipulatorHighGoal(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED) ;
  Command mid_goal = new ManipulatorMidGoal(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);
  Command ground = new ManipulatorGround(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);
  Command shelf_HuPL =  new ManipulatorShelfHumanPL(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);
  Command wrist_manualup = new WristManualUp(m_Wrist);
  Command zeros = new ManipulatorZero(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);
  Command gripperIn = new GripperIn(m_Gripper, m_LED);
  Command gripperOut = new GripperOut(m_Gripper, m_LED);
  Command signalCube = new SignalCube(m_LED);
  Command signalCone = new SignalCone(m_LED);

  // SlewRateLimiter for Joystick Motion Profiling

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(0.1);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(0.1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(30);

 
 
  
// The driver controllers

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort1);
  
  JoystickButton starButton = new JoystickButton(m_driverController, Button.kStart.value);
  JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);
  JoystickButton leftStick = new JoystickButton(m_driverController, Button.kBack.value);
  JoystickButton rightStick = new JoystickButton(m_driverController, Button.kRightStick.value);
  JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  JoystickButton rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  JoystickButton aButton = new JoystickButton(m_driverController, Button.kA.value);
  JoystickButton bButton = new JoystickButton(m_driverController, Button.kB.value);
  JoystickButton yButton = new JoystickButton(m_driverController, Button.kY.value);
  JoystickButton xButton = new JoystickButton(m_driverController, Button.kX.value);
  POVButton dPadUpButton = new POVButton(m_driverController, 0);
  POVButton dPadDownButton = new POVButton(m_driverController, 180);
  POVButton dPadLeftButton = new POVButton(m_driverController, 90);
  POVButton dPadRightButton = new POVButton(m_driverController, 270);

  Trigger leftTrigger = m_driverController
    .leftTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop())
    .castTo(Trigger::new);

  Trigger rightTrigger = m_driverController
    .rightTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop())
    .castTo(Trigger::new);

  public RobotContainer() {


/*  // NON LED COMMANDS
    //This is all commented out because we use a single button to order multiple commands, using commands made
    //for each part of the manipulator. they were then merged into multiple action commands, which send multiple
    //objects to multiple positions with one button press
     dPadLeftButton.onTrue(new StartEndCommand(m_ArmInOut::armGround,m_ArmInOut::armLowGoal,m_ArmInOut));
    dPadRightButton.onTrue(new StartEndCommand(m_ArmInOut::armZero,m_ArmInOut::armMidGoal,m_ArmInOut));
    leftBumper.onTrue(new StartEndCommand(m_ArmInOut::armTest,m_ArmInOut::armMidGoal,m_ArmInOut));

      // // dPadUpButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementUp, m_ArmInOut::armIncrementDown, m_ArmInOut));
      // // dPadDownButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementDown, m_ArmInOut::armIncrementUp, m_ArmInOut));
      //^^ for incrementing the position of the arm in-out

    yButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPositionZero,m_ArmPivot::ArmPositionHighGoal,m_ArmPivot));
    xButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPositionGround,m_ArmPivot::ArmPositionLowGoal,m_ArmPivot));
    backButton.onTrue(new StartEndCommand(m_ArmPivot::ArmPositionTest,m_ArmPivot::ArmPositionLowGoal,m_ArmPivot));

    dPadDownButton.onTrue(new StartEndCommand(m_Wrist::WristPositionTest,m_Wrist::WristPositionHighGoal,m_Wrist));
    dPadUpButton.onTrue(new StartEndCommand(m_Wrist::WristPositionGround,m_Wrist::WristPositionMidGoal,m_Wrist));
    rightBumper.onTrue(new StartEndCommand(m_Wrist::WristPositionZero,m_Wrist::WristPositionHighGoal,m_Wrist));

    aButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperIn,m_Gripper::GripperStop,m_Gripper));
    bButton.toggleOnTrue(new StartEndCommand(m_Gripper::GripperOut,m_Gripper::GripperStop,m_Gripper));

*/
    //  LED COMMANDS
    yButton.onTrue(high_goal);
    xButton.onTrue(mid_goal);

    aButton.onTrue(ground);
    bButton.onTrue(shelf_HuPL);

    leftBumper.onTrue(gripperIn);
    rightBumper.onTrue(gripperOut);

    

    dPadUpButton.onTrue(wrist_manualup);
    dPadDownButton.onTrue(zeros);

    // dPadLeftButton.toggleOnTrue(WristDownManual);
    // dPadRightButton.toggleOnTrue(WristUpManual);

    // dPadUpButton.toggleOnTrue(ArmPivotUpManual);
    // dPadDownButton.toggleOnTrue(ArmPivotDownManual);

    //  dPadLeftButton.toggleOnTrue(signalCone);
    //  dPadRightButton.toggleOnTrue(signalCube);

    
     final ChaseTagCommand chaseTagCommand = 
    new ChaseTagCommand(m_camera, m_robotDrive, m_camera :: getRobotPose2d);

    leftStick.toggleOnTrue(new StartEndCommand(m_camera::CameraToLimelight,m_camera::CameraPipeline,m_camera));
    rightStick.toggleOnTrue(new StartEndCommand(m_camera::CameraToAprilTag,m_camera::CameraPipeline,m_camera));
    //Toggle Booleans
    // LED Light Trigger COntrol Code
    // aButton.toggleOnTrue(new StartEndCommand(m_LED::setBlue,m_LED::setGreen,m_LED));
    // bButton.toggleOnTrue(new StartEndCommand(m_LED::setGreen,m_LED::setBlue,m_LED));
    
  
  // m_robotDrive.setLeds(m_LED);
  // m_Manipulator.setLeds(m_LED);
 starButton.toggleOnTrue(new SlowSwerveDriveCommand(
  m_robotDrive,
  ()-> -(m_driverController.getLeftY()),
  ()->  m_driverController.getLeftX(),
  ()->  -m_driverController.getRightX()*0.7,
  true, m_LED));
    

  // Drive without Slew
 m_robotDrive.setDefaultCommand( 
    new SwerveDriveCommand(
      m_robotDrive,
      ()-> -m_driverController.getLeftY(),
      ()->  m_driverController.getLeftX(),
      ()->  -m_driverController.getRightX()*0.2,
      true, m_LED)); 
  
    //Drive with Slew
    // m_robotDrive.setDefaultCommand( 
    //   new SwerveDriveCommand(
    //     m_robotDrive,
    //     ()-> -m_ySpeedLimiter.calculate(m_driverController.getLeftY()),
    //     ()->  m_xSpeedLimiter.calculate(m_driverController.getLeftX()),
    //     ()->  -m_driverController.getRightX()*0.7,
    //     true));



      m_FieldSim.initSim();
  
  
  
   
  }

  public void periodic() {
    m_FieldSim.periodic();
    m_LED.update();
    // Already runs ever 5 miliseconds: 
    // Add to a variable when joystick power > 0
    // Reset it to 0 when joystick = 0
    // dont let it go over 1
    SmartDashboard.putNumber("Left Joystick",m_driverController.getLeftY());
  }

  public void robotInit() {
    // m_robotDrive.setOdometry(new Pose2d(3.67,1.30,new Rotation2d()));

     m_Chooser.addOption("Auto 1", new FollowTrajectory(m_robotDrive, m_FieldSim, "Blue Auto 1"));
     m_Chooser.addOption("Auto 2", new FollowTrajectory(m_robotDrive, m_FieldSim, "Blue Auto 2"));
     m_Chooser.addOption("Auto 3", new FollowTrajectory(m_robotDrive, m_FieldSim, "Blue Auto 3"));

     SmartDashboard.putData("Auto Selector", m_Chooser);
  }

  public Command getAutoCommand() {

   return m_Chooser.getSelected();
  }


}
 