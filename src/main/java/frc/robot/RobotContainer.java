// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto.BalanceChargeStation;
import frc.robot.commands.Auto.FollowTrajectory;
import frc.robot.commands.Auto.ScoreHighCubeBalance;
import frc.robot.commands.Auto.ScoreHighCubeDriveBack;
import frc.robot.commands.Auto.ScoreMidDriveBack;
import frc.robot.commands.Auto.ScoreMidNoDrive;
import frc.robot.commands.Auto.ScoreHighCubeNoDrive;
import frc.robot.commands.Auto.ScoreMidBalance;
import frc.robot.commands.led.LedHighGoal;
import frc.robot.commands.led.LedLowGoal;
import frc.robot.commands.led.LedMidGoal;
import frc.robot.commands.led.LedShelf;
import frc.robot.commands.manipulator.ManipulatorHighGoal;
import frc.robot.commands.manipulator.ManipulatorGround;
import frc.robot.commands.manipulator.ManipulatorMidGoal;
import frc.robot.commands.manipulator.ManipulatorShelfHumanPL;
import frc.robot.commands.manipulator.ManipulatorZero;
// import frc.robot.commands.signal.SignalCone;
// import frc.robot.commands.signal.SignalCube;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.commands.swerve.SetSwerveBrakeMode;
// import frc.robot.commands.swerve.BoostDriveCommand;
import frc.robot.commands.swerve.SlowSwerveDriveCommand;
import frc.robot.commands.vision.ChaseTagCommand;
import frc.robot.simulation.FieldSim;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperCamera;
// import frc.robot.subsystems.GripperCamera;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperIntake;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;
import frc.robot.subsystems.Camera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.ArcadeDriveCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ConstantsFactory;
import frc.robot.config.Constants;

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
    private final GripperCamera m_GripperCamera = new GripperCamera();
    private final ArmInOut m_ArmInOut = new ArmInOut();
    private final ArmPivot m_ArmPivot = new ArmPivot();
    private final Wrist m_Wrist = new Wrist();
    // private PhotonCamera camera = new PhotonCamera("USB Camera 0");
    private FieldSim m_FieldSim = new FieldSim(m_robotDrive);
    private SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

    Command high_goal = new ParallelCommandGroup(new LedHighGoal(m_LED),
            new ManipulatorHighGoal(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED));
    Command mid_goal = new ParallelCommandGroup(new LedMidGoal(m_LED),
            new ManipulatorMidGoal(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED));
    Command ground = new ParallelCommandGroup(new LedLowGoal(m_LED),
            new ManipulatorGround(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED));
    Command shelf_HuPL = new ParallelCommandGroup(new LedShelf(m_LED),
            new ManipulatorShelfHumanPL(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED));
    Command zeros = new ManipulatorZero(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);

    Command scoreMidDriveBack = new ScoreMidDriveBack(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot, m_Gripper,
            m_Wrist, m_LED);
    Command scoreMidNoDrive = new ScoreMidNoDrive(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot, m_Gripper,
            m_Wrist,
            m_LED);
    Command scoreHighCubeNoDrive = new ScoreHighCubeNoDrive(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot,
            m_Gripper,
            m_Wrist, m_LED);
    Command scoreHighCubeDriveBack = new ScoreHighCubeDriveBack(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot,
            m_Gripper, m_Wrist, m_LED);
    Command scoreMidBalance = new ScoreMidBalance(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot,
            m_Gripper, m_Wrist, m_LED);

    Command scoreHighCubeBalance = new ScoreHighCubeBalance(m_robotDrive, m_FieldSim, m_ArmInOut, m_ArmPivot, m_Gripper,
            m_Wrist, m_LED);
    // Command signalCube = new SignalCube(m_LED);
    // Command signalCone = new SignalCone(m_LED);

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

    private Constants m_constants;

    public RobotContainer() {

        /*
         * // NON LED COMMANDS
         * //This is all commented out because we use a single button to order multiple
         * commands, using commands made
         * //for each part of the manipulator. they were then merged into multiple
         * action commands, which send multiple
         * //objects to multiple positions with one button press
         * dPadLeftButton.onTrue(new
         * StartEndCommand(m_ArmInOut::armGround,m_ArmInOut::armLowGoal,m_ArmInOut));
         * dPadRightButton.onTrue(new
         * StartEndCommand(m_ArmInOut::armZero,m_ArmInOut::armMidGoal,m_ArmInOut));
         * leftBumper.onTrue(new
         * StartEndCommand(m_ArmInOut::armTest,m_ArmInOut::armMidGoal,m_ArmInOut));
         * 
         * // // dPadUpButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementUp,
         * m_ArmInOut::armIncrementDown, m_ArmInOut));
         * // // dPadDownButton.onTrue(new StartEndCommand(m_ArmInOut::armIncrementDown,
         * m_ArmInOut::armIncrementUp, m_ArmInOut));
         * //^^ for incrementing the position of the arm in-out
         * 
         * yButton.onTrue(new
         * StartEndCommand(m_ArmPivot::ArmPositionZero,m_ArmPivot::ArmPositionHighGoal,
         * m_ArmPivot));
         * xButton.onTrue(new
         * StartEndCommand(m_ArmPivot::ArmPositionGround,m_ArmPivot::ArmPositionLowGoal,
         * m_ArmPivot));
         * backButton.onTrue(new
         * StartEndCommand(m_ArmPivot::ArmPositionTest,m_ArmPivot::ArmPositionLowGoal,
         * m_ArmPivot));
         * 
         * dPadDownButton.onTrue(new
         * StartEndCommand(m_Wrist::WristPositionTest,m_Wrist::WristPositionHighGoal,
         * m_Wrist));
         * dPadUpButton.onTrue(new
         * StartEndCommand(m_Wrist::WristPositionGround,m_Wrist::WristPositionMidGoal,
         * m_Wrist));
         * rightBumper.onTrue(new
         * StartEndCommand(m_Wrist::WristPositionZero,m_Wrist::WristPositionHighGoal,
         * m_Wrist));
         * 
         * aButton.toggleOnTrue(new
         * StartEndCommand(m_Gripper::GripperIn,m_Gripper::GripperStop,m_Gripper));
         * bButton.toggleOnTrue(new
         * StartEndCommand(m_Gripper::GripperOut,m_Gripper::GripperStop,m_Gripper));
         * 36
         */
        // LED COMMANDS
        yButton.onTrue(high_goal);
        xButton.onTrue(shelf_HuPL);

        aButton.onTrue(ground);
        bButton.onTrue(mid_goal);

        leftBumper.onTrue(new ParallelCommandGroup(new InstantCommand(m_LED::toggleOutake),
                new StartEndCommand(m_Gripper::GripperOut, m_Gripper::GripperStop, m_Gripper)));
        rightBumper.onTrue(new ParallelCommandGroup(new InstantCommand(m_LED::toggleIntake),
                new StartEndCommand(m_Gripper::GripperIn, m_Gripper::GripperStop, m_Gripper)));

        leftBumper.onFalse(new ParallelCommandGroup(new InstantCommand(m_LED::toggleOutake),
                new StartEndCommand(m_Gripper::GripperStop, m_Gripper::GripperStop, m_Gripper)));
        rightBumper.onFalse(new ParallelCommandGroup(new InstantCommand(m_LED::toggleIntake),
                new StartEndCommand(m_Gripper::GripperStop, m_Gripper::GripperStop, m_Gripper)));

        backButton.onTrue(new ParallelCommandGroup(new InstantCommand(m_LED::toggleFastOutake),
                new StartEndCommand(m_Gripper::GripperFast, m_Gripper::GripperStop, m_Gripper)));
        backButton.onFalse(new ParallelCommandGroup(new InstantCommand(m_LED::toggleFastOutake),
                new StartEndCommand(m_Gripper::GripperStop, m_Gripper::GripperStop, m_Gripper)));

        dPadUpButton.onTrue(
                new StartEndCommand(m_Wrist::WristPositionZero, m_Wrist::WristPositionZero, m_Wrist));
        dPadRightButton
                .onTrue(new StartEndCommand(m_Wrist::WristPositionManualUp,
                        m_Wrist::WristPositionManualUp, m_Wrist));
        dPadLeftButton
                .onTrue(new StartEndCommand(m_ArmPivot::ArmPositionMidDown,
                        m_ArmPivot::ArmPositionMidDown,
                        m_ArmPivot));

        dPadDownButton.onTrue(zeros);

        SmartDashboard.putData("Active / Toggle Cone", new InstantCommand(m_LED::toggleCone));
        SmartDashboard.putData("Active / Toggle Cube", new InstantCommand(m_LED::toggleCube));

        ConstantsFactory factory = new ConstantsFactory("src/main/resources/driveConstants.yaml");
        m_constants = factory.getConstants(Constants.class);
        
        SmartDashboard.putString("Constant Name", m_constants.getName());

        // final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(m_camera,
        // m_robotDrive,
        // m_camera::getRobotPose2d);

        // starButton.toggleOnTrue(new BalanceChargeStation(m_robotDrive, true));

        // rightStick.toggleOnTrue(
        // new StartEndCommand(m_camera::CameraToAprilTag, m_camera::CameraPipeline,
        // m_camera));
        // Toggle Booleans
        // LED Light Trigger COntrol Code
        // bButton.onTrue(new InstantCommand(m_blinkin::setPurple));
        // aButton.onTrue(new InstantCommand(m_blinkin::setYellow));

        m_robotDrive.setDefaultCommand(new SlowSwerveDriveCommand(
                m_robotDrive,
                () -> -(m_driverController.getLeftY()) * 0.2,
                () -> m_driverController.getLeftX() * 0.2,
                () -> -m_driverController.getRightX() * 0.2,
                () -> m_driverController.getRightTriggerAxis() * 0.8,
                true, m_LED));

        // Drive without Slew
        // starButton.toggleOnTrue(
        // new SwerveDriveCommand(
        // m_robotDrive,
        // () -> -m_driverController.getLeftY(),
        // () -> m_driverController.getLeftX(),
        // () -> -m_driverController.getRightX() * 0.5,
        // () -> m_driverController.getRightTriggerAxis(),
        // true, m_LED));

        // Drive with Slew
        // m_robotDrive.setDefaultCommand(
        // new SwerveDriveCommand(
        // m_robotDrive,
        // ()-> -m_ySpeedLimiter.calculate(m_driverController.getLeftY()),
        // ()-> m_xSpeedLimiter.calculate(m_driverController.getLeftX()),
        // ()-> -m_driverController.getRightX()*0.7,
        // true));

    }

    public void periodic() {
        m_FieldSim.periodic();
        m_LED.update();
        // Already runs ever 5 miliseconds:
        // Add to a variable when joystick power > 0
        // Reset it to 0 when joystick = 0
        // dont let it go over 1
        SmartDashboard.putNumber("Left Joystick", m_driverController.getLeftY());
    }

    /*
     * public void setTeleOpGyro() {
     * // m_robotDrive.setOdometry(new Pose2d(3.67,1.30,new Rotation2d()));
     * 
     * // m_Chooser.addOption("Auto 1", new FollowTrajectory(m_robotDrive,
     * m_FieldSim, "Blue Auto 1", m_ArmInOut, m_ArmPivot, m_Gripper, m_Wrist,
     * m_LED));
     * // m_Chooser.addOption("Auto 2", new FollowTrajectory(m_robotDrive,
     * m_FieldSim, "Blue Auto 2", m_ArmInOut, m_ArmPivot, m_Gripper, m_Wrist,
     * m_LED));
     * // m_Chooser.addOption("Auto 3", new FollowTrajectory(m_robotDrive,
     * m_FieldSim, "Blue Auto 3", m_ArmInOut, m_ArmPivot, m_Gripper, m_Wrist,
     * m_LED));
     * 
     * 
     * m_robotDrive.teleOpGyroReset();
     * 
     * 
     * }
     */

    public Command robotDisabled() {

        Command zero = new ManipulatorZero(m_ArmPivot, m_ArmInOut, m_Wrist, m_LED);
        return zero;

    }

    public void teleopInit() {
        m_robotDrive.setBrakeMode(NeutralMode.Coast);

    }

    public void autoInit() {
        m_robotDrive.setBrakeMode(NeutralMode.Brake);
    }

    public void robotInit() {

        // m_robotDrive.resetEncoders();
        // m_Chooser.addOption("Score Mid Drive Back", scoreMidDriveBack);
        m_Chooser.addOption("Do Nothing", new WaitCommand(1));
        m_Chooser.addOption("Score Mid No Drive", scoreMidNoDrive);
        m_Chooser.addOption("Score Mid Drive Back", scoreMidDriveBack);
        m_Chooser.addOption("Score Mid Balance", scoreMidBalance);
        m_Chooser.addOption("Score High Cube No Drive", scoreHighCubeNoDrive);
        m_Chooser.addOption("Score High Cube Drive Back", scoreHighCubeDriveBack);
        m_Chooser.addOption("Score High Cube Balance", scoreHighCubeBalance);

        SmartDashboard.putData("Auto Selector", m_Chooser);


        
    }

    public Command getAutoCommand() {

        return m_Chooser.getSelected();
    }

}
