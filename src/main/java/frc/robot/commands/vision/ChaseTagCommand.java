// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;

public class ChaseTagCommand extends CommandBase {
    private DriveSubsystem m_drive;
    private double lastTimeWhenFollowing = 0;
    private Camera m_camera;
    private LED m_LED;

    /** Creates a new BalanceChargeStation. */
    public ChaseTagCommand(DriveSubsystem m_drive, boolean isReversed, LED m_LED, Camera m_camera) {
        this.m_drive = m_drive;
        this.m_LED = m_LED;
        this.m_camera = m_camera;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Chase AprilTag");
        this.lastTimeWhenFollowing = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // var gyroPitch = m_drive.getGyroPitch();
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        var tagPosition = m_camera.tagPosX;
        double angleDeadband = 5;
        double speed = 0;

        if (Math.abs(tx) < angleDeadband) {

            m_drive.drive(0, 0, 0, true);
            m_LED.autoEnd();
            return;
        }
        var currentTime = System.currentTimeMillis();
        var diff = currentTime - lastTimeWhenFollowing;

        speed = tx > 0 ? 0.05 : -0.05;
        m_drive.drive(0, speed, 0, true);
        lastTimeWhenFollowing = System.currentTimeMillis();

        SmartDashboard.putNumber("Chase Strafe Speed", speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.setDiamondShape();
        System.out.println("Chase AprilTag Ended");
    }
}

// package frc.robot.commands.vision;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.Camera;
// import frc.robot.subsystems.DriveSubsystem;

// public class ChaseTagCommand extends CommandBase {

// private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new
// TrapezoidProfile.Constraints(3, 2);
// private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new
// TrapezoidProfile.Constraints(3, 2);
// private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new
// TrapezoidProfile.Constraints(8, 8);
// private boolean isTarget = false;
// private static final int TAG_TO_CHASE = 2;
// private static final Transform3d TAG_TO_GOAL =
// new Transform3d(
// new Translation3d(1.5, 0.0, 0.0),
// new Rotation3d(0.0, 0.0, Math.PI));

// private final Camera m_camera;
// private final DriveSubsystem drivetrainSubsystem;
// private final Supplier<Pose2d> poseProvider;

// private final ProfiledPIDController xController = new
// ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
// private final ProfiledPIDController yController = new
// ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
// private final ProfiledPIDController omegaController = new
// ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

// public ChaseTagCommand(
// Camera m_camera,
// DriveSubsystem drivetrainSubsystem,
// Supplier<Pose2d> poseProvider) {
// this.m_camera = m_camera;
// this.drivetrainSubsystem = drivetrainSubsystem;
// this.poseProvider = poseProvider;

// xController.setTolerance(0.2);
// yController.setTolerance(0.2);
// omegaController.setTolerance(Units.degreesToRadians(3));
// omegaController.enableContinuousInput(-Math.PI, Math.PI);

// addRequirements(drivetrainSubsystem);
// }

// @Override
// public void initialize() {
// var robotPose = poseProvider.get();
// omegaController.reset(robotPose.getRotation().getRadians());
// xController.reset(robotPose.getX());
// yController.reset(robotPose.getY());
// }

// @Override
// public void execute() {
// var robotPose2d = poseProvider.get();
// var robotPose =
// new Pose3d(
// robotPose2d.getX(),
// robotPose2d.getY(),
// 0.0,
// new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

// boolean target = m_camera.getValidTarget();

// if (target = true ) {

// isTarget = true;
// }

// if(isTarget){
// // Find the tag we want to chase
// var targetOpt = m_camera.getTagIds();

// var chaseTarget = m_camera.getValidTargetType();
// // This is new target data, so recalculate the goal
// // lastTarget = chaseTarget;

// // Transform the robot's pose to find the camera's pose
// var cameraPose = robotPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);

// // Trasnform the camera's pose to the target's pose
// Pose3d camToTarget = m_camera.getCameraPose3d();
// Transform3d cTransform3d = new Transform3d(cameraPose,camToTarget);

// var targetPose = cameraPose.transformBy(cTransform3d);

// // Transform the tag's pose to set our goal

// var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

// // Drive
// xController.setGoal(goalPose.getX());
// yController.setGoal(goalPose.getY());
// omegaController.setGoal(goalPose.getRotation().getRadians());
// }
// if (target = false) {
// // No target has been visible
// drivetrainSubsystem.drive(0, 0, 0, false, false);
// } else {
// // Drive to the target
// var xSpeed = xController.calculate(robotPose.getX());
// if (xController.atGoal()) {
// xSpeed = 0;
// }

// var ySpeed = yController.calculate(robotPose.getY());
// if (yController.atGoal()) {
// ySpeed = 0;
// }

// var omegaSpeed =
// omegaController.calculate(robotPose2d.getRotation().getRadians());
// if (omegaController.atGoal()) {
// omegaSpeed = 0;
// }

// drivetrainSubsystem.drive(xSpeed,ySpeed,omegaSpeed,false,false);
// }
// }

// @Override
// public void end(boolean interrupted) {

// }

// }