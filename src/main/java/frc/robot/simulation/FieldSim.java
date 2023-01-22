// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

/** Add your docs here. */
public class FieldSim {

    private Field2d m_field2d;
    private DriveSubsystem m_drive =  new DriveSubsystem();

    public FieldSim(DriveSubsystem drive){

    m_drive = drive;
    m_field2d = new Field2d();

    }

    public Field2d getField2d() {
        return m_field2d;
    }

    public void initSim() {

        Pose2d startPosition = new Pose2d(Units.inchesToMeters(30),Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0)));
        m_field2d.setRobotPose(startPosition);

        m_drive.resetOdometry(m_field2d.getRobotPose(), startPosition.getRotation());

        m_field2d.getObject("trajectory").setPose(new Pose2d());
    }

    public void disabledInit() {
        
            }

    private void updateModulePoses() {


    }
    public void simulationPeriodic() {

        m_field2d.setRobotPose(m_drive.getPose());

        updateModulePoses();

        m_field2d.getObject("Swerve Modules").setPoses(m_drive.getModulePoses());

        SmartDashboard.putData("Field2d", m_field2d);

    }

    public Pose2d getRobotPose() {
        return m_field2d.getRobotPose();
    }

    public synchronized void resetRobotPose(Pose2d pose){
        m_field2d.setRobotPose(pose);
        m_drive.resetOdometry(pose, pose.getRotation());
    }




}
