package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

public class TrajectoryUtils {
    public static ArrayList<Pose2d> readCsvTrajectory(String filename) {
        BufferedReader reader;
        String fileLine;
        String[] fields;
        ArrayList<Pose2d> trajectoryPoints = new ArrayList<>();
        String fullpath = "/home/lvuser/deploy/Trajectories/" + filename + ".csv";
        try {
            reader = new BufferedReader(new FileReader(fullpath));
            while ((fileLine = reader.readLine()) != null) {
                fields = fileLine.split(",");
                trajectoryPoints.add(new Pose2d(Units.feetToMeters(Double.parseDouble(fields[0])),
                                                Units.feetToMeters(Double.parseDouble(fields[1])),
                                                Rotation2d.fromDegrees(Double.parseDouble(fields[2]))));

            }
        } catch (FileNotFoundException e) {
            System.out.println("Error: Could not find file");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("Error: Could not read file");
            e.printStackTrace();
        }
        return trajectoryPoints;
    }

//    public static VitruvianRamseteCommand generateRamseteCommand(DriveTrain driveTrain, ArrayList<Pose2d> path, TrajectoryConfig config) {
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, config);
//
//        VitruvianRamseteCommand ramseteCommand = new VitruvianRamseteCommand(
//                trajectory,
//                driveTrain::getRobotPose,
//                new RamseteController(),
//                driveTrain.getFeedforward(),
//                driveTrain.getDriveTrainKinematics(),
//                driveTrain::getSpeeds,
//                driveTrain.getLeftPIDController(),
//                driveTrain.getRightPIDController(),
//                driveTrain::setVoltageOutput,
//                driveTrain,
//                path,
//                config
//        );
//        return ramseteCommand;
//    }

    public static SwerveControllerCommand generateSwerveCommand(DriveSubsystem m_drive, Trajectory trajectory) {

        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
                trajectory,
                m_drive::getPose,
                SwerveConstants.kDriveKinematics,
                //Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0,0),
                new PIDController(Constants.AutoConstants.kPYController, 0,0),
                new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),
                m_drive::setModuleStates,
                m_drive
        );
        return swerveCommand;
    }

    public static SwerveControllerCommand generateSwerveCommand(DriveSubsystem m_drive, Trajectory trajectory, Supplier<Rotation2d> robotHeading) {

        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
                trajectory,
                m_drive::getPose,
                SwerveConstants.kDriveKinematics,
                //Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0,0),
                new PIDController(Constants.AutoConstants.kPYController, 0,0),
                new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),
                robotHeading,
                m_drive::setModuleStates,
                m_drive
        );
        return swerveCommand;
    }
}