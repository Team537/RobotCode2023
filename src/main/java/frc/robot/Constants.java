// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    // public static final int kFrontLeft = 9;
    // public static final int kRearLeft = 3;
    // public static final int kFrontRight = 5;
    // public static final int kRearRight = 7;

    //For Bolt Testing 
    /*
     public static final int kFrontLeft = 1;
     public static final int kRearLeft = 3;
     public static final int kFrontRight = 2;
     public static final int kRearRight = 4;*/

    public static final double kTrackwidthMeters = 0.415;
    public static final double kWheelBase =  0.415;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    /*  These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        These characterization values MUST be determined either experimentally or
        theoretically
        for *your* robot's drive.
        The Robot Characterization Toolsuite provides a convenient tool for obtaining
        these values for your robot. */
        
    public static final double ksVolts = 0.305;
    public static final double kvVoltSecondsPerMeter = 2.29;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0131;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;


    //Gyro Command Constants
    public static final double kTurnP = 8.5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 4;
    public static final double kTurnRateToleranceDegPerS = 1;




  }

  public static final class GripperConstants{
    public static final int kGripper = 15;
    public static final int kGripper2 = 17; // subject to change

  }
  
  public static final class ArmInOutConstants {
    public static final int kArmInOut = 14;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    //Max Velocity
    public static final double kMaxV = 3000;
    public static final double kMinV = 0;
    //Max Acceleration
    public static final double kMaxA = 2500;
    public static final double kAllE = 0;

    //ARM INOUT SET POSITIONS
    public static final double kArmPositionIn = -50;
    public static final double kArmPositionOut = 0.0;
    public static final double kArmPositionOutOut = -200.0;

    //INCREMENT TEST POS
    /*public static final double kIncrementUp = -10;
    public static final double kIncrementDown = 10;*/
  }



  public static final class ArmPivotConstants {

    public static final int kArmPivot1 = 19;
   

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    //max velocity
    public static final double kMaxV = 3000;
    public static final double kMinV = 0;
    //max acceleration
    public static final double kMaxA = 2000;
    public static final double kAllE = 0;

    //ARM PIVOT SET POSITONS
    public static final double kArmPositionUp = 58.0;
    public static final double kArmPositionMiddle = 15.0;
    public static final double kArmPositionDown = 0.0;

  }

  public static final class WristConstants {

    public static final int kWrist = 11;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    //Max Velocity
    public static final double kMaxV = 3000;
    public static final double kMinV = 0;
    //Max Acceleration
    public static final double kMaxA = 2500;
    public static final double kAllE = 0;

    //WRIST SET POSITIONS
    public static final double kWristPositionUp = -10;
    public static final double kWristPositionMiddle = -3;
    public static final double kWristPositionDown = -5;
    public static final double kWristPositionZero = 0;

    /* 
    optimal picking up: -5
    optimal drop off cone: -10
    

    */



  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort1 = 1;
  }

  public final static class Conversions {
    
    public static double metersToFeet(double meters) {
      return meters * 3.28084;
    }

    public static double feetToMeters(double feet) {
      return feet * 0.3048;
    }

    public static double ticksToFeet(double ticks) {
      return (ticks / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12);
    }

    public static double ticksToFeetPerSecond(double ticksPer100ms) {
      return (ticksPer100ms / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12) * 10;
    }

    public static double ticksToMetersWheel(double ticks) {
      return (ticks / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762);
    }

    public static double ticksToMetersPerSecondWheel(double ticksPer100ms) {
      return (ticksPer100ms / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762) * 10;
    }

    public static double degreesToTicks(double angle) {
      return 4096.00 * (angle / 360.00);
    }

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = .2;
    public static final double kMaxAccelerationMetersPerSecondSquared =.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 3.5;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(SwerveConstants.kModuleMaxAngularVelocity * 10,
                        SwerveConstants.kModuleMaxAngularAcceleration * 10);
  }

  public static class kGains {

    public static final double kP = 0.000102;
    public static final double kI = 0.0;
    public static final double kD = 0.000438;
    public static final double kF = 0.0;
  }

  public static class GyroPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

                  //@ASINX where do these constants go to
                  public static final int kTimeoutMs = 10;
                  public static final int kPIDLoopIdx = 0;
                  public static final int kSlotIdx = 0;

                  //gear ratio is 6:1 
                  public static final double targetMeters = 2 * (6 * 2048 * 0.4787787204060999);

                  //Smoothing is from 0 to 8
                  public static final int smoothing = 4;
  

  public static class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(0.0, -0.1375, -.90), new Rotation3d(0.0, 0.0, -0.10));
        public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
  } 

  public static class limelight {
    public static final int kCamera = 0;
    public static final int kLimelight = 1;
    public static final int kAprilTagRed = 2;
    public static final int kAprilTagBlue = 3;


  }


  public static class SwerveConstants {
    public static final double kWheelRadius = 1.75;
    public static final int kEncoderResolution = 4096;
    public static final double  kDriveMotorGearRatio = 7.13; 
    public static final double kTurningMotorGearRatio = 15.428;
    public static final double kMaxSpeed = 15.0;
    public static final double kModuleMaxAngularVelocity = Math.PI*4;
    public static final double kModuleMaxAngularAcceleration = 3 * Math.PI; 

    public static final double kPModuleTurningController = 0;
    public static final double kPModuleDriverController = 0;

    public static final double kSTurn = 0.1;
    public static final double kVTurn = 0.2;
    
    public static final double kSDrive = 0.587;
    public static final double kVDrive = 2.3;
    public static final double kADrive = 0.517;

    public static final double ksDriveVoltSecondsPerMeter = 1;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1;
    public static final double kaDriveVoltSecondsSquaredPerMeter = .5;


    public static final double kTurningEncoderDistancePerPulse =
    
    360 / (DriveConstants.kEncoderCPR* kTurningMotorGearRatio);

    public static final double kDriveEncoderDistancePerPulse =
    (2*kWheelRadius * Math.PI) / (DriveConstants.kEncoderCPR * kDriveMotorGearRatio);

    public static final double kMaxSpeedMetersPerSecond = 8;
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 4;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 1;
    

    public static final int kFrontLeftDrive = 0;
    public static final int kFrontLeftTurn = 1;
    public static final int kFrontRightDrive = 2;
    public static final int kFrontRightTurn = 3;
    public static final int kBackLeftDrive = 4;
    public static final int kBackLeftTurn = 5;
    public static final int kBackRightDrive = 6;
    public static final int kBackRightTurn = 7;
    
    public static final int kFrontLeftSRXMagCoder = 0;
    public static final int kFrontRightSRXMagCoder = 1;
    public static final int kBackLeftSRXMagCoder = 2;
    public static final int kBackRightSRXMagCoder = 3;

    public static final double kFrontLeftSRXMagCoderOffset = 320;//175;
    public static final double kFrontRightSRXMagCoderOffset = 0;
    public static final double kBackLeftSRXMagCoderOffset = 0.;
    public static final double kBackRightSRXMagCoderOffset = 0.;

    public static final double kP_X = 0.01;
    public static final double kI_X = 0;
    public static final double kD_X = 0.1;
    public static final double kP_Y = 0.01;
    public static final double kI_Y = 0;
    public static final double kD_Y = 0.1;
    public static final double kP_Rot = 0.001;
    public static final double kI_Rot = 0;
    public static final double kD_Rot = 0.01;

   public static final int kPigeonID = 10;

    public static final TrapezoidProfile.Constraints kRotControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);


        public static final Map<ModulePosition, Translation2d> kModuleTranslations =
        Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackwidthMeters / 2),
            ModulePosition.BACK_LEFT, new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackwidthMeters / 2),
            ModulePosition.BACK_RIGHT, new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2));

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( 
      ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));
           

        public enum ModulePosition {
          FRONT_LEFT,
          FRONT_RIGHT,
          BACK_LEFT,
          BACK_RIGHT
        }
  }
}


