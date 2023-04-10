// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
public class Constants {

  public static final class DriveSpeedConstants {
    public static final double kMaxRobotSpeed = 1;
    public static final double kBaseRobotSpeed = 0.2;

  }

  public static final class SparkPIDFConstants {

    // This is the default PIDF for spark maxes. Perportional, Integral, Derivetive,
    // Izone and Feed Forward.
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;

    // This is the max output based on percent output forward and backward
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    // This is the max cruise velocity in RPM. This is the speed it travels at when
    // going to the set position
    public static final double kMaxV = 4000;
    public static final double kMinV = 0;

    // This is the max acceleration when accelerating to the cruise velocity. In RPM
    public static final double kMaxA = 4000;
    // specific accel constant for the pivot
    public static final double kMaxAccelArmPivot = 4000;

    // allowed closed loop error
    public static final double kAllE = 0;

  }

  public static final class DriveConstants {

    public static final double kTrackwidthMeters = 0.415;
    public static final double kWheelBase = 0.415;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double ksVolts = 0.305;
    public static final double kvVoltSecondsPerMeter = 2.29;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0131;

    public static final double kPDriveVel = 8.5;

    // Gyro Command Constants
    public static final double kTurnP = 8.5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 4;
    public static final double kTurnRateToleranceDegPerS = 1;

  }

  public static final class GripperConstants {

    // These are the ids for each gripper motor
    public static final int kGripper = 15;
    public static final int kGripper2 = 17;

  }

  public static final class ArmInOutConstants {

    // motor id for inout
    public static final int kArmInOut = 14;

    // These are the set positions for the differnt scoring heights on the arm inout
    // motor.
    // current limits are (+-)(it is +- because the string can get flipped) 300 to 0

    public static final double kArmInOutPositionHighGoal = 50; // for "shooting" cones, it was 100
    // the value for normal high goal is 225

    public static final double kArmInOutPositionMidGoal = 52;
    public static final double kArmInOutPositionShelfHumanPL = 60;
    public static final double kArmInOutPositionShelfHigh = 50;
    public static final double kArmInOutPositionGround = 0.0;

    // used for setting the robot to zero (and accounts for the string rolling up
    // different)
    public static final double kArmInOutPositionZero = -1;

    // positions used to increment inout positions
    /*
     * public static final double kIncrementUp = -10;
     * public static final double kIncrementDown = 10;
     */
  }

  public static final class ArmPivotConstants {
    // the id for the arm pivot constant.
    public static final int kArmPivot1 = 19;

    // these are the setpositions for the armpivot. current limits are 0 to 58
    public static final double kArmPivotPositionHighGoal = 7;
    public static final double kArmPivotPositionShelfHumanPL = 15;
    public static final double kArmPivotPositionMidGoal = 18;
    public static final double kArmPivotPositionShelfHigh = 25;

    public static final double kArmPivotPositionZero = 0.0;
    public static final double kArmPivotPositionGround = 58.0;

    // this is the position for moving tht arm down a bit when scoring mid. via
    // tylers prefrences
    public static final double kArmPivotPositionMidDown = 23;

  }

  public static final class WristConstants {
    // motor id for wrist
    public static final int kWrist = 11;

    // set positions for the wrist. known limits are 0 to -17(could be farther)
    public static final double kWristPositionHighGoal = -17;
    public static final double kWristPositionShelfHumanPL = -17;
    public static final double kWristPositionMidGoal = -12;
    public static final double kWristPositionShelfHigh = -10;

    public static final double kWristPositionZero = 0;
    public static final double kWristPositionGround = -6;

    // when carrying cones and cubes on the ground, they currently hit
    // the ground, need to add something to bring the wrist up 1 rotation
    public static final double kWristPositionManualUp = -3;
    public static final double kWristPositionManualDown = -15;

  }

  public static final class OIConstants {

    // these are the ports for each controller. the controller ports can be switched
    // in the driver station
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

  public static class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.0, -0.1375, -.90),
        new Rotation3d(0.0, 0.0, -0.10));
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
  }

  public static class limelight {
    public static final int kCamera = 0;
    public static final int kLimelight = 1;
    public static final int kAprilTagRed = 2;
    public static final int kAprilTagBlue = 3;

  }

  public static class LEDConstants {
    public static final int length = 59;
    public static final int centerLed = 29;
    public static final int halfLength = (int) Math.ceil(length / 2.0);
    public static final int batteryStartIndex = 72;
    public static final int batteryEndIndex = 118;
    public static final double strobeDuration = 0.2;
    public static final double rainbowFastFullLength = 40.0;
    public static final double rainbowFastDuration = 0.25;
    public static final double rainbowSlowFullLength = 80.0;
    public static final double rainbowSlowDuration = 4.0;
    public static final double breathDuration = 2.0;
    public static final double waveExponent = 0.4;
    public static final double waveFastFullLength = 40.0;
    public static final double waveFastDuration = 0.25;
    public static final double waveAllianceFullLength = 15.0;
    public static final double waveAllianceDuration = 2.0;
    public static final double waveSlowFullLength = 40.0;
    public static final double waveSlowDuration = 3.0;
  }

  public static class SwerveConstants {
    public static final double WHEEL_RADIUS = 1.75;
    public static final int ENCODER_RESOLUTION = 2048;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 7.13;
    public static final double TURNING_MOTOR_GEAR_RATIO = 15.428;

    public static final double FEED_FORWARD_STATIC_GAIN = 0.605 / 12;
    public static final double FEED_FORWARD_VELOCITY_GAIN = 1.72 / 13.8;
    public static final double FEED_FORWARD_ACCELERATION_GAIN = 0.193 / 12;

    public static final double TURN_ENCODER_METERS_PER_PULSE =

        360 / (SwerveConstants.ENCODER_RESOLUTION * TURNING_MOTOR_GEAR_RATIO);

    public static final double DRIVE_ENCODER_METERS_PER_PULSE = (2 * WHEEL_RADIUS * Math.PI)
        / (SwerveConstants.ENCODER_RESOLUTION * DRIVE_MOTOR_GEAR_RATIO);

    public static final double MAX_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ROTATION_RADIANS_PER_SECOND = Math.PI * 1;
    public static final double MAX_ROTATION_RADIANS_PER_SECOND_SQUARED = Math.PI * 1;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 3;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 5;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 7;

    public static final int FRONT_LEFT_SRX_MAG_ENCODER_ID = 4; // 0
    public static final int FRONT_RIGHT_SRX_MAG_ENCODER_ID = 5; // 1
    public static final int BACK_LEFT_SRX_MAG_ENCODER_ID = 6; // 2
    public static final int BACK_RIGHT_SRX_MAG_ENCODER_ID = 7; // 3

    public static final double FRONT_LEFT_SRX_MAG_ENCODER_OFFSET = 320;// 175;
    public static final double FRONT_RIGHT_SRX_MAG_ENCODER_OFFSET = 0;
    public static final double BACK_LEFT_SRX_MAG_ENCODER_OFFSET = 0.;
    public static final double BACK_RIGHT_SRX_MAG_ENCODER_OFFSET = 0.;

    public static final double P_X = 2;
    public static final double I_X = 0;
    public static final double D_X = 0;
    public static final double P_Y = 2;
    public static final double I_Y = 0;
    public static final double D_Y = 0;
    public static final double P_ROTATION = 0.1;
    public static final double I_ROTATION = 0;
    public static final double D_ROTATION = 0.01;

    public static final int PIGEON_ID = 10;

    public static final TrapezoidProfile.Constraints ROTATION_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ROTATION_RADIANS_PER_SECOND, MAX_ROTATION_RADIANS_PER_SECOND_SQUARED);

    public static final Map<ModulePosition, Translation2d> MODULE_TRANSLATIONS = Map.of(
        ModulePosition.FRONT_LEFT,
        new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2),
        ModulePosition.FRONT_RIGHT,
        new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackwidthMeters / 2),
        ModulePosition.BACK_LEFT,
        new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackwidthMeters / 2),
        ModulePosition.BACK_RIGHT,
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2));

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        ModuleMap.orderedValues(MODULE_TRANSLATIONS, new Translation2d[0]));

    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }
  // public class AutoConstants{

  // public HashMap<String, Command> autoMap = new HashMap<>();

  // }

}
