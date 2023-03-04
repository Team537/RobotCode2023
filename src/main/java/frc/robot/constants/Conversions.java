package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.RotationSequence;


public final class Conversions {
    
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

    

      public static class GeoConversions {

        public static Transform2d translationToTransform2d(Translation2d translation2d) {
            return new Transform2d(translation2d, new Rotation2d());
          }
    
         public static Transform2d translationToTransform2d(double x, double y) {
            return new Transform2d(new Translation2d(x, y), new Rotation2d());
          }
    
        public static Transform2d rotationToTransform2d(Rotation2d rotation) {
            return new Transform2d(new Translation2d(), rotation);
          }
    
        public static Transform2d pose2dToTransform2d(Pose2d pose2d) {
            return new Transform2d(pose2d.getTranslation(), pose2d.getRotation());
          }
    
        public static Pose2d transform2dToPose2d(Transform2d transform2d) {
            return new Pose2d(transform2d.getTranslation(), transform2d.getRotation());
          }
    
        public static Pose2d rotationToPose2d(Rotation2d rotation) {
            return new Pose2d(new Translation2d(), rotation);
          }
        
        public static Twist2d multiplyTwist(Twist2d twist2d, double factor) {
            return new Twist2d(twist2d.dx * factor, twist2d.dy * factor, twist2d.dtheta * factor);
          }
    
         public static Transform3d pose3dToTransform3d(Pose3d pose3d) {
            return new Transform3d(pose3d.getTranslation(), pose3d.getRotation());
          }
    
          public static Pose3d transform3dToPose3d(Transform3d transform3d) {
            return new Pose3d(transform3d.getTranslation(), transform3d.getRotation());
          }
    
        public static Translation2d translation3dTo2dXY(Translation3d translation3d) {
            return new Translation2d(translation3d.getX(), translation3d.getY());
          }
    
        public static Translation2d translation3dTo2dXZ(Translation3d translation3d) {
            return new Translation2d(translation3d.getX(), translation3d.getZ());
          }

      }

      public static class AllianceFlipConversions {
        /** Flips a translation to the correct side of the field based on the current alliance color. */
        public static Translation2d apply(Translation2d translation) {
          if (shouldFlip()) {
            return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
          } else {
            return translation;
          }
        }
      
        /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
        public static double apply(double xCoordinate) {
          if (shouldFlip()) {
            return FieldConstants.fieldLength - xCoordinate;
          } else {
            return xCoordinate;
          }
        }
      
        /** Flips a rotation based on the current alliance color. */
        public static Rotation2d apply(Rotation2d rotation) {
          if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
          } else {
            return rotation;
          }
        }
      
        /** Flips a pose to the correct side of the field based on the current alliance color. */
        public static Pose2d apply(Pose2d pose) {
          if (shouldFlip()) {
            return new Pose2d(
                FieldConstants.fieldLength - pose.getX(),
                pose.getY(),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
          } else {
            return pose;
          }
        }
      
        /**
         * Flips a trajectory state to the correct side of the field based on the current alliance color.
         */
        public static Trajectory.State apply(Trajectory.State state) {
          if (shouldFlip()) {
            return new Trajectory.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                    FieldConstants.fieldLength - state.poseMeters.getX(),
                    state.poseMeters.getY(),
                    new Rotation2d(
                        -state.poseMeters.getRotation().getCos(),
                        state.poseMeters.getRotation().getSin())),
                -state.curvatureRadPerMeter);
          } else {
            return state;
          }
        }
      
        /** Flips a rotation sequence state based on the current alliance color. */
        public static RotationSequence.State apply(RotationSequence.State state) {
          if (shouldFlip()) {
            return new RotationSequence.State(
                new Rotation2d(-state.position.getCos(), state.position.getSin()),
                -state.velocityRadiansPerSec);
          } else {
            return state;
          }
        }
      
        private static boolean shouldFlip() {
          return DriverStation.getAlliance() == Alliance.Red;
        }
      }
  }


