package frc.robot.utils;


import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;


// Path Point for Dynamic Trajectories
public class PathPoint {
  private final Translation2d translation;
  private final Rotation2d driveRotation;
  private final Rotation2d swerveRotation;

  /** Constructs a pathpoint at the origin and without rotation **/
  public PathPoint() {
    this(new Translation2d());
  }

  public PathPoint(
      Translation2d translation, Rotation2d driveRotation, Rotation2d holonomicRotation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = driveRotation;
    this.swerveRotation = holonomicRotation;
  }

  public PathPoint(Translation2d translation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = null;
    this.swerveRotation = null;
  }

  
  public static PathPoint fromDiffPose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new PathPoint(pose.getTranslation(), pose.getRotation(), null);
  }

 
  public static PathPoint fromDiffPose(Pose2d pose, Rotation2d holonomicRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new PathPoint(pose.getTranslation(), pose.getRotation(), holonomicRotation);
  }

 
  public static PathPoint fromSwervePose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new PathPoint(pose.getTranslation(), null, pose.getRotation());
  }


  public static PathPoint fromSwervePose(Pose2d pose, Rotation2d driveRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new PathPoint(pose.getTranslation(), driveRotation, pose.getRotation());
  }

  /** Returns the translation component of the pathpoint. */
  public Translation2d getTranslation() {
    return translation;
  }

  /**
   * Returns the drive rotation component of the pathpoint (or an empty optional if not specified).
   */
  public Optional<Rotation2d> getDriveRotation() {
    return Optional.ofNullable(driveRotation);
  }

  /**
   * Returns the holonomic rotation component of the pathpoint (or an empty optional if not
   * specified).
   */
  public Optional<Rotation2d> getSwerveRotation() {
    return Optional.ofNullable(swerveRotation);
  }
}