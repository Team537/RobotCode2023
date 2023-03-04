package frc.robot.utils;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeMap;
import java.util.stream.Collectors;

/** Generator for creating a drive trajectory and rotation sequence from a series of points. */
public class ChargerPathGenerator {
  private Trajectory swerveTrajectory = new Trajectory(List.of(new Trajectory.State()));
  private RotationSequence swerveRotationSequence = new RotationSequence(new TreeMap<>());

  /**
   * Charger Robotics Custom Path Trajectory Generator for Dynamic Path Generation
   * 
   * Use Carefully, as Sharp Turns will lead to not desirable results
   * 
   * 
   * @param config Trajectory configuration
   * @param pathPoints A series of points
   */
  public void generate(TrajectoryConfig config, List<PathPoint> pathPoints) {
    if (pathPoints.size() < 2) {
      throw new InvalidParameterException(
          "Please include at least 2 Points to generate a trajectory path.");
    }

    // Generate drive pathpoints
    List<Translation2d> swerveTranslations =
        pathPoints.stream().map(pathPoint -> pathPoint.getTranslation()).collect(Collectors.toList());
    List<Optional<Rotation2d>> swerveRotations =
        pathPoints.stream()
            .map(pathPoint -> pathPoint.getDriveRotation())
            .collect(Collectors.toList());
    swerveRotations.remove(0);
    swerveRotations.remove(swerveRotations.size() - 1);



    // Add first drive pathpoint
    if (pathPoints.get(0).getDriveRotation().isPresent()) {
      swerveRotations.add(0, pathPoints.get(0).getDriveRotation());
    } else {
      swerveRotations.add(
          0,
          Optional.of(
              pathPoints
                  .get(1)
                  .getTranslation()
                  .minus(pathPoints.get(0).getTranslation())
                  .getAngle()));
    }

    // Add last drive pathpoint
    if (pathPoints.get(pathPoints.size() - 1).getDriveRotation().isPresent()) {
      swerveRotations.add(pathPoints.get(pathPoints.size() - 1).getDriveRotation());
    } else {
      swerveRotations.add(
          Optional.of(
              pathPoints
                  .get(pathPoints.size() - 1)
                  .getTranslation()
                  .minus(pathPoints.get(pathPoints.size() - 2).getTranslation())
                  .getAngle()));
    }

    // Generate drive trajectory
    swerveTrajectory = new Trajectory();
    boolean firstPruneTrajectory = true;
    boolean nextQuintic = swerveRotations.get(1).isPresent();
    int index = 1;
    int pruneTrajectoryStart = 0;
    int pruneTrajectoryEnd = 0;
    while (true) {
      boolean generatePruneTrajectory = false;
      boolean lastpoint = index == pathPoints.size() - 1;
      if (nextQuintic) {
        if (lastpoint || swerveRotations.get(index).isEmpty()) { // Translation or End Found
          // pathpoints
          generatePruneTrajectory = true;
          pruneTrajectoryEnd = lastpoint ? index : index - 1;
        }
      } else {
        if (swerveRotations.get(index).isPresent()) { // Pose Found
          generatePruneTrajectory = true;
          pruneTrajectoryEnd = index;
        }
      }

      if (generatePruneTrajectory) {
        // Prune-trajectory configuration
        TrajectoryConfig pruneConfig = copyConfig(config);
        if (!firstPruneTrajectory) {
          pruneConfig.setStartVelocity(
              swerveTrajectory
                  .getStates()
                  .get(swerveTrajectory.getStates().size() - 1)
                  .velocityMetersPerSecond);
        }
        if (!lastpoint) {
          pruneConfig.setEndVelocity(pruneConfig.getMaxVelocity());
        }
        firstPruneTrajectory = false;

        // Generate Prune-trajectory 
        if (nextQuintic) {
          List<Pose2d> quinticPathpoints = new ArrayList<>();
          for (int i = pruneTrajectoryStart; i < pruneTrajectoryEnd + 1; i++) {
            quinticPathpoints.add(new Pose2d(swerveTranslations.get(i), swerveRotations.get(i).get()));
          }
          swerveTrajectory =
              swerveTrajectory.concatenate(
                  TrajectoryGenerator.generateTrajectory(quinticPathpoints, pruneConfig));
        } else {
          List<Translation2d> cubicInteriorPathPoints = new ArrayList<>();
          for (int i = pruneTrajectoryStart + 1; i < pruneTrajectoryEnd; i++) {
            cubicInteriorPathPoints.add(swerveTranslations.get(i));
          }
          swerveTrajectory =
              swerveTrajectory.concatenate(
                  TrajectoryGenerator.generateTrajectory(
                      new Pose2d(
                          swerveTranslations.get(pruneTrajectoryStart),
                          swerveRotations.get(pruneTrajectoryStart).get()),
                      cubicInteriorPathPoints,
                      new Pose2d(
                          swerveTranslations.get(pruneTrajectoryEnd),
                          swerveRotations.get(pruneTrajectoryEnd).get()),
                      pruneConfig));
        }

        // Break if complete
        if (lastpoint) {
          break;
        }

        // Prepare for next trajectory
        nextQuintic = !nextQuintic;
        pruneTrajectoryStart = pruneTrajectoryEnd;
      }

      index++;
    }

    // Find swerve pathpoints
    TreeMap<Double, Rotation2d> swervePathpoints = new TreeMap<>();
    int stateIndex = 0;
    for (int pathPointIndex = 0; pathPointIndex < swerveTranslations.size(); pathPointIndex++) {
      double timestamp;
      if (pathPointIndex == 0) {
        timestamp = 0.0;
      } else if (pathPointIndex == swerveTranslations.size() - 1) {
        timestamp = swerveTrajectory.getTotalTimeSeconds();
      } else {
        while (!swerveTrajectory
            .getStates()
            .get(stateIndex)
            .poseMeters
            .getTranslation()
            .equals(swerveTranslations.get(pathPointIndex))) {
          stateIndex++;
        }
        timestamp = swerveTrajectory.getStates().get(stateIndex).timeSeconds;
      }

      if (pathPoints.get(pathPointIndex).getSwerveRotation().isPresent()) {
        swervePathpoints.put(
            timestamp, pathPoints.get(pathPointIndex).getSwerveRotation().get());
      }
    }
    swerveRotationSequence = new RotationSequence(swervePathpoints);
  }

  private TrajectoryConfig copyConfig(TrajectoryConfig config) {
    TrajectoryConfig newConfig =
        new TrajectoryConfig(config.getMaxVelocity(), config.getMaxAcceleration());
    newConfig.addConstraints(config.getConstraints());
    newConfig.setStartVelocity(config.getStartVelocity());
    newConfig.setEndVelocity(config.getEndVelocity());
    newConfig.setReversed(config.isReversed());
    return newConfig;
  }

  /** Returns the generated drive trajectory. */
  public Trajectory getSwerveTrajectory() {
    return swerveTrajectory;
  }

  /** Returns the generated swerve rotation sequence. */
  public RotationSequence getSwerveRotationSequence() {
    return swerveRotationSequence;
  }
}