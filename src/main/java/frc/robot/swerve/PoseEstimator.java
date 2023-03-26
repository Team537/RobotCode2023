package frc.robot.swerve;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GeoConversions;

public class PoseEstimator {
    private static final double historyLengthSecs = 0.3;
    private Pose2d basePose = new Pose2d();
    private Pose2d latestPose = new Pose2d();
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

    public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
        for (int i = 0; i < 3; ++i) {
            q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    public Pose2d getLatestPose() {
        return latestPose;
    }

    public void resetPose(Pose2d pose) {
        basePose = pose;
        updates.clear();
        update();
    }

    public void addDriveData(double timestamp, Twist2d twist) {
        updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        update();
    }

    private void update() {
        // Clear old data and update base pose
        while (updates.size() > 1
                && updates.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
            var update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose, q);
        }

    }

    // unused for now
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        for (var timestampedVisionUpdate : visionData) {
            var timestamp = timestampedVisionUpdate.timestamp();
            var visionUpdate = new VisionUpdate(timestampedVisionUpdate.pose(), timestampedVisionUpdate.stdDevs());

            if (updates.containsKey(timestamp)) {
                // There was already an update at this timestamp, add to it
                var oldVisionUpdates = updates.get(timestamp).visionUpdates();
                oldVisionUpdates.add(visionUpdate);
                oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);

            } else {
                // Insert a new update
                var prevUpdate = updates.floorEntry(timestamp);
                var nextUpdate = updates.ceilingEntry(timestamp);
                if (prevUpdate == null || nextUpdate == null) {
                    // Outside the range of existing data
                    return;
                }

                // Create partial twists (prev -> vision, vision -> next)
                var twist0 = GeoConversions.multiplyTwist(
                        nextUpdate.getValue().twist(),
                        (timestamp - prevUpdate.getKey()) / (nextUpdate.getKey() - prevUpdate.getKey()));
                var twist1 = GeoConversions.multiplyTwist(
                        nextUpdate.getValue().twist(),
                        (nextUpdate.getKey() - timestamp) / (nextUpdate.getKey() - prevUpdate.getKey()));

                // Add new pose updates
                var newVisionUpdates = new ArrayList<VisionUpdate>();
                newVisionUpdates.add(visionUpdate);
                newVisionUpdates.sort(VisionUpdate.compareDescStdDev);
                updates.put(timestamp, new PoseUpdate(twist0, newVisionUpdates));
                updates.put(
                        nextUpdate.getKey(), new PoseUpdate(twist1, nextUpdate.getValue().visionUpdates()));
            }
        }

        // Recalculate latest pose once
        update();
    }

    private static record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {
        public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
            // Apply drive twist
            var pose = lastPose.exp(twist);

            // Apply vision updates
            // unused for now
            for (var visionUpdate : visionUpdates) {
                // Kalman Gains Link to Source
                // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
                Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
                var r = new double[3];
                for (int i = 0; i < 3; ++i) {
                    r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
                }
                for (int row = 0; row < 3; ++row) {
                    if (q.get(row, 0) == 0.0) {
                        visionK.set(row, row, 0.0);
                    } else {
                        visionK.set(
                                row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
                    }
                }

                // Calculate twist between current and vision pose
                var visionTwist = pose.log(visionUpdate.pose());

                // Multiply by Kalman gain matrix
                var twistMatrix = visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

                // Apply twist
                pose = pose.exp(
                        new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }

    // unused for now
    public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
        public static final Comparator<VisionUpdate> compareDescStdDev = (VisionUpdate a, VisionUpdate b) -> {
            return -Double.compare(
                    a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                    b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
        };
    }

    // unused for now
    public static record TimestampedVisionUpdate(
            double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
    }

}
