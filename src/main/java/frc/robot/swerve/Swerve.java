package frc.robot.swerve;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.swerve.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.utils.TunableNumber;

public class Swerve extends SubsystemBase {
    private static final double coastThresholdMetersPerSec = 0.05; // Value of which under the Swerve will coast when
                                                                   // disabled
    private static final double coastThresholdSecs = 6.0; // Speed at which the Swerve needs to be under for this amoutn
                                                          // of time to swtich to coast
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

    private static final TunableNumber maxLinearSpeed = new TunableNumber("Swerve/MaxLinearSpeed");
    private static final TunableNumber trackWidthX = new TunableNumber("Swerve/TrackWidthX");
    private static final TunableNumber trackWidthY = new TunableNumber("Swerve/TrackWidthY");

    private double maxAngularSpeed;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private boolean isCharacterizing = false;
    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };
    private double characterizationVolts = 0.0;
    private boolean isBrakeMode = false;
    private Timer lastMovementTimer = new Timer();

    private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Twist2d fieldVelocity = new Twist2d();
    static {
        maxLinearSpeed.initDefault(Units.feetToMeters(SwerveConstants.kMaxSpeed));
        trackWidthX.initDefault(DriveConstants.kTrackwidthMeters);
        trackWidthY.initDefault(DriveConstants.kTrackwidthMeters);
    }

    public Swerve(
            GyroIO gyroIO,
            SwerveModuleIO flModuleIO,
            SwerveModuleIO frModuleIO,
            SwerveModuleIO blModuleIO,
            SwerveModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, ModulePosition.FRONT_LEFT);
        modules[1] = new Module(frModuleIO, ModulePosition.FRONT_RIGHT);
        modules[2] = new Module(blModuleIO, ModulePosition.BACK_LEFT);
        modules[3] = new Module(brModuleIO, ModulePosition.BACK_RIGHT);
        lastMovementTimer.start();
        for (var module : modules) {
            module.setBrakeMode(false);
        }
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        if (maxLinearSpeed.hasChanged(hashCode())
                || trackWidthX.hasChanged(hashCode())
                || trackWidthY.hasChanged(hashCode())) {
            kinematics = new SwerveDriveKinematics(getModuleTranslations());
            maxAngularSpeed = maxLinearSpeed.get()
                    / Arrays.stream(getModuleTranslations())
                            .map(translation -> translation.getNorm())
                            .max(Double::compare)
                            .get();
        }
        if (DriverStation.isDisabled()) {
            // Stop moving while disabled
            for (var module : modules) {
                module.stop();
            }

            // Clear logs
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

        } else if (isCharacterizing) {
            // Run in characterization mode
            for (var module : modules) {
                module.runCharacterization(characterizationVolts);
            }

            // Clear setpoint logs
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

        } else {
            // Calculate module setpoints
            var setpointTwist = new Pose2d()
                    .log(
                            new Pose2d(
                                    setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                                    setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                                    new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)));
            var adjustedSpeeds = new ChassisSpeeds(
                    setpointTwist.dx / Constants.loopPeriodSecs,
                    setpointTwist.dy / Constants.loopPeriodSecs,
                    setpointTwist.dtheta / Constants.loopPeriodSecs);
            SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed.get());

            // Set to last angles if zero
            if (adjustedSpeeds.vxMetersPerSecond == 0.0
                    && adjustedSpeeds.vyMetersPerSecond == 0.0
                    && adjustedSpeeds.omegaRadiansPerSecond == 0) {
                for (int i = 0; i < 4; i++) {
                    setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
                }
            }
            lastSetpointStates = setpointStates;

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                optimizedStates[i] = modules[i].setDesiredState(setpointStates[i]);
            }

            // Log setpoint states
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

        // Update odometry
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
                    modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        var twist = kinematics.toTwist2d(wheelDeltas);
        var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
        if (gyroInputs.connected) {
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        }
        lastGyroYaw = gyroYaw;
        poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Log 3D odometry pose
        Pose3d robotPose3d = new Pose3d(getPose());
        robotPose3d = robotPose3d
                .exp(
                        new Twist3d(
                                0.0,
                                0.0,
                                Math.abs(gyroInputs.pitchPositionRad) * trackWidthX.get() / 2.0,
                                0.0,
                                gyroInputs.pitchPositionRad,
                                0.0))
                .exp(
                        new Twist3d(
                                0.0,
                                0.0,
                                Math.abs(gyroInputs.rollPositionRad) * trackWidthY.get() / 2.0,
                                gyroInputs.rollPositionRad,
                                0.0,
                                0.0));
        Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

        // Update field velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getRotation());
        fieldVelocity = new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                        ? gyroInputs.yawVelocityRadPerSec
                        : chassisSpeeds.omegaRadiansPerSecond);

        // Update brake mode
        boolean stillMoving = false;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(modules[i].getVelocityMetersPerSec()) > coastThresholdMetersPerSec) {
                stillMoving = true;
            }
        }
        if (stillMoving)
            lastMovementTimer.reset();
        if (DriverStation.isEnabled()) {
            if (!isBrakeMode) {
                isBrakeMode = true;
                for (var module : modules) {
                    module.setBrakeMode(true);
                }
            }
        } else {
            if (isBrakeMode && lastMovementTimer.hasElapsed(coastThresholdSecs)) {
                isBrakeMode = false;
                for (var module : modules) {
                    module.setBrakeMode(false);
                }
            }
        }

    }

    public void runVelocity(ChassisSpeeds speeds) {
        isCharacterizing = false;
        setpoint = speeds;
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        stop();
        for (int i = 0; i < 4; i++) {
            lastSetpointStates[i] = new SwerveModuleState(
                    lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
        }
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed.get();
    }

    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed;
    }

    public Twist2d getFieldVelocity() {
        return fieldVelocity;
    }

    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPositionRad);
    }

    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPositionRad);
    }

    public double getPitchVelocity() {
        return gyroInputs.pitchVelocityRadPerSec;
    }

    public double getRollVelocity() {
        return gyroInputs.rollVelocityRadPerSec;
    }

    public Pose2d getPose() {
        return poseEstimator.getLatestPose();
    }

    public Rotation2d getRotation() {
        return poseEstimator.getLatestPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    // unused for now
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        poseEstimator.addVisionData(visionData);
    }

    public void runCharacterizationVolts(double volts) {
        isCharacterizing = true;
        characterizationVolts = volts;
    }

    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2),
                new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2),
                new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackwidthMeters / 2),
                new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackwidthMeters / 2)
        };
    }

}
