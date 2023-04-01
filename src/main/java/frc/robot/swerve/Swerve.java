package frc.robot.swerve;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import frc.robot.utils.ModuleMap;
import frc.robot.utils.TunableNumber;

public class Swerve extends SubsystemBase {
    private static final double coastThresholdMetersPerSec = 0.05; // Value of which under the Swerve will coast when
                                                                   // disabled
    private static final double coastThresholdSecs = 6.0; // Speed at which the Swerve needs to be under for this amoutn
                                                          // of time to swtich to coast
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
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

    private HashMap<ModulePosition, Module> swerveModules = new HashMap<>(
            Map.of(ModulePosition.FRONT_LEFT, new Module(new SwerveModuleIOSim(), ModulePosition.FRONT_LEFT),
                    ModulePosition.FRONT_RIGHT, new Module(new SwerveModuleIOSim(), ModulePosition.FRONT_RIGHT),
                    ModulePosition.BACK_LEFT, new Module(new SwerveModuleIOSim(), ModulePosition.BACK_LEFT),
                    ModulePosition.BACK_RIGHT, new Module(new SwerveModuleIOSim(), ModulePosition.BACK_RIGHT)

            ));

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            kinematics,
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());

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
        // swerveModules = new HashMap<>(
        // Map.of(ModulePosition.FRONT_LEFT, new Module(flModuleIO,
        // ModulePosition.FRONT_LEFT),
        // ModulePosition.FRONT_RIGHT, new Module(frModuleIO,
        // ModulePosition.FRONT_RIGHT),
        // ModulePosition.BACK_LEFT, new Module(blModuleIO, ModulePosition.BACK_LEFT),
        // ModulePosition.BACK_RIGHT, new Module(brModuleIO, ModulePosition.BACK_RIGHT)

        // ));

        lastMovementTimer.start();
        for (var module : ModuleMap.orderedValuesList(swerveModules)) {
            module.setBrakeMode(false);
        }

    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);
        for (var module : ModuleMap.orderedValuesList(swerveModules)) {
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
            for (var module : ModuleMap.orderedValuesList(swerveModules)) {
                module.stop();
            }

            // Clear logs
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

        } else if (isCharacterizing) {
            // Run in characterization mode
            for (var module : ModuleMap.orderedValuesList(swerveModules)) {
                module.runCharacterization(characterizationVolts);
            }

            // Clear setpoint logs
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

        } else {
            // Calculate module setpoints
            Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
                    .of(SwerveConstants.kDriveKinematics.toSwerveModuleStates(setpoint));

            SwerveDriveKinematics.desaturateWheelSpeeds(
                    ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]),
                    SwerveConstants.kMaxSpeedMetersPerSecond);

            for (Module module : ModuleMap.orderedValuesList(swerveModules))
                module.setDesiredState(moduleStates.get(module.getPosition()));

            Logger.getInstance().recordOutput("Swerve/States", getModuleStates());

        }

        // Update brake mode
        boolean stillMoving = false;
        Map<ModulePosition, Double> map = new HashMap<>(
                Map.of(ModulePosition.FRONT_LEFT, 0.0,
                        ModulePosition.FRONT_RIGHT, 1.0,
                        ModulePosition.BACK_LEFT, 2.0,
                        ModulePosition.BACK_RIGHT, 3.0)

        );

        for (ModulePosition i : swerveModules.keySet()) {
            if (Math.abs(map.put(i, swerveModules.get(i).getVelocityMetersPerSec())) > coastThresholdMetersPerSec) {
                stillMoving = true;
            }
        }
        if (stillMoving)
            lastMovementTimer.reset();
        if (DriverStation.isEnabled()) {
            if (!isBrakeMode) {
                isBrakeMode = true;
                for (var module : ModuleMap.orderedValuesList(swerveModules)) {
                    module.setBrakeMode(true);
                }
            }
        } else {
            if (isBrakeMode && lastMovementTimer.hasElapsed(coastThresholdSecs)) {
                isBrakeMode = false;
                for (var module : ModuleMap.orderedValuesList(swerveModules)) {
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
        return m_odometry.getEstimatedPosition();
    }

    public Rotation2d getHeadingRotation2d() {
        return new Rotation2d(gyroInputs.yawPositionRad);
    }

    public void runCharacterizationVolts(double volts) {
        isCharacterizing = true;
        characterizationVolts = volts;
    }

    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : ModuleMap.orderedValuesList(swerveModules)) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
                swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
                swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
                swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
                swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
                swerveModules.get(ModulePosition.BACK_LEFT).getState(),
                swerveModules.get(ModulePosition.BACK_RIGHT).getState()
        };
    }

    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {

                // Old Translations
                // new Translation2d(-DriveConstants.kWheelBase / 2,
                // DriveConstants.kTrackwidthMeters / 2),
                // new Translation2d(-DriveConstants.kWheelBase / 2,
                // DriveConstants.kTrackwidthMeters / 2),
                // new Translation2d(DriveConstants.kWheelBase / 2,
                // -DriveConstants.kTrackwidthMeters / 2),
                // new Translation2d(DriveConstants.kWheelBase / 2,
                // DriveConstants.kTrackwidthMeters / 2)
                new Translation2d(trackWidthX.get() / 2.0, trackWidthY.get() / 2.0),
                new Translation2d(trackWidthX.get() / 2.0, -trackWidthY.get() / 2.0),
                new Translation2d(-trackWidthX.get() / 2.0, trackWidthY.get() / 2.0),
                new Translation2d(-trackWidthX.get() / 2.0, -trackWidthY.get() / 2.0)
        };
    }

}
