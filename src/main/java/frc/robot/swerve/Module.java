package frc.robot.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.TunableNumber;

public class Module {

    private final SwerveModuleIO io;
    private final SwerveModuleIOinputsAutoLogged inputs = new SwerveModuleIOinputsAutoLogged();
    private final ModulePosition modulePosition;

    private static final TunableNumber wheelRadius = new TunableNumber("Swerve/Module/WheelRadius");
    private static final TunableNumber driveKp = new TunableNumber("Swerve/Module/Drive/DriveKp");
    private static final TunableNumber driveKi = new TunableNumber("Swerve/Module/Drive/DriveKi");
    private static final TunableNumber driveKd = new TunableNumber("Swerve/Module/Drive/DriveKd");
    private static final TunableNumber driveKs = new TunableNumber("Swerve/Module/Drive/DriveKs");
    private static final TunableNumber driveKv = new TunableNumber("Swerve/Module/Drive/DriveKv");

    private static final TunableNumber turnKp = new TunableNumber("Swerve/Module/Turn/TurnKp");
    private static final TunableNumber turnKi = new TunableNumber("Swerve/Module/Turn/TurnKi");
    private static final TunableNumber turnKd = new TunableNumber("Swerve/Module/Turn/TurnKd");

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
    private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

    static {
        wheelRadius.initDefault(Units.inchesToMeters(1.9193));
        driveKp.initDefault(0.1);
        driveKi.initDefault(0.0);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.18868);
        driveKv.initDefault(0.12825);
        turnKp.initDefault(0.5);
        turnKi.initDefault(0.0);
        turnKd.initDefault(0.0);
    }

    public Module(SwerveModuleIO io, ModulePosition modulePosition) {

        this.io = io;
        this.modulePosition = modulePosition;
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Swerve/Module" + modulePosition.toString(), inputs);

        if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode()) || driveKi.hasChanged(hashCode())) {
            driveFeedback.setPID(driveKp.get(), driveKi.get(), driveKd.get());
        }
        if (turnKp.hasChanged(hashCode()) || turnKd.hasChanged(hashCode()) || turnKi.hasChanged(hashCode())) {
            turnFeedback.setPID(turnKp.get(), turnKi.get(), turnKd.get());
        }
        if (driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())) {
            driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
        }
    }

    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        var desiredState = SwerveModuleState.optimize(state, getAngle());

        io.setTurnVolts(
                turnFeedback.calculate(getAngle().getRadians(), desiredState.angle.getRadians()));

        double velocityRadPerSec = desiredState.speedMetersPerSecond / wheelRadius.get();

        io.setDriveVolts(
                driveFeedforward.calculate(velocityRadPerSec)
                        + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

        return desiredState;

    }

    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }

    public void runCharacterization(double volts) {
        io.setTurnVolts(turnFeedback.calculate(getAngle().getRadians(), 0.0));
        io.setDriveVolts(volts);
    }

    public void stop() {
        io.setTurnVolts(0.0);
        io.setDriveVolts(0.0);
    }

    public void setBrakeMode(boolean brake) {
        io.setDriveBrake(brake);
        io.setTurnBrake(brake);
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadius.get();
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadius.get();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public static double getWheelRadius() {
        return wheelRadius.get();
    }
}
