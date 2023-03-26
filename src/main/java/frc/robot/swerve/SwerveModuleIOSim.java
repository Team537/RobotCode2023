package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.kDriveMotorGearRatio, 0.02);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.kTurningMotorGearRatio,
            0.03);

    public double turnPositionRad = 0;
    public double turnAbsolutePositionRad = 0;
    public double driveVolts = 0;
    public double turnVolts = 0;

    public void updateInputs(SwerveModuleIOinputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnAbsolutePositionRad += angleDiffRad;
        turnPositionRad += angleDiffRad;

        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }

        while (turnAbsolutePositionRad > 0) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveVolts = driveVolts;
        inputs.driveAmps = new double[] { driveSim.getCurrentDrawAmps() };
        inputs.driveTempCelsius = new double[] {};

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnPositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnVolts = turnVolts;
        inputs.turnAmps = new double[] { turnSim.getCurrentDrawAmps() };
        inputs.turnTempCelsius = new double[] {};

    }

    public void setDriveVolts(double volts) {
        driveVolts = MathUtil.clamp(-12, volts, 12);
        driveSim.setInputVoltage(volts);

    }

    public void setTurnVolts(double volts) {
        turnVolts = MathUtil.clamp(-12, volts, 12);
        turnSim.setInputVoltage(volts);

    }

}
