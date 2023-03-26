package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.CtreUtils;

public class SwerveModuleIOFalcon500 implements SwerveModuleIO {

    private final WPI_TalonFX m_drive;
    private final WPI_TalonFX m_turn;

    public SwerveModuleIOFalcon500(ModulePosition modulePosition) {

        switch (modulePosition) {

            case FRONT_LEFT:
                m_drive = new WPI_TalonFX(SwerveConstants.kFrontLeftDrive);
                m_turn = new WPI_TalonFX(SwerveConstants.kFrontLeftTurn);
                m_drive.setInverted(false);
                break;

            case FRONT_RIGHT:
                m_drive = new WPI_TalonFX(SwerveConstants.kFrontRightDrive);
                m_turn = new WPI_TalonFX(SwerveConstants.kFrontRightTurn);
                m_drive.setInverted(true);
                break;
            case BACK_LEFT:
                m_drive = new WPI_TalonFX(SwerveConstants.kBackLeftDrive);
                m_turn = new WPI_TalonFX(SwerveConstants.kBackLeftTurn);
                m_drive.setInverted(true);
                break;
            case BACK_RIGHT:
                m_drive = new WPI_TalonFX(SwerveConstants.kBackRightDrive);
                m_turn = new WPI_TalonFX(SwerveConstants.kBackRightTurn);
                m_drive.setInverted(false);
                break;
            default:
                throw new RuntimeException("Invalid module index for Swerve");

        }
        m_drive.configFactoryDefault();
        m_drive.configAllSettings(CtreUtils.generateDriveMotorConfig());
        m_drive.setSensorPhase(true);
        m_drive.setSafetyEnabled(true);
        m_drive.enableVoltageCompensation(true);
        m_drive.setNeutralMode(NeutralMode.Coast);

        m_turn.configFactoryDefault();
        m_turn.configAllSettings(CtreUtils.generateTurnMotorConfig());

    }

    public void updateInputs(SwerveModuleIOinputs inputs) {
        inputs.drivePositionRad = m_drive.getSelectedSensorPosition() * Math.PI * 2
                * SwerveConstants.kDriveMotorGearRatio / 2048;

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.driveVelocityRadPerSec = m_drive.getSelectedSensorVelocity() * 10 * Math.PI * 2
                * SwerveConstants.kDriveMotorGearRatio / 2048;

        inputs.driveVolts = m_drive.getMotorOutputVoltage() * m_drive.getBusVoltage();
        inputs.driveAmps = new double[] { m_drive.getSupplyCurrent() };
        inputs.driveTempCelsius = new double[] { m_drive.getTemperature() };

        inputs.turnAbsolutePositionRad = 0; // Temporary, wait until Mag Encoder Implementation
        inputs.turnPositionRad = m_turn.getSelectedSensorPosition() * 2 * Math.PI
                / (2048 * SwerveConstants.kTurningMotorGearRatio);

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.turnVelocityRadPerSec = m_turn.getSelectedSensorVelocity() * 10 * 2 * Math.PI
                / (2048 * SwerveConstants.kTurningMotorGearRatio);

        inputs.turnVolts = m_turn.getMotorOutputVoltage() * m_turn.getBusVoltage();
        inputs.turnAmps = new double[] { m_turn.getSupplyCurrent() };
        inputs.turnTempCelsius = new double[] { m_turn.getTemperature() };
    }

    public void setDriveVolts(double volts) {

        m_drive.setVoltage(volts);

    }

    public void setTurnVolts(double volts) {

        m_turn.setVoltage(volts);

    }

    public void setDriveBrake(boolean brake) {

        m_drive.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setTurnBrake(boolean brake) {

        m_turn.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }
}
