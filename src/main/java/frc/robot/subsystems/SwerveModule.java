// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.CtreUtils;


public class SwerveModule extends SubsystemBase {
  ModulePosition m_modulePosition;
  int m_moduleNumber;
  TalonFX m_turnMotor;
  TalonFX m_driveMotor;
  CANCoder m_angleEncoder;
  double m_angleOffset;
  double m_lastAngle;
  Pose2d m_pose;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.ksDriveVoltSecondsPerMeter,
          SwerveConstants.kvDriveVoltSecondsSquaredPerMeter,
          SwerveConstants.kaDriveVoltSecondsSquaredPerMeter);

  private final FlywheelSim m_turnMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.1, 0.0008),  DCMotor.getFalcon500(1), SwerveConstants.kTurningMotorGearRatio);

  private final FlywheelSim m_driveMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(4, 1.24),DCMotor.getFalcon500(1), SwerveConstants.kDriveMotorGearRatio);

  private double m_drivePercentOutput;
  private double m_turnPercentOutput;
  private double m_driveMotorSimDistance;
  private double m_turnMotorSimDistance;

  public SwerveModule(
      ModulePosition modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANCoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());

    m_turnMotor.configFactoryDefault();
    m_turnMotor.configAllSettings(CtreUtils.generateTurnMotorConfig());

    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());
    

    resetAngleToAbsolute();
  }

  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
    m_turnMotor.setSelectedSensorPosition(angle / SwerveConstants.kTurningEncoderDistancePerPulse);
  }

  public double getHeadingDegrees() {
    return m_turnMotor.getSelectedSensorPosition() * SwerveConstants.kTurningEncoderDistancePerPulse;
  };
  

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMetersPerSecond() {
    return m_driveMotor.getSelectedSensorVelocity() * SwerveConstants.kDriveEncoderDistancePerPulse * 10;
  }

  public double getDriveMeters() {
    return m_driveMotor.getSelectedSensorPosition() *SwerveConstants.kDriveEncoderDistancePerPulse;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = desiredState.speedMetersPerSecond / (SwerveConstants.kDriveEncoderDistancePerPulse * 10);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // Jittering.
    m_turnMotor.set(ControlMode.Position, angle / SwerveConstants.kTurningEncoderDistancePerPulse);
    m_lastAngle = angle;

    m_drivePercentOutput = m_driveMotor.getMotorOutputPercent();
    m_turnPercentOutput = m_turnMotor.getMotorOutputPercent();
  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  public void setDriveNeutralMode(NeutralMode mode) {
    m_driveMotor.setNeutralMode(mode);
  }

  public void setTurnNeutralMode(NeutralMode mode) {
    m_turnMotor.setNeutralMode(mode);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber(
        "module " + m_moduleNumber + " heading", getState().angle.getDegrees());
    SmartDashboard.putNumber(
        "module " + m_moduleNumber + " CANCoder reading", m_angleEncoder.getAbsolutePosition());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    m_turnMotorSim.setInputVoltage(m_turnPercentOutput * RobotController.getBatteryVoltage());
    m_driveMotorSim.setInputVoltage(m_drivePercentOutput * RobotController.getBatteryVoltage());

    m_turnMotorSim.update(0.02);
    m_driveMotorSim.update(0.02);

    Unmanaged.feedEnable(20);

    m_turnMotorSimDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
    m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;

    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int) (m_turnMotorSimDistance / SwerveConstants.kTurningEncoderDistancePerPulse));
    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_turnMotorSim.getAngularVelocityRadPerSec()
                    / (SwerveConstants.kTurningEncoderDistancePerPulse * 10)));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int) (m_driveMotorSimDistance / SwerveConstants.kDriveEncoderDistancePerPulse));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_driveMotorSim.getAngularVelocityRadPerSec()
                    / (SwerveConstants.kDriveEncoderDistancePerPulse * 10)));
  }

       
}
