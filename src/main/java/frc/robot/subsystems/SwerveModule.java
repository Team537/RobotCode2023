// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.AccelerationLimiter;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.SRXMagEncoder;

public class SwerveModule extends SubsystemBase {

  ModulePosition modulePosition;
  int moduleNumber;
  WPI_TalonFX turnMotor;
  WPI_TalonFX driveMotor;
  double angleOffset;
  double lastAngle;
  double angle;
  double dt;
  double deg;
  Pose2d pose;
  SRXMagEncoder SrxMagEncoder;
  private final Timer simTimer = new Timer();
  private double lastSimTime = 0;

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
      SwerveConstants.FEED_FORWARD_STATIC_GAIN,
      SwerveConstants.FEED_FORWARD_VELOCITY_GAIN,
      SwerveConstants.FEED_FORWARD_ACCELERATION_GAIN);

  private final FlywheelSim turnMotorSim = new FlywheelSim(
      // Sim Values
      LinearSystemId.identifyVelocitySystem(0.1, 0.0001), DCMotor.getFalcon500(1),
      SwerveConstants.TURNING_MOTOR_GEAR_RATIO);

  private final FlywheelSim driveMotorSim = new FlywheelSim(
      // Sim Values
      LinearSystemId.identifyVelocitySystem(1.6, 0.52878), DCMotor.getFalcon500(1),
      SwerveConstants.DRIVE_MOTOR_GEAR_RATIO);

  private double drivePercentOutput;
  private double turnPercentOutput;
  private double driveMotorSimDistance;
  private double turnMotorSimDistance;
  private final int driveEncoderSimSign;
  private final int turnEncoderSimSign;
  public SlewRateLimiter slewRateOutput = new SlewRateLimiter(100);
  public AccelerationLimiter accel = new AccelerationLimiter(25, 100);
  public LinearFilter filter = LinearFilter.movingAverage(2);

  public SwerveModule(
      ModulePosition modulePosition,
      WPI_TalonFX turnMotor,
      WPI_TalonFX driveMotor,
      int angleEncoder,
      double angleOffset,
      boolean isInverted) {
    this.modulePosition = modulePosition;
    moduleNumber = modulePosition.ordinal();
    this.turnMotor = turnMotor;
    this.driveMotor = driveMotor;
    // m_angleEncoder = angleEncoder;
    this.angleOffset = angleOffset;

    if (Robot.isReal()) {
      driveMotor.setInverted(isInverted);
    }

    if (RobotBase.isSimulation()) {
      simTimer.reset();
      simTimer.start();
    }

    driveEncoderSimSign = driveMotor.getInverted() ? -1 : 1;
    turnEncoderSimSign = turnMotor.getInverted() ? -1 : 1;

    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());
    driveMotor.setSensorPhase(true);
    driveMotor.setSafetyEnabled(true);
    driveMotor.enableVoltageCompensation(true);

    SrxMagEncoder = new SRXMagEncoder(new DutyCycle(new DigitalInput(angleEncoder)), 0);
    SrxMagEncoder.setDistancePerRotation(360);

    turnMotor.configFactoryDefault();
    turnMotor.configAllSettings(CtreUtils.generateTurnMotorConfig());

    // resetAngleToAbsolute();
  }

  /**
   * Gets Module Position
   * 
   * @return Module Position
   * 
   */
  public ModulePosition getModulePosition() {
    return modulePosition;
  }

  /**
   * Resets Angle to Absolute Position by subtracting angle offset
   * and setting the Turn Motor to the Set Position
   * 
   * 
   */
  public void resetAngleToAbsolute() {
    // TODO: Do we need to do 1 full revolution for the
    // mag encoder to read the right value?

    // Angle/Mag encoder position is always an absolute position - always sensing
    // motor position is always 0 when code starts
    // set motor position to opposite of mag, to make that mag angle 0
    // add wanted position to -angle to make position what you want
    var angle = Units.radiansToDegrees(SrxMagEncoder.getAbsoluteAngle());
    turnMotor.setSelectedSensorPosition((-angle + angleOffset) / SwerveConstants.TURN_ENCODER_METERS_PER_PULSE);

  }

  /**
   * Gets Heading in Degrees
   * 
   * @return Turn Motor Position
   * 
   * 
   */
  public double getHeadingDegrees() {
    return turnMotor.getSelectedSensorPosition() * SwerveConstants.TURN_ENCODER_METERS_PER_PULSE;
  };

  /**
   * Gets {@link Rotation2d} from Heading
   * 
   * @return {@link Rotation2d}
   * 
   * 
   */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Get Drive Speed in Meters per Second
   * 
   * @return Drive Velocity
   * 
   * 
   */
  public double getDriveMetersPerSecond() {
    return driveMotor.getSelectedSensorVelocity() * SwerveConstants.DRIVE_ENCODER_METERS_PER_PULSE * 10;
  }

  /**
   * Get Drive Distance in Meters
   * 
   * @return Drive Motor Encoder Position
   * 
   * 
   */
  public double getDriveMeters() {
    return driveMotor.getSelectedSensorPosition() * SwerveConstants.DRIVE_ENCODER_METERS_PER_PULSE;
  }

  /**
   * Sets Optimal State using CTRE Utils Optimization and sets Motor Percent
   * Output using Gyro Angle
   * 
   * @param desiredState Sweve Module States
   * @param isOpenLoop   Controls the feedback loop to be open or closed, Useful
   *                     for Auto
   * 
   * 
   * 
   */

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    setDriveState(desiredState);
    setTurnState(desiredState);

    // Get Percent Output for Sim

    drivePercentOutput = driveMotor.getMotorOutputPercent();
    turnPercentOutput = turnMotor.getMotorOutputPercent();
  }

  public void setDriveState(SwerveModuleState desiredState) {

    double velocity = (desiredState.speedMetersPerSecond / (SwerveConstants.DRIVE_ENCODER_METERS_PER_PULSE * 10));

    driveMotor.set(
        ControlMode.Velocity,
        velocity,
        DemandType.ArbitraryFeedForward,
        feedForward.calculate(desiredState.speedMetersPerSecond));
  }

  public void setTurnState(SwerveModuleState desiredState) {
    angle = (Math
        .abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_ROTATION_RADIANS_PER_SECOND * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%.
    turnMotor.set(ControlMode.Position, angle / SwerveConstants.TURN_ENCODER_METERS_PER_PULSE);
    lastAngle = angle;

  }

  /**
   * Get Module State
   * 
   * @return {@link SwerveModuleState}
   * 
   * 
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  /**
   * Get Module Position
   * 
   * @return {@link SwerveModulePosition}
   * 
   * 
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  /**
   * Sets Module Pose
   * 
   * 
   * 
   */
  public void setModulePose(Pose2d pose) {
    this.pose = pose;
  }

  /**
   * Gets Module Pose
   * 
   * @return {@link Pose2d}
   * 
   */
  public Pose2d getModulePose() {
    return pose;
  }

  /**
   * Sets Drive Neutral Mode
   * 
   * 
   * 
   */
  public void setDriveNeutralMode(NeutralMode Brake) {
    driveMotor.setNeutralMode(Brake);
  }

  /**
   * Sets Turn Neutral Mode
   * 
   * 
   * 
   */
  public void setTurnNeutralMode(NeutralMode Brake) {
    turnMotor.setNeutralMode(Brake);
  }

  /**
   * Updates SmartDashboard
   * 
   * 
   * 
   */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Heading", getState().angle.getDegrees());
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Mag Coder Reading", SrxMagEncoder.getAbsoluteAngle());
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Integrated Sensor Reading", turnMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Position", getDriveMeters());
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Linear Velocity", getState().speedMetersPerSecond);
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Angle", angle);
    SmartDashboard.putNumber(
        "Module " + moduleNumber + " Last Angle", lastAngle);

  }

  /**
   * Runs Periodically after Init
   * 
   * 
   * 
   */
  @Override
  public void periodic() {
    updateSmartDashboard();

  }

  /**
   * Runs Periodically during Simulation
   * 
   * 
   * 
   */
  @Override
  public void simulationPeriodic() {

    turnMotorSim.setInputVoltage(turnPercentOutput *
        RobotController.getBatteryVoltage());
    driveMotorSim.setInputVoltage(drivePercentOutput *
        RobotController.getBatteryVoltage());

    var currentTime = simTimer.get();
    dt = currentTime - lastSimTime;
    turnMotorSim.update(dt);
    driveMotorSim.update(dt);
    lastSimTime = currentTime;

    updateSimMotors();

    Unmanaged.feedEnable(20);

  }

  public void updateSimMotors() {
    turnMotorSimDistance += turnMotorSim.getAngularVelocityRadPerSec() * dt;
    driveMotorSimDistance += driveMotorSim.getAngularVelocityRadPerSec() * dt;

    turnMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(turnEncoderSimSign *
            (int) (turnMotorSimDistance / SwerveConstants.TURN_ENCODER_METERS_PER_PULSE));
    turnMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(turnEncoderSimSign *
            (int) (turnMotorSim.getAngularVelocityRadPerSec()
                / (SwerveConstants.TURN_ENCODER_METERS_PER_PULSE * 10)));
    driveMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(driveEncoderSimSign
            * (int) (driveMotorSimDistance / SwerveConstants.DRIVE_ENCODER_METERS_PER_PULSE));
    driveMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(driveEncoderSimSign
            * (int) (driveMotorSim.getAngularVelocityRadPerSec()
                / (SwerveConstants.DRIVE_ENCODER_METERS_PER_PULSE * 10)));

  }

  public void resetEncoders() {

    double position = SrxMagEncoder.getAbsolutePosition();

    // double angle = position/(4096/360);
    // Not needed for now, needed if frequency is used to determine position

    turnMotor.setSelectedSensorPosition(0);

  }

}
