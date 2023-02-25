// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.SRXMagEncoder;


public class SwerveModule extends SubsystemBase {


  ModulePosition m_modulePosition; //Enum of Module Positions
  int m_moduleNumber;
  WPI_TalonFX m_turnMotor;
  WPI_TalonFX  m_driveMotor;
  // CANCoder m_angleEncoder; // Mag Encoder
  double m_angleOffset; //Offset of Mag Encoder
  double m_lastAngle;
  double angle;
  double deg;
  Pose2d m_pose;
  SRXMagEncoder m_SrxMagEncoder;


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
  public SlewRateLimiter output = new SlewRateLimiter(0.3);

  public SwerveModule(
      ModulePosition modulePosition,
      WPI_TalonFX  turnMotor,
      WPI_TalonFX  driveMotor,
      int angleEncoder,
      double angleOffset,
      boolean isInverted) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal(); //Returns Index of Enum
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    // m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    m_driveMotor.setInverted(isInverted);
    // m_turnMotor.setInverted(isInverted);
    //Uses CTRE Utils to Configure Swerve Module Components for Optimal Performance

    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());

    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_SrxMagEncoder = new SRXMagEncoder(new DutyCycle(new DigitalInput(angleEncoder)), angleOffset);


    m_SrxMagEncoder.setDistancePerRotation(360);



    m_turnMotor.configFactoryDefault();
    m_turnMotor.configAllSettings(CtreUtils.generateTurnMotorConfig());
    // m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, angleEncoder, 10);
    // m_turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

    // m_angleEncoder.configFactoryDefault();
    // m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());
    
   

    resetAngleToAbsolute();
  }

/**
 * Gets Module Position
 * @return Module Position
 * 
 */
  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }


/**
 * Resets Angle to Absolute Position by subtracting angle offset
 * and setting the Turn Motor to the Set Position
 * 
 * 
 */
  public void resetAngleToAbsolute() {

  //   double pos = 0;
  // double angle =  (m_SrxMagEncoder.getAbsolutePosition() - m_SrxMagEncoder.getPositionOffset());

  //    m_turnMotor.setSelectedSensorPosition(angle+pos);
   }

/**
 * Gets Heading in Degrees
 * @return Turn Motor Position
 * 
 * 
 */
  public double getHeadingDegrees() {
    return m_turnMotor.getSelectedSensorPosition() * SwerveConstants.kTurningEncoderDistancePerPulse;
  };
  
/**
 * Gets {@link Rotation2d} from Heading
 * @return {@link Rotation2d}
 * 
 * 
 */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }


  /**
 * Get Drive Speed in Meters per Second
 * @return Drive Velocity
 * 
 * 
 */
  public double getDriveMetersPerSecond() {
    return m_driveMotor.getSelectedSensorVelocity() * SwerveConstants.kDriveEncoderDistancePerPulse * 10;
  }

  // public void setDriveInvertedt() {
  //   m_driveMotor.setInverted(true);
  // }

/**
 * Get Drive Distance in Meters 
 * @return Drive Motor Encoder Position
 * 
 * 
 */
  public double getDriveMeters() {
    return m_driveMotor.getSelectedSensorPosition() *SwerveConstants.kDriveEncoderDistancePerPulse;
  }


/**
 * Sets Optimal State using CTRE Utils Optimization and sets Motor Percent Output using Gyro Angle
 * @param desiredState Sweve Module States
 * @param isOpenLoop Controls the feedback loop to be open or closed, Useful for Auto
 * 
 * 
 * 
 */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());


    //Feedback loop Type

    if (isOpenLoop) {
      double percentOutput = output.calculate(desiredState.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond);
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = output.calculate(desiredState.speedMetersPerSecond / (SwerveConstants.kDriveEncoderDistancePerPulse * 10));
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    //Turn Motor Output Adjustment based on Angle
    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxRotationRadiansPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%.
    m_turnMotor.set(ControlMode.Position, angle / SwerveConstants.kTurningEncoderDistancePerPulse);
    m_lastAngle = angle;

    m_drivePercentOutput = m_driveMotor.getMotorOutputPercent();
    m_turnPercentOutput = m_turnMotor.getMotorOutputPercent();
  }
  
  /**
 * Get Module State
 * @return {@link SwerveModuleState} 
 * 
 * 
 */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }
/**
 * Get Module Position
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
    m_pose = pose;
  }
/**
 * Gets Module Pose
 * 
 * @return {@link Pose2d}
 * 
 */
  public Pose2d getModulePose() {
    return m_pose;
  }
/**
 * Sets Drive Neutral Mode
 * 
 * 
 * 
 */
  public void setDriveNeutralMode(NeutralMode Coast) {
    m_driveMotor.setNeutralMode(Coast);
  }
/**
 * Sets Turn Neutral Mode
 * 
 * 
 * 
 */
  public void setTurnNeutralMode(NeutralMode Brake) {
    m_turnMotor.setNeutralMode(Brake);
  }


 

   

/**
 * Updates SmartDashboard
 * 
 * 
 * 
 */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber(
        "Module " + m_moduleNumber + " Heading", getState().angle.getDegrees());
    SmartDashboard.putNumber(
        "Module " + m_moduleNumber + " Mag Coder Reading", m_SrxMagEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(
          "Module " + m_moduleNumber + " Integrated Sensor Reading", m_turnMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(
          "Module " + m_moduleNumber + " Position", getDriveMeters());
    SmartDashboard.putNumber(
            "Module " + m_moduleNumber + " Linear Velocity", getDriveMetersPerSecond());
  
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
    deg = m_SrxMagEncoder.getDistance();
     angle = Math.toRadians(deg);
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
        deg =+ 360;
    }
  }

  /**
 * Runs Periodically during Simulation
 * 
 * 
 * 
 */
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





  public void resetEncoders(){

 double position = m_SrxMagEncoder.getAbsolutePosition();

//  double angle = position/(4096/360);
// Not needed for now, needed if frequency is used to determine position

 m_turnMotor.setSelectedSensorPosition(0);

  }

       
}
