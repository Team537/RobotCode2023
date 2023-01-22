package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;



public class SwerveModule extends SubsystemBase {
  int mModuleNumber;

  public final TalonFX mTurningMotor;
  public final TalonFX mDriveMotor;
  double mZeroOffset;
  boolean mInverted;

  static double kF;
  static double kP;
  static double kI;
  static double kD;
  int kI_Zone = 900;
  int kMaxIAccum = 1000000;
  int kErrorBand = 50;

  int kCruiseVelocity = 14000;
  int kMotionAcceleration = kCruiseVelocity * 10;


  private double m_turnOutput;
  private double m_driveOutput;

  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.kPModuleDriverController, 0, 0);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(SwerveConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(SwerveConstants.kModuleMaxAngularVelocity, SwerveConstants.kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSDrive, SwerveConstants.kVDrive, SwerveConstants.kADrive);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSTurn, SwerveConstants.kVTurn);

  private double simTurnEncoderDistance;
  private double simThrottleEncoderDistance;

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;
  private EncoderSim simulationTurnEncoderSim;
  private EncoderSim simulationThrottleEncoderSim;

  private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
        
          LinearSystemId.identifyVelocitySystem(0.16, SwerveConstants.kVoltSecondsSquaredPerRadian),
          DCMotor.getFalcon500(1),
          SwerveConstants.kTurningMotorGearRatio
  );

  private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(2, 1.24),
          DCMotor.getFalcon500(1),
          SwerveConstants.kDriveMotorGearRatio
  );

  Pose2d swerveModulePose = new Pose2d();

  public SwerveModule(int moduleNumber, TalonFX TurningMotor, TalonFX driveMotor, double zeroOffset, boolean invertTurn, boolean invertThrottle) {
    mModuleNumber = moduleNumber;
    mTurningMotor = TurningMotor;
    mDriveMotor = driveMotor;
    mZeroOffset = zeroOffset;

    mTurningMotor.configFactoryDefault();
    mTurningMotor.configOpenloopRamp(0.1);
    mTurningMotor.configClosedloopRamp(0.1);

    mTurningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mTurningMotor.setInverted(invertTurn);
    mDriveMotor.setInverted(invertThrottle);
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);

    mTurningMotor.config_kF(0,kF);
    mTurningMotor.config_kP(0,kP);
    mTurningMotor.config_kI(0,kI);
    mTurningMotor.config_IntegralZone(0, kI_Zone);
    mTurningMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    mTurningMotor.config_kD(0,kD);
    mTurningMotor.configMotionCruiseVelocity(kCruiseVelocity);
    mTurningMotor.configMotionAcceleration(kMotionAcceleration);
    mTurningMotor.configAllowableClosedloopError(0, kErrorBand);

    mTurningMotor.setNeutralMode(NeutralMode.Brake);
    mDriveMotor.setNeutralMode(NeutralMode.Brake);

    
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    if(RobotBase.isSimulation()) {
      switch (moduleNumber) {
        case 3:
          simulationTurnEncoder = new Encoder(15, 14);
          simulationThrottleEncoder = new Encoder(0, 1);
          break;
        case 2:
          simulationTurnEncoder = new Encoder(13, 12);
          simulationThrottleEncoder = new Encoder(2, 3);
          break;
        case 1:
          simulationTurnEncoder = new Encoder(11, 10);
          simulationThrottleEncoder = new Encoder(4, 5);
          break;
        case 0:
          simulationTurnEncoder = new Encoder(9, 8);
          simulationThrottleEncoder = new Encoder(6, 7);
          break;
      }

      simulationTurnEncoder.setDistancePerPulse(SwerveConstants.kTurningEncoderDistancePerPulse);
      simulationThrottleEncoder.setDistancePerPulse(SwerveConstants.kDriveEncoderDistancePerPulse);

      simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
      simulationThrottleEncoderSim = new EncoderSim(simulationThrottleEncoder);
    }
  }

    
  
 
  public void resetEncoders() {
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
    
  }

public void resetSimEncoders() {
  simulationTurnEncoder.reset();
    simulationThrottleEncoder.reset();
}

  public Rotation2d getHeading() {
    return new Rotation2d(getTurningRadians());
  }

 
  public double getTurningRadians() {
    if(RobotBase.isReal()) 
      return mTurningMotor.getSelectedSensorPosition() * SwerveConstants.kTurningEncoderDistancePerPulse;
    else
      return simulationTurnEncoder.getDistance();
  }

  

  public double getTurnAngle() {
    return Units.radiansToDegrees(getTurningRadians());
  }

public SwerveModulePosition getPosition() {

  return new SwerveModulePosition (mDriveMotor.getSelectedSensorPosition() * SwerveConstants.kTurningEncoderDistancePerPulse, 
  new Rotation2d(mTurningMotor.getSelectedSensorPosition() * SwerveConstants.kTurningEncoderDistancePerPulse));

}
  
  public double getVelocity() {
    if(RobotBase.isReal())
      return mDriveMotor.getSelectedSensorVelocity() * SwerveConstants.kDriveEncoderDistancePerPulse * 10;
    else
      return simulationThrottleEncoder.getRate();
  }

 
  public SwerveModuleState getState() {
      return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningRadians()));
  }

  
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    m_driveOutput = m_drivePIDController.calculate(
            getVelocity(), outputState.speedMetersPerSecond);

    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    m_turnOutput = m_turningPIDController.calculate(getTurningRadians(), outputState.angle.getRadians());

    // double turnFeedforward =
    //         m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);



    mDriveMotor.set(ControlMode.PercentOutput, m_driveOutput + driveFeedforward);
    mTurningMotor.set(ControlMode.PercentOutput, m_turnOutput);


  }

  public void setPercentOutput(double speed) {
    mDriveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBrakeMode(boolean mode) { // True is brake, false is coast
    mDriveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    mTurningMotor.setNeutralMode(NeutralMode.Brake);
  }
  public Pose2d getPose() {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }

 

  @Override
  public void simulationPeriodic() {
    moduleRotationSimModel.setInputVoltage(m_turnOutput / SwerveConstants.kModuleMaxAngularVelocity* RobotController.getBatteryVoltage());
    moduleThrottleSimModel.setInputVoltage(m_driveOutput / SwerveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());

    moduleRotationSimModel.update(0.02);
    moduleThrottleSimModel.update(0.02);

    simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationTurnEncoderSim.setDistance(simTurnEncoderDistance);
    simulationTurnEncoderSim.setRate(moduleRotationSimModel.getAngularVelocityRadPerSec());

    simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationThrottleEncoderSim.setDistance(simThrottleEncoderDistance);
    simulationThrottleEncoderSim.setRate(moduleThrottleSimModel.getAngularVelocityRadPerSec());


  }
}