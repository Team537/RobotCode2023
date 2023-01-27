// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {
  private final CANSparkMax m_ArmPivot1 = new CANSparkMax(Constants.ArmPivotConstants.kArmPivot1, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerPivot1 = m_ArmPivot1.getPIDController();
  private RelativeEncoder m_encoderPivot1 = m_ArmPivot1.getEncoder();
  private final CANSparkMax m_ArmPivot2 = new CANSparkMax(Constants.ArmPivotConstants.kArmPivot2, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerPivot2 = m_ArmPivot2.getPIDController();
  private RelativeEncoder m_encoderPivot2 = m_ArmPivot2.getEncoder();

  /** Creates a new ArmPivot. */
  public  ArmPivot() {

  }

  public void ArmPosition1() {
    m_pidControllerPivot1.setP(Constants.ClimberConstants.kP);
    m_pidControllerPivot1.setI(Constants.ClimberConstants.kI);
    m_pidControllerPivot1.setD(Constants.ClimberConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ClimberConstants.kLeftRotationsUp, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerPivot2.setP(Constants.ClimberConstants.kP);
    m_pidControllerPivot2.setI(Constants.ClimberConstants.kI);
    m_pidControllerPivot2.setD(Constants.ClimberConstants.kD);
    m_pidControllerPivot2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerPivot2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerPivot2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerPivot2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerPivot2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerPivot2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerPivot2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerPivot2.setReference(Constants.ClimberConstants.kRightRotationsUp, CANSparkMax.ControlType.kSmartMotion);
  }

  public void ArmPosition2() {
    m_pidControllerPivot1.setP(Constants.ClimberConstants.kP);
    m_pidControllerPivot1.setI(Constants.ClimberConstants.kI);
    m_pidControllerPivot1.setD(Constants.ClimberConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ClimberConstants.kLeftRotationsUp, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerPivot2.setP(Constants.ClimberConstants.kP);
    m_pidControllerPivot2.setI(Constants.ClimberConstants.kI);
    m_pidControllerPivot2.setD(Constants.ClimberConstants.kD);
    m_pidControllerPivot2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerPivot2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerPivot2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerPivot2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerPivot2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerPivot2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerPivot2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerPivot2.setReference(Constants.ClimberConstants.kRightRotationsUp, CANSparkMax.ControlType.kSmartMotion);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
