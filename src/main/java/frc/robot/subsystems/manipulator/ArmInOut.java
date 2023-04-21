// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmInOut extends SubsystemBase {
  /** Creates a new ArmInOut. */
  public ArmInOut() {
  }

  private CANSparkMax m_ArmInOut = new CANSparkMax(Constants.ArmInOutConstants.kArmInOut, MotorType.kBrushless);
  private SparkMaxPIDController m_ArmInOutPidController = m_ArmInOut.getPIDController();
  private RelativeEncoder m_ArmInOutEncoder = m_ArmInOut.getEncoder();
  private String armInOutState = "Default";

  public void ArmInOutPidDefaults() {
    m_ArmInOutPidController.setP(Constants.SparkPIDFConstants.kP);
    m_ArmInOutPidController.setI(Constants.SparkPIDFConstants.kI);
    m_ArmInOutPidController.setD(Constants.SparkPIDFConstants.kD);
    m_ArmInOutPidController.setIZone(Constants.SparkPIDFConstants.kIz);
    m_ArmInOutPidController.setFF(Constants.SparkPIDFConstants.kFF);
    m_ArmInOutPidController.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_ArmInOutPidController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_ArmInOutPidController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_ArmInOutPidController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxVelocityArmInOut, 0);
    m_ArmInOutPidController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmInOut, 0);

  }

  public void ArmInOutMidGoal() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionMidGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Mid Goal";
  }

  public void ArmInOutHighGoal() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionHighGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "High Goal";

  }

  public void ArmInOutShelfHumanPL() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionShelfHumanPL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfMid";
  }

  public void ArmInOutZero() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionZero,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Zero";
  }

  public void ArmInOutGroundForward() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionGroundForward,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "GroundForward";

  }

  public void ArmInOutGroundBack() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.kArmInOutPositionGroundBack,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "GroundBack";
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Extend Position", m_ArmInOutEncoder.getPosition());
    SmartDashboard.putNumber("Extension Velocity", m_ArmInOutEncoder.getVelocity());
    SmartDashboard.putString("Arm In Out State", armInOutState);

    // This method will be called once per scheduler run
  }

}
