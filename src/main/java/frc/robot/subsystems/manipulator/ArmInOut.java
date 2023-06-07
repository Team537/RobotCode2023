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

  private CANSparkMax m_extension = new CANSparkMax(Constants.ArmInOutConstants.kArmInOut, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerExtension = m_extension.getPIDController();
  private RelativeEncoder m_encoderExtension = m_extension.getEncoder();
  private String armInOutState = "Default";

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Extend Position", m_encoderExtension.getPosition());
    SmartDashboard.putNumber("Extension Velocity", m_encoderExtension.getVelocity());
    SmartDashboard.putString("Arm In Out State", armInOutState);

    // This method will be called once per scheduler run
  }

  public boolean armMidGoal() {
    m_pidControllerExtension.setP(Constants.SparkPIDFConstants.P);
    m_pidControllerExtension.setI(Constants.SparkPIDFConstants.I);
    m_pidControllerExtension.setD(Constants.SparkPIDFConstants.D);
    m_pidControllerExtension.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_pidControllerExtension.setFF(Constants.SparkPIDFConstants.FF);
    m_pidControllerExtension.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_MID_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Mid Goal";
    return true;
  }

  public void armHighGoal() {
    m_pidControllerExtension.setP(Constants.SparkPIDFConstants.P);
    m_pidControllerExtension.setI(Constants.SparkPIDFConstants.I);
    m_pidControllerExtension.setD(Constants.SparkPIDFConstants.D);
    m_pidControllerExtension.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_pidControllerExtension.setFF(Constants.SparkPIDFConstants.FF);
    m_pidControllerExtension.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_HIGH_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "High Goal";

  }

  public void armShelfHumanPL() {
    m_pidControllerExtension.setP(Constants.SparkPIDFConstants.P);
    m_pidControllerExtension.setI(Constants.SparkPIDFConstants.I);
    m_pidControllerExtension.setD(Constants.SparkPIDFConstants.D);
    m_pidControllerExtension.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_pidControllerExtension.setFF(Constants.SparkPIDFConstants.FF);
    m_pidControllerExtension.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_SHELF_HUMANPL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfMid";
  }

  public void armZero() {
    m_pidControllerExtension.setP(Constants.SparkPIDFConstants.P);
    m_pidControllerExtension.setI(Constants.SparkPIDFConstants.I);
    m_pidControllerExtension.setD(Constants.SparkPIDFConstants.D);
    m_pidControllerExtension.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_pidControllerExtension.setFF(Constants.SparkPIDFConstants.FF);
    m_pidControllerExtension.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_ZERO,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Zero";
  }

  public void armGround() {
    m_pidControllerExtension.setP(Constants.SparkPIDFConstants.P);
    m_pidControllerExtension.setI(Constants.SparkPIDFConstants.I);
    m_pidControllerExtension.setD(Constants.SparkPIDFConstants.D);
    m_pidControllerExtension.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_pidControllerExtension.setFF(Constants.SparkPIDFConstants.FF);
    m_pidControllerExtension.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_pidControllerExtension.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_pidControllerExtension.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    m_pidControllerExtension.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_pidControllerExtension.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_GROUND,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Ground";

  }

}
