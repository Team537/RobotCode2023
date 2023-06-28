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

  private CANSparkMax m_ArmInOut = new CANSparkMax(Constants.ArmInOutConstants.ARM_INOUT, MotorType.kBrushless);
  private SparkMaxPIDController m_ArmInOutPidController = m_ArmInOut.getPIDController();
  private RelativeEncoder m_ArmInOutEncoder = m_ArmInOut.getEncoder();
  private String armInOutState = "Default";
  double ArmInOutPositionEnter = 0;

  public void ArmInOutPidDefaults() {
    m_ArmInOutPidController.setP(Constants.SparkPIDFConstants.P);
    m_ArmInOutPidController.setI(Constants.SparkPIDFConstants.I);
    m_ArmInOutPidController.setD(Constants.SparkPIDFConstants.D);
    m_ArmInOutPidController.setIZone(Constants.SparkPIDFConstants.IZONE);
    m_ArmInOutPidController.setFF(Constants.SparkPIDFConstants.FF);
    m_ArmInOutPidController.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    m_ArmInOutPidController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    m_ArmInOutPidController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
    m_ArmInOutPidController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    m_ArmInOutPidController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);

  }

  public void ArmInOutMidGoal() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_MID_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Mid Goal";
  }

  public void ArmInOutHighGoal() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_HIGH,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "High Goal";

  }

  public void ArmInOutShelfHumanPL() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_SHELF_HUMANPL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfMid";
  }

  public void ArmInOutGroundForward() {
    ArmInOutPidDefaults();
    m_ArmInOutPidController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_GROUND,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "GroundForward";

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Extend Position", m_ArmInOutEncoder.getPosition());
    SmartDashboard.putNumber("Extension Velocity", m_ArmInOutEncoder.getVelocity());
    SmartDashboard.putString("Arm In Out State", armInOutState);
    ArmInOutPositionEnter = SmartDashboard.getNumber("arm inout get set pos", ArmInOutPositionEnter);
    SmartDashboard.putNumber("arm inout get set pos", ArmInOutPositionEnter);
    // This method will be called once per scheduler run
  }

  // used for testing, set in out to position from smart dashboard
  public void ArmInOutSetSmartDash() {
    ArmInOutPidDefaults();
    armInOutUpdatePosition();
    m_ArmInOutPidController.setReference(ArmInOutPositionEnter,
        CANSparkMax.ControlType.kSmartMotion);

  }

  public void armInOutUpdatePosition() {
    ArmInOutPositionEnter = SmartDashboard.getNumber("arm inout get set pos",
        ArmInOutPositionEnter);

  }
}
