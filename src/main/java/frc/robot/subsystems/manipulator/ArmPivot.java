// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivot extends SubsystemBase {
  private CANSparkMax m_ArmPivot1 = new CANSparkMax(Constants.ArmPivotConstants.kArmPivot1, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerPivot1 = m_ArmPivot1.getPIDController();
  private RelativeEncoder m_encoderPivot1 = m_ArmPivot1.getEncoder();
  private String armPivotState = "Default";

  /** Creates a new ArmPivot. */
  public ArmPivot() {

  }

  public void ArmPositionMidGoal() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionMidGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Mid Goal";

  }

  public void ArmPositionHighGoal() {

    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionHighGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "High Goal";

  }

  public void ArmPositionShelfHumanPL() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionShelfHumanPL,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "ShelfMid";

  }

  public void ArmPositionZero() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionZero,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Zero";

  }

  public void ArmPositionGround() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionGround,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Ground";

  }

  public void ArmPositionMidDown() {
    m_pidControllerPivot1.setP(Constants.SparkPIDFConstants.kP);
    m_pidControllerPivot1.setI(Constants.SparkPIDFConstants.kI);
    m_pidControllerPivot1.setD(Constants.SparkPIDFConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.SparkPIDFConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.SparkPIDFConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPivotPositionMidDown,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Test";

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(" Pivot Position", m_encoderPivot1.getPosition());
    SmartDashboard.putNumber("Pivot Velocity", m_encoderPivot1.getVelocity());
    SmartDashboard.putString("Arm Pivot State", armPivotState);
    // This method will be called once per scheduler run
  }
}
