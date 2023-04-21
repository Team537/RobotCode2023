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
  private CANSparkMax m_ArmPivot = new CANSparkMax(Constants.ArmPivotConstants.kArmPivot1, MotorType.kBrushless);
  private SparkMaxPIDController m_ArmPivotPidController = m_ArmPivot.getPIDController();
  private RelativeEncoder m_ArmPivotEncoder = m_ArmPivot.getEncoder();
  private String armPivotState = "Default";

  /** Creates a new ArmPivot. */
  public ArmPivot() {

  }

  public void ArmPivotPidDefaults() {
    m_ArmPivotPidController.setP(Constants.SparkPIDFConstants.kP);
    m_ArmPivotPidController.setI(Constants.SparkPIDFConstants.kI);
    m_ArmPivotPidController.setD(Constants.SparkPIDFConstants.kD);
    m_ArmPivotPidController.setIZone(Constants.SparkPIDFConstants.kIz);
    m_ArmPivotPidController.setFF(Constants.SparkPIDFConstants.kFF);
    m_ArmPivotPidController.setOutputRange(Constants.SparkPIDFConstants.kMinOutput,
        Constants.SparkPIDFConstants.kMaxOutput);
    m_ArmPivotPidController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.kMaxVelocityArmPivot, 0);
    m_ArmPivotPidController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.kMinV, 0);
    m_ArmPivotPidController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.kMaxAccelArmPivot, 0);
    m_ArmPivotPidController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.kAllE, 0);
  }

  public void ArmPositionMidGoal() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionMidGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Mid Goal";

  }

  public void ArmPositionHighGoal() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionHighGoal,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "High Goal";

  }

  public void ArmPositionShelfHumanPL() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionShelfHumanPL,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "ShelfMid";

  }

  public void ArmPositionZero() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionZero,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Zero";

  }

  public void ArmPositionGroundForward() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionGroundForward,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "GroundForward";

  }

  public void ArmPositionMidDown() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionMidDown,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Test";

  }

  public void ArmPivotGroundBack() {
    ArmPivotPidDefaults();
    m_ArmPivotPidController.setReference(Constants.ArmPivotConstants.kArmPivotPositionGroundBack,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "GroundBack";
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pivot Position", m_ArmPivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Velocity", m_ArmPivotEncoder.getVelocity());
    SmartDashboard.putString("Arm Pivot State", armPivotState);
    // This method will be called once per scheduler run
  }
}
