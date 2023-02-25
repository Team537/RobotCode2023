// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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




  /** Creates a new ArmPivot. */
  public  ArmPivot() {

  }

  public void ArmPositionDown() {
    m_pidControllerPivot1.setP(Constants.ArmPivotConstants.kP);
    m_pidControllerPivot1.setI(Constants.ArmPivotConstants.kI);
    m_pidControllerPivot1.setD(Constants.ArmPivotConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.ArmPivotConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.ArmPivotConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.ArmPivotConstants.kMinOutput, Constants.ArmPivotConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.ArmPivotConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.ArmPivotConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.ArmPivotConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.ArmPivotConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPositionDown, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putBoolean("Arm Down", true);
    SmartDashboard.putBoolean("Arm Up", false);

   
  }

  public void ArmPositionUp() {
    m_pidControllerPivot1.setP(Constants.ArmPivotConstants.kP);
    m_pidControllerPivot1.setI(Constants.ArmPivotConstants.kI);
    m_pidControllerPivot1.setD(Constants.ArmPivotConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.ArmPivotConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.ArmPivotConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.ArmPivotConstants.kMinOutput, Constants.ArmPivotConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.ArmPivotConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.ArmPivotConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.ArmPivotConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.ArmPivotConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPositionUp, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putBoolean("Arm Down", false);
    SmartDashboard.putBoolean("Arm Up", true);

    
  }
  public void ArmPositionMiddle() {
    m_pidControllerPivot1.setP(Constants.ArmPivotConstants.kP);
    m_pidControllerPivot1.setI(Constants.ArmPivotConstants.kI);
    m_pidControllerPivot1.setD(Constants.ArmPivotConstants.kD);
    m_pidControllerPivot1.setIZone(Constants.ArmPivotConstants.kIz);
    m_pidControllerPivot1.setFF(Constants.ArmPivotConstants.kFF);
    m_pidControllerPivot1.setOutputRange(Constants.ArmPivotConstants.kMinOutput, Constants.ArmPivotConstants.kMaxOutput);
    m_pidControllerPivot1.setSmartMotionMaxVelocity(Constants.ArmPivotConstants.kMaxV, 0);
    m_pidControllerPivot1.setSmartMotionMinOutputVelocity(Constants.ArmPivotConstants.kMinV, 0);
    m_pidControllerPivot1.setSmartMotionMaxAccel(Constants.ArmPivotConstants.kMaxA, 0);
    m_pidControllerPivot1.setSmartMotionAllowedClosedLoopError(Constants.ArmPivotConstants.kAllE, 0);
    m_pidControllerPivot1.setReference(Constants.ArmPivotConstants.kArmPositionMiddle, CANSparkMax.ControlType.kSmartMotion);
    // SmartDashboard.putBoolean("Arm Down", true);
    // SmartDashboard.putBoolean("Arm Up", false);

   
  }

  

  @Override
  public void periodic() {

    SmartDashboard.putNumber(" Pivot Position", m_encoderPivot1.getPosition());
    SmartDashboard.putNumber("Pivot Velocity",m_encoderPivot1.getVelocity());
    // This method will be called once per scheduler run
  }
}
