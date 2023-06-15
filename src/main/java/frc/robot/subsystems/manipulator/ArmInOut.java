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
  private CANSparkMax armExtensionMotor = new CANSparkMax(Constants.ArmInOutConstants.ARM_INOUT, MotorType.kBrushless);
  private SparkMaxPIDController armExtensionsPIDController = armExtensionMotor.getPIDController();
  private RelativeEncoder armExtensionEncoder = armExtensionMotor.getEncoder();
  private String armInOutState = "Default";
  /** Creates a new ArmInOut. */
  public ArmInOut() {
   
    armExtensionsPIDController.setP(Constants.SparkPIDFConstants.P);
    armExtensionsPIDController.setI(Constants.SparkPIDFConstants.I);
    armExtensionsPIDController.setD(Constants.SparkPIDFConstants.D);
    armExtensionsPIDController.setIZone(Constants.SparkPIDFConstants.IZONE);
    armExtensionsPIDController.setFF(Constants.SparkPIDFConstants.FF);
    armExtensionsPIDController.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    armExtensionsPIDController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    armExtensionsPIDController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    armExtensionsPIDController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL, 0);
    armExtensionsPIDController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);
  }

 

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Extend Position", armExtensionEncoder.getPosition());
    SmartDashboard.putNumber("Extension Velocity", armExtensionEncoder.getVelocity());
    SmartDashboard.putString("Arm In Out State", armInOutState);

    // This method will be called once per scheduler run
  }

  public boolean armMidGoal() {
   
    armExtensionsPIDController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_MID_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Mid Goal";
    return true;
  }

  public void armHighGoal() {
   
    armExtensionsPIDController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_HIGH_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "High Goal";

  }

  public void armShelfHumanPL() {
  
    armExtensionsPIDController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_SHELF_HUMANPL,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "ShelfMid";
  }

  public void armZero() {
   
    armExtensionsPIDController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_ZERO,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Zero";
  }

  public void armGround() {
    
    armExtensionsPIDController.setReference(Constants.ArmInOutConstants.ARM_INOUT_POS_GROUND,
        CANSparkMax.ControlType.kSmartMotion);
    armInOutState = "Ground";

  }

}
