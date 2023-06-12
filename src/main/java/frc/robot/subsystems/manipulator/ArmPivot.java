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
  private CANSparkMax armPivot = new CANSparkMax(Constants.ArmPivotConstants.ARM_PIVOT, MotorType.kBrushless);
  private SparkMaxPIDController armPivotPIDController = armPivot.getPIDController();
  private RelativeEncoder armPivotEncoder = armPivot.getEncoder();
  private String armPivotState = "Default";

  /** Creates a new ArmPivot. */
  public ArmPivot() {
    armPivotPIDController.setP(Constants.SparkPIDFConstants.P);
    armPivotPIDController.setI(Constants.SparkPIDFConstants.I);
    armPivotPIDController.setD(Constants.SparkPIDFConstants.D);
    armPivotPIDController.setIZone(Constants.SparkPIDFConstants.IZONE);
    armPivotPIDController.setFF(Constants.SparkPIDFConstants.FF);
    armPivotPIDController.setOutputRange(Constants.SparkPIDFConstants.MIN_OUTPUT,
        Constants.SparkPIDFConstants.MAX_OUTPUT);
    armPivotPIDController.setSmartMotionMaxVelocity(Constants.SparkPIDFConstants.MAX_VELOCITY, 0);
    armPivotPIDController.setSmartMotionMinOutputVelocity(Constants.SparkPIDFConstants.MIN_VELOCITY, 0);
    armPivotPIDController.setSmartMotionMaxAccel(Constants.SparkPIDFConstants.MAX_ACCEL_ARM_PIVOT, 0);
    armPivotPIDController.setSmartMotionAllowedClosedLoopError(Constants.SparkPIDFConstants.ALL_E, 0);

  }

  public void ArmPositionMidGoal() {
   
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_MID_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Mid Goal";

  }

  public void ArmPositionHighGoal() {

  
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_HIGH_GOAL,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "High Goal";

  }

  public void ArmPositionShelfHumanPL() {
  
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_SHELF_HUMAN_PL,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "ShelfMid";

  }

  public void ArmPositionZero() {
  
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_ZERO,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Zero";

  }

  public void ArmPositionGround() {
   
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_GROUND,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Ground";

  }

  public void ArmPositionMidDown() {
   
    armPivotPIDController.setReference(Constants.ArmPivotConstants.ARM_PIVOT_POS_MID_DOWN,
        CANSparkMax.ControlType.kSmartMotion);
    armPivotState = "Test";

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(" Pivot Position", armPivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Velocity", armPivotEncoder.getVelocity());
    SmartDashboard.putString("Arm Pivot State", armPivotState);
    // This method will be called once per scheduler run
  }
}
