// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperIntake extends SubsystemBase {
  CANSparkMax m_Gripper = new CANSparkMax(GripperConstants.kGripper, MotorType.kBrushless);
  CANSparkMax m_Gripper2 = new CANSparkMax(GripperConstants.kGripper2, MotorType.kBrushless);
  UsbCamera cam =  new UsbCamera("Usb Camera 0", 0);
  MjpegServer mjep = new MjpegServer("server_1", 1181);

  

  /** Creates a new GripperIntake. */
  public GripperIntake() {
    CameraServer.startAutomaticCapture(0);
    mjep.setSource(cam);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void GripperIn() {
    m_Gripper.set(0.5);
    m_Gripper2.set(-0.5);
  }
  public void GripperStop() {
    m_Gripper.set(0);
    m_Gripper2.set(0);
  }
  public void GripperOut() {
    m_Gripper.set(-0.5);
    m_Gripper2.set(0.5);
  }

}
