// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperCamera extends SubsystemBase {
  /** Creates a new GripperCamera. */

  UsbCamera cam =  new UsbCamera("Usb Camera 0", 0);
  MjpegServer mjep = new MjpegServer("server_1", 1181);
  
  public GripperCamera() {
    CameraServer.startAutomaticCapture(0);
    // mjep.setSource(cam);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
