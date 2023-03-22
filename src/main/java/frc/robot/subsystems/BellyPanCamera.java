// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BellyPanCamera extends SubsystemBase {
  /** Creates a new GripperCamera. */

  public BellyPanCamera() {

    UsbCamera cam1 = new UsbCamera("Usb Camera 1", 0);
    MjpegServer mjep1 = new MjpegServer("server_2", 1182);
    CameraServer.startAutomaticCapture(0);
    mjep1.setSource(cam1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
