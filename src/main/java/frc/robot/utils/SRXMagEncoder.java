// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class SRXMagEncoder extends DutyCycleEncoder {
  /** Creates a new SRXMagEncoder. */
  private double offset;

  public SRXMagEncoder(DutyCycle cycle, double offset) {

    super(cycle);
    this.offset = offset;

  }

  @Override
  public double getDistance() {
      return super.getDistance() + Math.toDegrees(this.offset);
  }

 
}
