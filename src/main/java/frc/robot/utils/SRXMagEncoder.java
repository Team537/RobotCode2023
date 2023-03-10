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
    setDistancePerRotation(360);
    double angle =  (360 * super.getAbsolutePosition()) - offset;

    System.out.println(angle);
  }

  
  public double getEncoderDistance() {
    
    double angle =  (360 * super.getAbsolutePosition()) - offset;


      return  angle;//Math.toDegrees(this.offset); // this is already in degrees
      
  }

  public double getAbsoluteAngle() {  
    double angle =  Math.toRadians(this.getEncoderDistance());
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }

    return angle;
}


public void resetAngleToAbsolute(double zero){
  
  super.setPositionOffset(zero);


}
}
