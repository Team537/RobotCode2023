package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map.Entry;
import java.util.TreeMap;


public class RotationSequence {
  private final TreeMap<Double, Rotation2d> sequence = new TreeMap<>();

  /** Constructs a rotation sequence from a series of timed rotation positions. */
  public RotationSequence(TreeMap<Double, Rotation2d> sequence) {
    this.sequence.putAll(sequence);
  }


  public State sample(double timeSeconds) {
    double positionRadians;
    double velocityRadiansPerSec;

    Entry<Double, Rotation2d> lastPoint = sequence.floorEntry(timeSeconds);
    Entry<Double, Rotation2d> nextPoint = sequence.higherEntry(timeSeconds);
    if (lastPoint == null && nextPoint == null) { // No points in sequence
      positionRadians = 0.0;
      velocityRadiansPerSec = 0.0;
    } else if (lastPoint == null) { // Before start of sequence
      positionRadians = nextPoint.getValue().getRadians();
      velocityRadiansPerSec = 0.0;
    } else if (nextPoint == null) { // Before end of sequence
      positionRadians = lastPoint.getValue().getRadians();
      velocityRadiansPerSec = 0.0;
    } else {
      double accelerationRadiansPerSec2 =
          (4 * nextPoint.getValue().minus(lastPoint.getValue()).getRadians())
              / Math.pow(nextPoint.getKey() - lastPoint.getKey(), 2);

      if (timeSeconds < (nextPoint.getKey() + lastPoint.getKey()) / 2) { // Accelerating
        positionRadians =
            lastPoint.getValue().getRadians()
                + ((accelerationRadiansPerSec2 / 2)
                    * Math.pow(timeSeconds - lastPoint.getKey(), 2));
        velocityRadiansPerSec = (timeSeconds - lastPoint.getKey()) * accelerationRadiansPerSec2;

      } else { // Decelerating
        positionRadians =
            nextPoint.getValue().getRadians()
                - ((accelerationRadiansPerSec2 / 2)
                    * Math.pow(timeSeconds - nextPoint.getKey(), 2));
        velocityRadiansPerSec = (nextPoint.getKey() - timeSeconds) * accelerationRadiansPerSec2;
      }
    }

    // Keep position within  range
    while (positionRadians > Math.PI) {
      positionRadians -= Math.PI * 2;
    }
    while (positionRadians < -Math.PI) {
      positionRadians += Math.PI * 2;
    }

    return new State(new Rotation2d(positionRadians), velocityRadiansPerSec);
  }

  
  public static class State {
    public Rotation2d position;
    public double velocityRadiansPerSec;

    public State() {
      position = new Rotation2d();
      velocityRadiansPerSec = 0.0;
    }


    public State(Rotation2d position, double velocityRadiansPerSec) {
      this.position = position;
      this.velocityRadiansPerSec = velocityRadiansPerSec;
    }
  }
}
