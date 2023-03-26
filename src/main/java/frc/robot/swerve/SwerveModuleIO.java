package frc.robot.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    // Logs Swerve Inputs from Implemented Hardware or Sim IO
    @AutoLog
    public static class SwerveModuleIOinputs {

        public double drivePositionRad = 0;
        public double driveVelocityRadPerSec = 0;
        public double driveVolts = 0;
        public double[] driveAmps = new double[] {};
        public double[] driveTempCelsius = new double[] {};

        public double turnAbsolutePositionRad = 0;
        public double turnPositionRad = 0;
        public double turnVelocityRadPerSec = 0;
        public double turnVolts = 0;
        public double[] turnAmps = new double[] {};
        public double[] turnTempCelsius = new double[] {};

    }

    // Updates Module Inputs
    public default void updateInputs(SwerveModuleIOinputs inputs) {
    }

    // Sets Volts (Does not mean Drive is controlled by volts, this is simply
    // communicating to the motors how many volts they need to be at to be at the
    // set velocity)
    public default void setDriveVolts(double volts) {
    }

    public default void setTurnVolts(double volts) {
    }

    // Sets Brake Mode
    public default void setDriveBrake(boolean brake) {
    }

    public default void setTurnBrake(boolean brake) {
    }

}
