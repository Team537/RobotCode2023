package frc.robot.subsystems;

import frc.robot.Constants;
import java.beans.Transient;
import java.io.Console;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModulePosition;

import edu.wpi.first.wpilibj.simulation.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class SwerveModuleTest {
    private SwerveModule m_swerve;

    @BeforeEach // this method will run before each test
    void setup() {

    }

    @Test
    void testFrontLeftModule() {
        WPI_TalonFX frontLeft = new WPI_TalonFX(SwerveConstants.kFrontLeftTurn);
        WPI_TalonFX frontLeftDrive = new WPI_TalonFX(SwerveConstants.kFrontLeftDrive);

        m_swerve = new SwerveModule(
                ModulePosition.FRONT_LEFT,
                frontLeft,
                frontLeftDrive,
                SwerveConstants.kFrontLeftSRXMagCoder,
                SwerveConstants.kFrontLeftSRXMagCoderOffset,
                false);

        double randomPosition = Math.random() * 42;
        System.out.println(randomPosition);
        frontLeft.setSelectedSensorPosition(randomPosition);
        m_swerve.resetAngleToAbsolute();
        assertEquals(0.0, frontLeft.getSelectedSensorPosition());
    }
}
