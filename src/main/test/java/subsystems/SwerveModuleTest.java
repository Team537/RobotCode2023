import java.beans.Transient;
import java.io.Console;

import edu.wpi.first.wpilibj.simulation.WPI_TalonFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveModuleTest {
    private SwerveModule m_swerve;

    @BeforeEach // this method will run before each test
    void setup() {

    }

    @Test
    void testFrontLeftModule() {
        com.ctre.phoenix.motorcontrol.can.WPI_TalonFX frontLeft = new WPI_TalonFX(SwerveConstants.kFrontLeftTurn);
        com.ctre.phoenix.motorcontrol.can.WPI_TalonFX frontLeftDrive = new WPI_TalonFX(SwerveConstants.kFrontLeftDrive);

        m_swerve = new SwerveModule(
                ModulePosition.FRONT_LEFT,
                frontLeft,
                frontLeftDrive,
                SwerveConstants.kFrontLeftSRXMagCoder,
                SwerveConstants.kFrontLeftSRXMagCoderOffset, false);

        double randomPosition = Math.random() * 42;
        System.out.println(randomPosition);
        frontLeft.setSelectedSensorPosition(randomPosition);
        m_swerve.resetAngleToAbsolute();
        assertEquals(0.0, frontLeft.getSelectedSensorPosition());
    }
}
