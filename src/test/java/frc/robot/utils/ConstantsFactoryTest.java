package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class ConstantsFactoryTest {

    @Test
    void testLoadConstants() {
        ConstantsFactory factory = new ConstantsFactory("src/main/resources/driveConstants.yaml");
        Constants myConstants = factory.getConstants();

        assertEquals("aName", myConstants.getName());
    }
}
