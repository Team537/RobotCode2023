import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class ConstantsFactoryTest {

    @Test
    void testLoadConstants() {
        ConstantsFactory factory = new ConstantsFactory("src/java/frc/robot/resources/driveConstants.yaml");
        Constants myConstants = factory.getConstants(Constants.class);

        assertEquals("aName", myConstants.getName());
    }
}
