package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class EncoderTest {

    private static Encoder encoder;
    private static double distancePerPulse;

    @BeforeAll
    public static void init() {
        distancePerPulse = 0.0002045;   // arbitrary DPP to be changed when we get a robot

        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        encoder = new Encoder(-1, 1);
        encoder.setDistancePerPulse(distancePerPulse);
    }

    @Test
    public void encoderTest() {

        Assertions.assertEquals(distancePerPulse, encoder.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderLeftTest()
    {
        Assertions.assertEquals(distancePerPulse, encoderLeft.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderRightTest()
    {
        Assertions.assertEquals(distancePerPulse, encoderRight.getDistancePerPulse(), .005);
    }
}
