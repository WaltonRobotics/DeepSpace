package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import org.junit.Assert;
import org.junit.Test;
import org.junit.Before;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class EncoderTest {

    private static Encoder encoder;
    private static double distancePerPulse;

    @Before
    public void init() {
        distancePerPulse = 0.0002045;   // arbitrary DPP to be changed when we get a robot

        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        encoder = new Encoder(4, 5);
        encoder.setDistancePerPulse(distancePerPulse);
    }

    @Test
    public void encoderTest() {

        Assert.assertEquals(distancePerPulse, encoder.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderLeftTest()
    {
        Assert.assertEquals(distancePerPulse, encoderLeft.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderRightTest()
    {
        Assert.assertEquals(distancePerPulse, encoderRight.getDistancePerPulse(), .005);
    }
}
