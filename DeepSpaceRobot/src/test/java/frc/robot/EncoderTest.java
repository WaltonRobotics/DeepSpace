package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import org.junit.Assert;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class EncoderTest {

    private static Encoder encoder;
    private static double distancePerPulse;

    @BeforeClass
    public static void init() {
        distancePerPulse = 0.0002045;   // arbitrary DPP to be changed when we get a robot
        encoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
        encoder.setDistancePerPulse(distancePerPulse);
        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);
    }

    @Test
    public void encoderTest() {

        Assert.assertEquals(distancePerPulse, encoder.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderLeftTest()
    {
        Assert.assertEquals(encoder.getDistancePerPulse(), encoderLeft.getDistancePerPulse(), .005);
    }

    @Test
    public void encoderRightTest()
        {
        Assert.assertEquals(distancePerPulse, encoderRight.getDistancePerPulse(), .005);
    }
}
