package frc.robot;

import frc.robot.command.teleop.Sqrt;
import frc.robot.command.teleop.Transform;
import junit.framework.Assert;

import org.junit.Test;

public class ControllerSqrtTest {
    @Test
    public void SqrtTest() {
        Transform sqrt = new Sqrt();

        Assert.assertEquals(3, sqrt.transform(input));
    }
}
