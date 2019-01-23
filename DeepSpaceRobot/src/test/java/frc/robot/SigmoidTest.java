package frc.robot;

import frc.robot.command.teleop.Sigmoid;
import frc.robot.command.teleop.Transform;
import org.junit.Assert;
import org.junit.Test;

public class SigmoidTest {
    @Test
    public void Sig() {
        Transform sig = new Sigmoid();
        System.out.println(sig.transform(1));
        Assert.fail();
    }
}
