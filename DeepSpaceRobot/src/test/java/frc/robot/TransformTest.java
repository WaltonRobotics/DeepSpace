package frc.robot;

import frc.robot.command.teleop.Sqrt;
import frc.robot.command.teleop.BarryAllen;
import frc.robot.command.teleop.NormalSpeed;
import frc.robot.command.teleop.Transform;
import frc.robot.command.teleop.HalfSpeed;
import frc.robot.command.teleop.Sigmoid;

import org.junit.Assert;
import org.junit.Test;

public class TransformTest {

    @Test
    public void SqrtTest() {
        Transform sqrt = new Sqrt();

        Assert.assertEquals(3, sqrt.transform(9));
    }

    @Test
    public void NormalTest() {
        Transform norm = new NormalSpeed();

        Assert.assertEquals(1, norm.transform(1));
    }

    @Test
    public void SigmoidTest() {
        Transform sig = new Sigmoid();
        double input = 1;
        Assert.assertEquals(1 / (1 + Math.pow(Math.E, -input)), sig.transform(1));
    }

    @Test
    public void SuperSaiyanTest() {
        Transform goku = new BarryAllen();
        Assert.assertEquals(1, goku.transform(1));
    }

    @Test
    public void HalfSpeedTest() {
        Transform haf = new HalfSpeed();
        Assert.assertEquals(.0, haf.transform(1));
    }

    @Test
    public void TakeOverTest() {
        
    }

}
