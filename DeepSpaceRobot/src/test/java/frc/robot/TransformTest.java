package frc.robot;

import frc.robot.command.teleop.TransformMap;
import org.junit.Assert;
import org.junit.Test;

public class TransformTest {

    @Test
    public void SqrtTest() {
        Transform sqrt = new Sqrt();
        Assert.assertEquals(3, (int) sqrt.transform(9),0);
    }

    @Test
    public void NormalTest() {
        Transform norm = new NormalSpeed();

        Assert.assertEquals(1, norm.transform(1), 0);
    }

    @Test
    public void SigmoidTest() {
        Transform sig = new Sigmoid();
        double input = 1;
        Assert.assertEquals(1 / (1 + Math.pow(Math.E, -input)) - 0.5, sig.transform(1), 0);
    }

    @Test
    public void SuperSaiyanTest() {
        Transform goku = (Transform) new BarryAllen();
        Assert.assertEquals(Math.pow(9, 10), goku.transform(9), 0);
    }

    @Test
    public void HalfSpeedTest() {
        HalfSpeed haf = new HalfSpeed();
        Assert.assertEquals(.5, haf.transform(1), 0);
    }

    @Test
    public void TransformMapTest() {
        TransformMap.clearTransforms();
        TransformMap.addTransform(new frc.robot.command.teleop.HalfSpeed());
        TransformMap.addTransform(new frc.robot.command.teleop.BarryAllen());

        Assert.assertEquals(TransformMap.calculate(0.5), Math.pow(0.5 / 2, 2), 0);

        TransformMap.clearTransforms();
    }

    @Test
    public void TakeOverTest() {
        DriverTakeOverTest.activity = new double[]{1304701232, 123412487};
        DriverTakeOverTest.noActivity = new double[]{0, 0};
        DriverTakeOverTest.isTakingOver();
        Assert.assertEquals(true, DriverTakeOverTest.getIsTakingOver());
    }
}
