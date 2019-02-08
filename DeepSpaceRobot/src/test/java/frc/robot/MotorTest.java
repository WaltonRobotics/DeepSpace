package frc.robot;


import frc.robot.subsystem.Drivetrain;
import org.junit.Assert;
import org.junit.Test;
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;


public class MotorTest
{
    @Test
    public void MotorTests() {
        int speed = 1;
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.setSpeeds(speed, speed);
        System.out.println(leftWheel.get());
        System.out.println(rightWheel.get());

        Assert.assertEquals(speed, -leftWheel.get(), .005);
        Assert.assertEquals(speed, rightWheel.get(), .005);
    }
}
