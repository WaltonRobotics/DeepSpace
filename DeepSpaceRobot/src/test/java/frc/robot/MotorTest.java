package frc.robot;

import frc.robot.Command.teleop.Drive;
import frc.robot.Subsystem.Drivetrain;
import org.junit.Assert;
import org.junit.Before;
import org.junit.jupiter.api.Test;

import static frc.robot.Robot.drivetrain;

public class MotorTest
{
    @Test
    public void move()
    {
        int speeds = 5;
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.setSpeeds(speeds, speeds);
        Assert.assertEquals(speeds, drivetrain.getMaxVelocity(), 0);
    }
}
