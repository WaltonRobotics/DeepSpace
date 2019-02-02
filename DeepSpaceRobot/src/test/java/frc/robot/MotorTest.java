package frc.robot;


import frc.team2974.robot.subsystems.Drivetrain;
import org.junit.Assert;
import org.junit.Test;

import static frc.team2974.robot.RobotMap.motorLeft;
import static frc.team2974.robot.RobotMap.motorRight;


public class MotorTest
{
    @Test
    public void MotorTests() {

        int speed = 1;
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.setSpeeds(speed, speed);
        System.out.println(motorLeft.get());
        System.out.println(motorRight.get());
        System.out.println(speed + "sped");

        Assert.assertEquals(speed, -motorLeft.get(), .005);
        System.out.println("Left motor test is running");
        Assert.assertEquals(speed, motorRight.get(), .005);
        System.out.println("Right motor test is running");
    }
}
