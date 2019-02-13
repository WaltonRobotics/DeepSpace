package frc.robot.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TestCommand;


import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

public class MotorTestCommand extends TestCommand {

    private final double speed = .5;
    private Timer timer;

    @Override
    protected void initializeTest() {
        drivetrain.reset();
        drivetrain.setSpeeds(0, 0);
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void executeTest() {
        drivetrain.setSpeeds(speed, speed);
        if ((speed != -leftWheel.get()))
            throw new AssertionError("Left wheel speed issue");
        {
            SmartDashboard.putBoolean("Check left wheel", false);
        }
        if ((speed != rightWheel.get()))
            throw new AssertionError("Right wheel speed issue");
        {
            SmartDashboard.putBoolean("Check right wheel", false);
        }
    }

    @Override
    protected void endTest() {

    }

    @Override
    protected boolean isFinished() {
        return timer.hasPeriodPassed(1.5);
    }
}
