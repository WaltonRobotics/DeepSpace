package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

public class MotorTestCommand extends Command {

    private final double speed = .25;
    private Timer timer;

    @Override
    protected void initialize() {
        drivetrain.setSpeeds(0, 0);
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void execute() {
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
    protected boolean isFinished() {
        drivetrain.reset();
        return timer.hasPeriodPassed(1.5);
    }
}
