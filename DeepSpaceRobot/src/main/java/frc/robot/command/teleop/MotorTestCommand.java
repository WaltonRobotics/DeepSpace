package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

public class MotorTestCommand extends Command {

    private final double speed = .5;
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
            SmartDashboard.putBoolean("Left wheel test failed", false);
        }
        if ((speed != rightWheel.get()))
            throw new AssertionError("Right wheel speed issue");
        {
            SmartDashboard.putBoolean("Right wheel test failed", false);
        }

        SmartDashboard.putBoolean("Wheel tests passed", true);
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        if(timer.hasPeriodPassed(1)){
            timer.stop();
            return true;
        }
        return false;
    }
}
