package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


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
        if ((speed != -leftWheel.get())) throw new AssertionError("Issue with the left wheel speed");
        if ((speed != rightWheel.get())) throw new AssertionError("Issue with the right wheel speed");
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
