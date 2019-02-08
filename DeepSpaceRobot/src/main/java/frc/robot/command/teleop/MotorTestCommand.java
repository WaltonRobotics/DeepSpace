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
        assert(speed == leftWheel.get());
        assert(speed == rightWheel.get());
        System.out.println(timer.get());
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
