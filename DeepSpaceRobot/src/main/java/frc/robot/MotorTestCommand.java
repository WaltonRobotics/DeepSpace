package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.leftWheel;
import static frc.robot.RobotMap.rightWheel;

public class MotorTestCommand extends Command {

    private final double speed = .5;
    private Timer timer;

    public MotorTestCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        System.out.println("Testing the motor");
        // requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        drivetrain.setSpeeds(0, 0);
        timer = new Timer();
        System.out.println("up here");
    }

    @Override
    protected void execute() {
        System.out.println("runnign");
        timer.start();
        drivetrain.setSpeeds(speed, speed);
        assert(speed == leftWheel.get());
        assert(speed == rightWheel.get());
        if(timer.hasPeriodPassed(1)){
            timer.stop();
            drivetrain.setSpeeds(0,0);
        }
    }

    @Override
    protected boolean isFinished() {
        System.out.println("Is finished?");
        drivetrain.reset();
        if(timer.hasPeriodPassed(1)){
            timer.stop();
            return true;
        }
        return false;
    }

    @Override
    protected void end() {
        drivetrain.setSpeeds(0,0);
    }
}
