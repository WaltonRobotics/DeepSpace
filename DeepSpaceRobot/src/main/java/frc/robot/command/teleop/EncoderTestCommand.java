package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import static frc.robot.Config.Hardware.DISTANCE_PER_PULSE;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class EncoderTestCommand extends Command {

    private static double distancePerPulse;
    private Timer timer;

    public EncoderTestCommand(){
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        distancePerPulse = DISTANCE_PER_PULSE;
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void execute() {
        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        assert (encoderLeft.getDistancePerPulse() == 2) : "something happened";
        assert (encoderRight.getDistancePerPulse() == distancePerPulse);
        System.out.println(timer.get());
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        if(timer.hasPeriodPassed(1)){
            return true;
        }
        return false;
    }
}
