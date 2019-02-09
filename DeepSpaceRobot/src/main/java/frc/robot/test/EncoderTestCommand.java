package frc.robot.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

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
        distancePerPulse = drivetrain.getRobotConfig().getLeftEncoderConfig().getDistancePerPulse();
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void execute() {
        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        if ((encoderLeft.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with left encoder distance per pulse");
        {
            SmartDashboard.putBoolean("Check left encoder distance per pulse", false);
        }
        if ((encoderRight.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with right encoder distance per pulse");
        {
            SmartDashboard.putBoolean("Check right encoder distance per pulse", false);
        }

        /* Assertions to be implemented after we get a robot

        drivetrain.setSpeeds(.25, .25);

        if ((encoderLeft.get() != 0))
            throw new AssertionError();
        {
            SmartDashboard.putBoolean("Encoder left did not return the expected digit", false);
        }
        if ((encoderRight.get() != 0))
            throw new AssertionError();
        {
            SmartDashboard.putBoolean("Encoder right did not return the expected digit", false);
        }
*/
        SmartDashboard.putBoolean("Encoder tests passed", true);
        if (encoderLeft.getDistancePerPulse() != distancePerPulse) throw new AssertionError("Issue with the left encoder distance per pulse");
        if (encoderRight.getDistancePerPulse() != distancePerPulse) throw new AssertionError("Issue with the right encoder distance per pulse");
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        if(timer.hasPeriodPassed(3))
        {
            timer.stop();
            return true;
        }

        return false;
    }
}
