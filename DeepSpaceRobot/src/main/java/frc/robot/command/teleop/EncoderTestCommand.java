package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        drivetrain.reset();
        System.out.println(encoderLeft.getDistancePerPulse());
        System.out.println(encoderRight.getDistancePerPulse());
        if ((encoderRight.get() != 0))
            throw new AssertionError("Failed to reset right encoder" + encoderRight.get());
        if ((encoderLeft.get() != 0))
            throw new AssertionError("Failed to reset left encoder" + encoderLeft.get());


        distancePerPulse = DISTANCE_PER_PULSE;
        timer = new Timer();
        timer.start();

        if ((encoderLeft.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with left encoder distance per pulse");
        if ((encoderRight.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with right encoder distance per pulse");
    }

    @Override
    protected void execute() {


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
    }

    @Override
    protected boolean isFinished() {
        if(timer.hasPeriodPassed(1))
        {
            timer.stop();
            return true;
        }
        return false;
    }
}
