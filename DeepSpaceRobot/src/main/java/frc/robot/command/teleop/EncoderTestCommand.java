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
        distancePerPulse = DISTANCE_PER_PULSE;
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
            SmartDashboard.putBoolean("Test failed, check left encoder", false);
        }
        if ((encoderRight.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with right encoder distance per pulse");
        {
            SmartDashboard.putBoolean("Test failed, check right encoder", false);
        }

        SmartDashboard.putBoolean("Encoder tests passed", true);
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        return timer.hasPeriodPassed(1);
    }
}
