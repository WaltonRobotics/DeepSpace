package frc.robot.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.TestCommand;

import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class EncoderTestCommand extends TestCommand {

    private static double distancePerPulse;
    private Timer timer;

    public EncoderTestCommand(){
        requires(Robot.drivetrain);
    }

    protected void initializeTest() {
        distancePerPulse = drivetrain.getRobotConfig().getLeftEncoderConfig().getDistancePerPulse();
        timer = new Timer();
        timer.start();
    }

    @Override
    protected void executeTest() {

        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        if ((encoderLeft.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with left encoder distance per pulse");
        if ((encoderRight.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with right encoder distance per pulse");

    }

  @Override
  protected void endTest() {

  }

  @Override
    protected boolean isFinished() {
        if(timer.hasPeriodPassed(1.5))
        {
            timer.stop();
            return true;
        }
        return false;
    }
}
