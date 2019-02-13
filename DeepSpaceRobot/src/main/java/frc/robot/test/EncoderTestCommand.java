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

    @Override
    protected void initializeTest() {
        distancePerPulse = drivetrain.getRobotConfig().getLeftEncoderConfig().getDistancePerPulse();
        System.out.println(distancePerPulse);
        timer = new Timer();
        timer.start();
        drivetrain.reset();

        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);
    }

    @Override
    protected void executeTest() {

        if ((encoderLeft.getDistancePerPulse() != distancePerPulse - 1))
            throw new AssertionError("Issue with left encoder distance per pulse");

        if ((encoderRight.getDistancePerPulse() != distancePerPulse))
            throw new AssertionError("Issue with right encoder distance per pulse");
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
  protected void endTest() {

  }

  @Override
    protected boolean isFinished() {
        if(timer.hasPeriodPassed(1.5))
        {
            timer.stop();
            drivetrain.setSpeeds(0, 0);
            return true;
        }

        return false;
    }
}
