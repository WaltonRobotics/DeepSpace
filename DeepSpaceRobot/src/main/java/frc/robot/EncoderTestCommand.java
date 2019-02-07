package frc.robot;

import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class EncoderTestCommand extends Command {

    private static double distancePerPulse;

    @Override
    protected void initialize() {
        requires(drivetrain);
        distancePerPulse = 0.0002045;
        System.out.println("hello");
    }

    @Override
    protected void execute() {
        System.out.println("an actual word");

        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        assert (encoderLeft.getDistancePerPulse() == distancePerPulse);
        assert (encoderRight.getDistancePerPulse() == distancePerPulse);
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        return false;
    }
}
