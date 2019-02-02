package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

public class EncoderTestCommand extends Command {

    private static double distancePerPulse;

    @Override
    protected void initialize() {
        distancePerPulse = 0.0002045;
    }

    @Override
    protected void execute() {
        encoderLeft.setDistancePerPulse(distancePerPulse);
        encoderRight.setDistancePerPulse(distancePerPulse);

        assert (encoderLeft.getDistancePerPulse() == distancePerPulse);
        assert (encoderRight.getDistancePerPulse() == distancePerPulse);
    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        return true;
    }
}
