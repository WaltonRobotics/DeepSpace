package frc.robot.command.teleop.util;

public class HalfSpeed implements Transform {

    @Override
    public double transform(double input) {
        return input / 2;
    }

}