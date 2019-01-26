package frc.robot.command.teleop;

public class Sqrt implements Transform{

    @Override
    public double transform(double input) {
        return Math.sqrt(Math.abs(input));
    }

}
