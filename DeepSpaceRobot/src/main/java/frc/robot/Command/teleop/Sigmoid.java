package frc.robot.command.teleop;


public class Sigmoid implements Transform{

    @Override
    public double transform(double input) {
        return (1 / (1 + Math.pow(Math.E, -input))) - 0.5;
    }
    // e = 2.7182818285
}