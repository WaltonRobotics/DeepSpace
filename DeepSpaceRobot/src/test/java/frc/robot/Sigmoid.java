package frc.robot;


public class Sigmoid implements Transform {

    @Override
    public double transform(double input) {
        return 1 / (1 + Math.pow(Math.E, -input));
    }
    // e = 2.7182818285
}