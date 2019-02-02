package frc.robot;

public class Sqrt implements Transform {

    @Override
    public double transform(double input) {
        return Math.signum(input) * Math.sqrt(Math.abs(input));;
    }

}
