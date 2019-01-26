package frc.robot;


public class HalfSpeed implements Transform {

    @Override
    public double transform(double input) {
        return input = input / 2;
    }

}