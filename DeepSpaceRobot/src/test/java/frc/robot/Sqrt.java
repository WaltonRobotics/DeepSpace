package frc.robot;

public class Sqrt implements Transform {

    @Override
    public double transform(double input) {
        System.out.println("Input: " + input);
        double result = Math.signum(input) * Math.sqrt(Math.abs(input));
        System.out.println("Result: " + result);
        return result;
    }

}
