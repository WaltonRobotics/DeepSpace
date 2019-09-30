package lib.motionControl;

/**
 * @author Vikas Malepati, Walton Robotics
 */
public class RamseteTuple {

    // Linear velocity
    public double v;

    // Angular velocity
    public double omega;

    public RamseteTuple(double v, double omega) {
        this.v = v;
        this.omega = omega;
    }

}
