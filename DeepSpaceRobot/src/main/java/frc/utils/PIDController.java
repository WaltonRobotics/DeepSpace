package frc.utils;

public class PIDController {

    private double m_P;                                 // factor for "proportional" control
    private double m_I;                                 // factor for "integral" control
    private double m_D;                                 // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;   // |maximum output|
    private double m_minimumOutput = -1.0;  // |minimum output|
    private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
    private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;              //is the pid controller enabled
    private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;          //the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;

    public PIDController(double Kp, double Ki, double Kd) {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    private void calculate() {
        int     sign = 1;

        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    else
                        m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }  

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;
        }
    }

}