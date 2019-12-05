package lib.utils;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import lib.controller.PIDController;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static lib.utils.ErrorMessages.requireNonNullParam;

/**
 * A command that controls an output with a {@link PIDController}.  Runs forever by default - to add
 * exit conditions and/or other behavior, subclass this class.  The controller calculation and
 * output are performed synchronously in the command's execute() method.
 */
public class PIDCommand extends Command {
    protected final PIDController m_controller;
    protected DoubleSupplier m_measurement;
    protected DoubleSupplier m_setpoint;
    protected DoubleConsumer m_useOutput;

    /**
     * Creates a new PIDCommand, which controls the given output with a PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpointSource    the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public PIDCommand(PIDController controller, DoubleSupplier measurementSource,
                      DoubleSupplier setpointSource, DoubleConsumer useOutput,
                      Subsystem... requirements) {
        requireNonNullParam(controller, "controller", "SynchronousPIDCommand");
        requireNonNullParam(measurementSource, "measurementSource", "SynchronousPIDCommand");
        requireNonNullParam(setpointSource, "setpointSource", "SynchronousPIDCommand");
        requireNonNullParam(useOutput, "useOutput", "SynchronousPIDCommand");

        m_controller = controller;
        m_useOutput = useOutput;
        m_measurement = measurementSource;
        m_setpoint = setpointSource;
    }

    /**
     * Creates a new PIDCommand, which controls the given output with a PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpoint          the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public PIDCommand(PIDController controller, DoubleSupplier measurementSource,
                      double setpoint, DoubleConsumer useOutput,
                      Subsystem... requirements) {
        this(controller, measurementSource, () -> setpoint, useOutput, requirements);
    }

    @Override
    public void initialize() {
        m_controller.reset();
    }

    @Override
    public void execute() {
        useOutput(m_controller.calculate(getMeasurement(), getSetpoint()));
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        useOutput(0);
    }

    /**
     * Sets the function that uses the output of the PIDController.
     *
     * @param useOutput The function that uses the output.
     */
    public final void setOutput(DoubleConsumer useOutput) {
        m_useOutput = useOutput;
    }

    /**
     * Returns the PIDController used by the command.
     *
     * @return The PIDController
     */
    public PIDController getController() {
        return m_controller;
    }

    /**
     * Sets the setpoint for the controller to track the given source.
     *
     * @param setpointSource The setpoint source
     */
    public void setSetpoint(DoubleSupplier setpointSource) {
        m_setpoint = setpointSource;
    }

    /**
     * Sets the setpoint for the controller to a constant value.
     *
     * @param setpoint The setpoint
     */
    public void setSetpoint(double setpoint) {
        setSetpoint(() -> setpoint);
    }

    /**
     * Sets the setpoint for the controller to a constant value relative (i.e. added to) the current
     * setpoint.
     *
     * @param relativeReference The change in setpoint
     */
    public void setSetpointRelative(double relativeReference) {
        setSetpoint(m_controller.getSetpoint() + relativeReference);
    }

    /**
     * Gets the setpoint for the controller.  Wraps the passed-in function for readability.
     *
     * @return The setpoint for the controller
     */
    private double getSetpoint() {
        return m_setpoint.getAsDouble();
    }

    /**
     * Gets the measurement of the process variable. Wraps the passed-in function for readability.
     *
     * @return The measurement of the process variable
     */
    private double getMeasurement() {
        return m_measurement.getAsDouble();
    }

    /**
     * Uses the output of the controller.  Wraps the passed-in function for readability.
     *
     * @param output The output value to use
     */
    private void useOutput(double output) {
        m_useOutput.accept(output);
    }
}
