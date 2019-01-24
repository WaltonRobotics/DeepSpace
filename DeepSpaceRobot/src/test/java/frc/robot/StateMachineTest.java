package test.java.frc.robot;

import org.junit.Test;

import frc.states.PeriodicTransition;
import frc.states.StateBuilder;

public class StateMachineTest {

    @Test
    public void testGenericTransition() {
        GenericTransition t = () -> null;

        assertEquals(t.run(), null);
    }

    @Test
    public void testPeriodicTransition() {
        PeriodicTransition t = () -> null;

        assertEquals(t.run(), null);
    }

    @Test
    public void testStateBuilder() {
        State secondState = new State(() -> null, () -> null, () -> null);
        State firstState = new State(() -> null, () -> secondState, () -> null);

        StateBuilder stateBuilder = new StateBuilder(firstState);

        stateBuilder.step();

        assertEquals(stateBuilder.getCurrentState(), secondState);
    }

}