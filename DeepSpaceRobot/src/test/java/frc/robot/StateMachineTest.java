package frc.robot;

import frc.states.State;
import org.junit.Test;

import frc.states.GenericTransition;
import frc.states.PeriodicTransition;
import frc.states.StateBuilder;
import frc.utils.Logger;

import static org.junit.Assert.*;

public class StateMachineTest {

    public static int periodicCounter = 0;
    public static int periodicNumberOfTimes = 5;
    public static Logger testLogger = new Logger();

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

    @Test
    public void testMultipleStateTransitions() {
        StateMachineTest.periodicCounter = 0;

        class TestFinish implements GenericTransition {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Finish test successful!");

                return null;
            }

        }

        class TestPeriodic implements PeriodicTransition {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Periodic test successful!");

                StateMachineTest.periodicCounter++;

                if (StateMachineTest.periodicCounter >= StateMachineTest.periodicNumberOfTimes) {
                    return new State(new TestFinish(), () -> null, () -> null);
                }

                return new State(new TestPeriodic());
            }
        }

        class TestInitialize implements GenericTransition {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Initialization test successful!");

                return new State(new TestPeriodic());
            }

        }

        State firstState = new State(new TestInitialize(), new TestPeriodic(), null);

        StateBuilder builder = new StateBuilder(firstState);

        assertTrue(builder.getCurrentState() == firstState);

        for (int i = 0; i < StateMachineTest.periodicNumberOfTimes - 1; i++) {
            builder.step();

            assertSame(builder.getCurrentState().getPeriodic().getClass(), TestPeriodic.class);
        }

        builder.step();

        assertTrue(builder.getCurrentState().getInitialize().getClass() == TestFinish.class);
    }

}