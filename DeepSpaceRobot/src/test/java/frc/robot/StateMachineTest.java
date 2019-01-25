package test.java.frc.robot;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import frc.states.PeriodicTransition;
import frc.states.StateBuilder;

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

        StateMachine.periodicCounter = 0;
        
        class TestFinish implements State {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Finish test successful!");
                
                return null;
            }

        }

        class TestPeriodic implements State {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Periodic test successful!");

                StateMachine.periodicCounter++;

                if (StateMachine.periodicCounter >= StateMachine.periodicNumberOfTimes) {
                    return new TestFinish();
                }

                return new TestPeriodic();
            }
        }

        class TestInitialize implements State {

            @Override
            public State run() {
                StateMachineTest.testLogger.logInfoLine("Initialization test successful!");
                
                return new TestPeriodic();
            }

        }

        State firstState = new TestInitialize();

        StateBuilder builder = new StateBuilder(firstState);

        assertTrue(builder.getCurrentState() == firstState);

        for (int i = 0; i < StateMachineTest.periodicNumberOfTimes; i++) {
            builder.step();

            assertTrue(builder.getCurrentState().getClass() == TestPeriodic.class);
        }

        assertTrue(builder.getCurrentState.getClass() == TestFinish.class);
    }

}