package test.java.frc.robot;

import org.junit.Test;

import frc.states.PeriodicTransition;
import frc.states.StateBuilder;

public class StateMachineTest {

    public static int periodicCounter = 0;
    public static int periodicNumberOfTimes = 5;

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
                String dateTime = new SimpleDataFormat("yyyy-MM-dd hh:mm:ss", Locale.getDefault().format(new Date()));

                System.out.println("Finish test successful at: " + dateTime);
                
                return null;
            }

        }

        class TestPeriodic implements State {

            @Override
            public State run() {
                String dateTime = new SimpleDataFormat("yyyy-MM-dd hh:mm:ss", Locale.getDefault().format(new Date()));

                System.out.println("Periodic test successful at: " + dateTime);

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
                String dateTime = new SimpleDataFormat("yyyy-MM-dd hh:mm:ss", Locale.getDefault().format(new Date()));

                System.out.println("Initialization test successful at: " + dateTime);
                
                return new TestPeriodic();
            }

        }
    }

}