package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;
import java.util.List;

public abstract class TestCommand extends CommandBase {

  private final List<TestResult> testResult = new ArrayList<>();

  public String getTestName() {
    return getClass().getSimpleName();
  }

  @Override
  public void initialize() {
    try {
      initializeTest();
      addSuccess("initialize");
    } catch (AssertionError error) {
      addFail("initialize", error);
    }
  }

  public void addSuccess(String testFunction) {
    testResult.add(
        new TestResult(
            String.format(
                "%s %s",
                getTestName(),
                testFunction
            ),
            true,
            Timer.getFPGATimestamp(),
            null
        ));
  }

  public void addFail(String testFunction, AssertionError assertionError) {
    testResult.add(
        new TestResult(
            String.format(
                "%s %s",
                getTestName(),
                testFunction
            ),
            false,
            Timer.getFPGATimestamp(), assertionError
        ));
  }

  @Override
  public void execute() {
    try {
      executeTest();
      addSuccess("execute");
    } catch (AssertionError error) {
      addFail("execute", error);
    }
  }

  @Override
  public void end(boolean interrupted) {
    try {
      endTest();
      addSuccess("end");
    } catch (AssertionError error) {
      addFail("end", error);
    }
  }

  protected abstract void initializeTest();

  protected abstract void executeTest();

  protected abstract void endTest();

  public List<TestResult> getTestResult() {
    return testResult;
  }

  @Override
  public String toString() {
    return "TestCommand{" +
        "testResult=" + testResult +
        "} " + super.toString();
  }
}