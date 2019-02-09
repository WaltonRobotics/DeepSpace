package frc.robot.util;

import edu.wpi.first.wpilibj.command.Command;
import java.util.ArrayList;
import java.util.List;

public abstract class TestCommand extends Command {

  private List<TestResult> testResult = new ArrayList<>();

  public abstract String getTestName();

  @Override
  protected void initialize() {
    try {
      initializeTest();
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "initialize"), true, null));
    } catch (AssertionError error) {
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "initialize"), false, error));
    }
  }

  @Override
  protected void execute() {
    try {
      executeTest();
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "execute"), true, null));
    } catch (AssertionError error) {
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "execute"), false, error));
    }
  }

  @Override
  protected void end() {
    try {
      endTest();
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "end"), true, null));
    } catch (AssertionError error) {
      testResult.add(new TestResult(String.format("%s %s", getTestName(), "end"), false, error));
    }
  }

  protected abstract void initializeTest();

  protected abstract void executeTest();

  protected abstract void endTest();

  public List<TestResult> getTestResult() {
    return testResult;
  }
}