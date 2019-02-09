package frc.robot.util;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.TestRunner.TestResult;

public abstract class TestCommand extends Command {

  private TestResult testResult = null;

  @Override
  protected void initialize() {
    initializeTest();
  }

  @Override
  protected void execute() {
    executeTest();
  }

  @Override
  protected void end() {
    testResult = endTest();
  }

  protected abstract void initializeTest();

  protected abstract void executeTest();

  protected abstract TestResult endTest();

  public TestResult getTestResult() {
    return testResult;
  }
}