package frc.robot.util;

import edu.wpi.first.wpilibj.command.CommandGroup;
import java.util.ArrayList;
import java.util.Collection;

public class TestRunner extends CommandGroup {

  private final Collection<TestCommand> testCommands = new ArrayList<>();

  public final synchronized void addSequential(TestCommand testCommand) {
    super.addSequential(testCommand);
    testCommands.add(testCommand);
  }

  public final synchronized void addSequential(TestCommand testCommand, double timeout) {
    super.addSequential(testCommand, timeout);
    testCommands.add(testCommand);
  }

  public final synchronized void addParallel(TestCommand testCommand, double timeout) {
    super.addParallel(testCommand, timeout);
    testCommands.add(testCommand);
  }

  public final synchronized void addParallel(TestCommand testCommand) {
    super.addParallel(testCommand);
    testCommands.add(testCommand);
  }

  @Override
  protected void end() {
    for (TestCommand testCommand : testCommands) {
      System.out.println(testCommand.getTestName());
      if (testCommand.isCompleted()) {
        for (TestResult testResult : testCommand.getTestResult()) {
          if (testResult.isSuccess()) {
            System.out.printf("[TEST][%f][%s][SUCCESS]%n", testResult.getTime(), testResult.getTestName());
          } else {
            System.out.printf("[TEST][%f][%s][FAIL]%n", testResult.getTime(), testResult.getTestName());
            testResult.getError().printStackTrace();
          }
        }
      } else {
        System.out.printf("%s has not finished running%n", testCommand.getTestName());
      }
      System.out.println();
    }
  }

  @Override
  public String toString() {
    return "TestRunner{" +
        "testCommands=" + testCommands +
        "} " + super.toString();
  }
}
