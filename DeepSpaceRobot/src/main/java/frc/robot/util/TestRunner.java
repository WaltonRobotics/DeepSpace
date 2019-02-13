package frc.robot.util;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.IllegalUseOfCommandException;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.ArrayList;
import java.util.List;

public class TestRunner extends CommandGroup {

  private List<TestCommand> testCommands = new ArrayList<>();

  public final synchronized void addSequential(TestCommand testCommand) {
    super.addSequential(testCommand);
    testCommands.add(testCommand);
  }

  public final synchronized void addSequential(TestCommand testCommand, double timeout) {
    super.addSequential(testCommand, timeout);
    testCommands.add(testCommand);
  }

  public final synchronized void addParrallel(TestCommand testCommand, double timeout) {
    super.addParallel(testCommand, timeout);
    testCommands.add(testCommand);
  }


  public final synchronized void addParrallel(TestCommand testCommand) {
    super.addParallel(testCommand);
    testCommands.add(testCommand);
  }

  @Override
  public synchronized void start() {
    super.start();
  }



  @Override
  protected void end() {
    super.end();
    System.out.println("hello i am ending");

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
}
