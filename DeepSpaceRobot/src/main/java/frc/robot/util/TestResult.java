package frc.robot.util;

public class TestResult {

  private final String testName;
  private final boolean success;
  private final AssertionError error;

  public TestResult(String testName, boolean success, AssertionError error) {
    this.testName = testName;
    this.success = success;
    this.error = error;
  }

  public String getTestName() {
    return testName;
  }

  public boolean isSuccess() {
    return success;
  }

  public AssertionError getError() {
    return error;
  }
}
