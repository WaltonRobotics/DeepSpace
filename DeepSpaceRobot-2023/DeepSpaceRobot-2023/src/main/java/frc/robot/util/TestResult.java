package frc.robot.util;

public class TestResult {

  private final String testName;
  private final boolean success;
  private final double time;
  private final AssertionError error;

  public TestResult(String testName, boolean success, double time, AssertionError error) {
    this.testName = testName;
    this.success = success;
    this.time = time;
    this.error = error;
  }

  public double getTime() {
    return time;
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

  @Override
  public String toString() {
    return "TestResult{" +
        "testName='" + testName + '\'' +
        ", success=" + success +
        ", time=" + time +
        ", error=" + error +
        '}';
  }
}
