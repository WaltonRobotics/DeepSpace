package frc.robot;

import frc.robot.util.Logger;
import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import org.junit.Assert;
import org.junit.Test;

public class LoggerTest {

  @Test
  public void loggingInfoTest() {
    ByteArrayOutputStream os = new ByteArrayOutputStream();

    ArrayList<OutputStream> writeables = new ArrayList<>();

    writeables.add(os);

    Logger logger = new Logger(writeables);

    String testLogString = "Logging info test.";

    logger.logInfo(testLogString);

    Assert.assertTrue(os.toString().contains(testLogString));
  }

  @Test
  public void loggingWarningTest() {
    ByteArrayOutputStream os = new ByteArrayOutputStream();

    ArrayList<OutputStream> writeables = new ArrayList<>();

    writeables.add(os);

    Logger logger = new Logger(writeables);

    String testLogString = "Logging warning test.";

    logger.logWarning(testLogString);

    Assert.assertTrue(os.toString().contains(testLogString));
  }

  @Test
  public void loggingErrorTest() {
    ByteArrayOutputStream os = new ByteArrayOutputStream();

    ArrayList<OutputStream> writeables = new ArrayList<>();

    writeables.add(os);

    Logger logger = new Logger(writeables);

    String testLogString = "Logging error test.";

    logger.logError(testLogString);

    Assert.assertTrue(os.toString().contains(testLogString));
  }

}