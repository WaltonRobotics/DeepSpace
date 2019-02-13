package frc.robot;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;

import frc.robot.util.Logger;

import org.junit.Test;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

public class LoggerTest {

    @Test
    public void loggingInfoTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList<OutputStream> writeables = new ArrayList<>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging info test.";

        logger.logInfo(testLogString);

        assertTrue(os.toString().contains(testLogString));
    }

    @Test
    public void loggingWarningTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList<OutputStream> writeables = new ArrayList<>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging warning test.";

        logger.logWarning(testLogString);

        assertTrue(os.toString().contains(testLogString));
    }

    @Test
    public void loggingErrorTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList<OutputStream> writeables = new ArrayList<>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging error test.";

        logger.logError(testLogString);

        assertTrue(os.toString().contains(testLogString));
    }

}