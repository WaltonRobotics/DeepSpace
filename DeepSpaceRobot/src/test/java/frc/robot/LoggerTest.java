package frc.robot;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import frc.utils.Logger;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class LoggerTest {

    @Test
    public void loggingInfoTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging info test.";

        logger.logInfo(testLogString);

        assertEquals(os.toString(), testLogString);
    }

    @Test
    public void loggingWarningTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging warning test.";

        logger.logWarning(testLogString);

        assertEquals(os.toString(), testLogString);
    }

    @Test
    public void loggingErrorTest() {
        ByteArrayOutputStream os = new ByteArrayOutputStream();

        ArrayList writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging error test.";

        logger.logError(testLogString);

        assertEquals(os.toString(), testLogString);
    }

}