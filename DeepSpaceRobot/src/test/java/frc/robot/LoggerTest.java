package frc.robot;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import frc.utils.Logger;

import org.junit.Test;

public class LoggerTest {

    @Test
    public void loggingInfoTest() {
        ByteArrayOutputStream os;

        List writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging info test.";

        logger.logInfo(testLogString);

        assertEquals(os.toString(), testLogString);
    }

    @Test
    public void loggingWarningTest() {
        ByteArrayOutputStream os;

        List writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging warning test.";

        logger.logWarning(testLogString);

        assertEquals(os.toString(), testLogString);
    }

    @Test
    public void loggingErrorTest() {
        ByteArrayOutputStream os;

        List writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging error test.";

        logger.logError(testLogString);

        assertEquals(os.toString(), testLogString);
    }

}