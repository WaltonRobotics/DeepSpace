package frc.utils;

import java.io.IOException;
import java.io.OutputStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class Logger {

    public static final int LOG_LEVEL_FATAL = 0;
    public static final int LOG_LEVEL_ERROR = 1;
    public static final int LOG_LEVEL_WARNING = 2;
    public static final int LOG_LEVEL_INFO = 3;

    public static final DateFormat LOG_DATE_FORMAT = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");

    private int logLevel;

    private ArrayList<OutputStream> writeables;

    public Logger() {
        logLevel = LOG_LEVEL_INFO;

        writeables = new ArrayList<OutputStream>();
        writeables.add(System.out);
    }

    public Logger(OutputStream os) {
        logLevel = LOG_LEVEL_INFO;

        writeables = new ArrayList<OutputStream>();
        writeables.add(os);
    }

    public Logger(ArrayList<OutputStream> writeables) {
        logLevel = LOG_LEVEL_INFO;

        this.writeables = writeables;
    }

    public void logMessage(String message, int desiredLevel, boolean newline) {
        if (logLevel >= desiredLevel) {
            String outputString = new String();

            if (newline) {
                if (desiredLevel == LOG_LEVEL_FATAL) outputString = "Fatal [";
                else if (desiredLevel == LOG_LEVEL_ERROR) outputString = "Error [";
                else if (desiredLevel == LOG_LEVEL_WARNING) outputString = "Warning [";
                else if (desiredLevel == LOG_LEVEL_INFO) outputString = "Info [";
                else return;

                outputString += LOG_DATE_FORMAT.format(System.currentTimeMillis());

                outputString += "]: ";
            }

            outputString += message;
            outputString += "\n";

            for (OutputStream os : writeables) {
                try {
                    os.write(outputString.getBytes());
                    os.flush();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public void logFatal(String message) {
        logMessage(message, LOG_LEVEL_FATAL, false);
    }

    public void logFatalLine(String message) {
        logMessage(message, LOG_LEVEL_FATAL, true);
    }

    public void logError(String message) {
        logMessage(message, LOG_LEVEL_ERROR, false);
    }

    public void logErrorLine(String message) {
        logMessage(message, LOG_LEVEL_ERROR, true);
    }

    public void logWarning(String message) {
        logMessage(message, LOG_LEVEL_WARNING, false);
    }

    public void logWarningLine(String message) {
        logMessage(message, LOG_LEVEL_WARNING, true);
    }

    public void logInfo(String message) {
        logMessage(message, LOG_LEVEL_INFO, false);
    }

    public void logInfoLine(String message) {
        logMessage(message, LOG_LEVEL_INFO, true);
    }

    public void addOutputStream(OutputStream o) {
        writeables.add(o);
    }

    public void removeOutputStream(OutputStream o) {
        writeables.remove(o);
    }

    public int getLogLevel() {
        return logLevel;
    }

    public void setLogLevel(int logLevel) {
        this.logLevel = logLevel;
    }

    public ArrayList<OutputStream> getWriteables() {
        return writeables;
    }

    public void setWriteables(ArrayList<OutputStream> writeables) {
        this.writeables = writeables;
    } 

}