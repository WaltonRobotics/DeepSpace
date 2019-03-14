package frc.robot.util;

import static org.opencv.imgproc.Imgproc.line;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class ParkingLines {

  private static double focusX;
  private static double focusY;
  private static double percentage = 1.0;
  private static double xOffset;

  private static Scalar lineColor = new Scalar(0, 255.0, 0);

  private ParkingLines() {
  }

  private static void drawLeftLine(Mat input) {

    double x = xOffset + (percentage * (focusX - xOffset));
    double y = percentage * focusY;
    double height = input.height();

    line(input, new Point(xOffset, height), new Point(x, height - y), lineColor, 2);
  }

  private static void drawRightLine(Mat input) {
    double width = input.width();
    double height = input.height();
    double xOffset = width - ParkingLines.xOffset;

    double x = xOffset + (percentage * (focusX - xOffset));
    double y = percentage * focusY;

    line(input, new Point(width - ParkingLines.xOffset, height), new Point(x, height - y), lineColor, 2);
  }

  public static void drawParkingLines(Mat input) {
    drawLeftLine(input);
    drawRightLine(input);
  }

  public static void setFocusPoint(double focusX, double focusY) {
    ParkingLines.focusX = focusX;
    ParkingLines.focusY = focusY;
  }

  public static void setPercentage(double percentage) {
    ParkingLines.percentage = percentage;
  }

  public static void setXOffset(double xOffset) {
    ParkingLines.xOffset = xOffset;
  }

  public static void setLineColor(Scalar lineColor) {
    ParkingLines.lineColor = lineColor;
  }
}
