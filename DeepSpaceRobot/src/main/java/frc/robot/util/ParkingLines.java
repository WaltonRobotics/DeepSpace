package frc.robot.util;

import static org.opencv.imgproc.Imgproc.line;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class ParkingLines {

  private static double focusX;
  private static double focusY;
  private static double percentage = 1;
  private static double xOffset;

  private static Scalar lineColor = new Scalar(0, 255, 0);

  private static void drawLeftLine(Mat input) {
    double l = Math.hypot(focusX, focusY) * percentage;

    double r = focusY / focusX;
    double x = l * Math.cos(Math.atan(r));
    double y = r * x;

    double height = input.height();

    line(input, new Point(xOffset, height), new Point(x, height - y), lineColor, 2);
  }

  private static void drawRightLine(Mat input) {
    double l = Math.hypot(focusX, focusY) * percentage;

    Size size = input.size();

    double r = focusY / (focusX - size.width);
    double x = l * Math.cos(Math.PI + Math.atan(r));
    double y = r * x;

    Point endPoint = new Point(x + size.width, size.height - y);
    line(input, new Point(size.width - xOffset, size.height), endPoint, lineColor, 2);
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
