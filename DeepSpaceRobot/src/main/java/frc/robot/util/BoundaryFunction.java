package frc.robot.util;


import javafx.scene.paint.Color;

public enum BoundaryFunction {
  ABS_SQRT(Color.BLUE) {
    @Override
    public double apply(double input) {
      return Math.sqrt(Math.abs(input));
    }
  },

  INVERSE(Color.RED) {
    private double a = 1;
    private double b = 1;

    @Override
    public double apply(double input) {
      return 1 / Math.abs(a * Math.pow(input, b));
    }
  };

  private final Color color;

  BoundaryFunction(Color color) {

    this.color = color;
  }

  public Color getColor() {
    return color;
  }

  public abstract double apply(double input);

}
