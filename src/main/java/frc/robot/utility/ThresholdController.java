package frc.robot.utility;

public class ThresholdController {
  private double lowerThreshold;
  private double upperThreshold;

  private boolean isOn;

  /** Creates a new ThresholdController with default thresholds of 0 */
  public ThresholdController() {
    this(0, 0);
  }

  /**
   * Creates a new ThresholdController with specified thresholds.
   *
   * @param lowerThreshold The lower threshold at which the controller turns ON.
   * @param upperThreshold The upper threshold at which the controller turns OFF.
   */
  public ThresholdController(double lowerThreshold, double upperThreshold) {
    setThresholds(lowerThreshold, upperThreshold);
    isOn = false;
  }

  /**
   * Sets both the lower and upper thresholds.
   *
   * @param lowerThreshold The lower threshold at which the controller turns ON.
   * @param upperThreshold The upper threshold at which the controller turns OFF.
   */
  public void setThresholds(double lowerThreshold, double upperThreshold) {
    if (lowerThreshold > upperThreshold) {
      throw new IllegalArgumentException("Lower threshold must be less than the upper threshold.");
    }
    this.lowerThreshold = lowerThreshold;
    this.upperThreshold = upperThreshold;
  }

  /**
   * Sets the lower threshold.
   *
   * @param lowerThreshold The lower threshold at which the controller turns ON.
   */
  public void setLowerThreshold(double lowerThreshold) {
    if (lowerThreshold > upperThreshold) {
      throw new IllegalArgumentException("Lower threshold must be less than the upper threshold.");
    }
    this.lowerThreshold = lowerThreshold;
  }

  /**
   * Sets the upper threshold.
   *
   * @param upperThreshold The upper threshold at which the controller turns OFF.
   */
  public void setUpperThreshold(double upperThreshold) {
    if (upperThreshold < lowerThreshold) {
      throw new IllegalArgumentException(
          "Upper threshold must be greater than than lower threshold.");
    }
    this.upperThreshold = upperThreshold;
  }

  /**
   * Returns the lower threshold.
   *
   * @return The lower threshold.
   */
  public double getLowerThreshold() {
    return lowerThreshold;
  }

  /**
   * Returns the upper threshold.
   *
   * @return The upper threshold.
   */
  public double getUpperThreshold() {
    return upperThreshold;
  }

  /**
   * Returns whether the controller is currently ON.
   *
   * @return True if the controller is ON; otherwise, false.
   */
  public boolean isOn() {
    return isOn;
  }

  /**
   * Returns the control output based on the thresholds.
   *
   * @param measurement The current measurement of the process variable.
   * @return 1 if the measurement is below the lower threshold or remains ON until the upper
   *     threshold is reached; otherwise, 0.
   */
  public boolean calculate(double measurement) {
    if (measurement > upperThreshold) {
      isOn = false;
    } else if (measurement < lowerThreshold) {
      isOn = true;
    }
    return isOn;
  }
}
