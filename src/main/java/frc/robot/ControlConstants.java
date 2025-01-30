package frc.robot;

/** ALL VALUES ARE IN PSI */
public class ControlConstants {

  /**
   * Threshold which reservoir tank (Big one) will try to fill to. Once this pressure is reached the
   * compressor will turn off.
   */
  public static final double reservoirMaxThresholdPressure = 70.0;

  /**
   * Threshold which reservoir tank (Big one) will try to stay above. Once this pressure is below
   * this value the compressor will turn on.
   */
  public static final double reservoirMinThresholdPressure = 60.0;

  /**
   * Starting target pressure of shot tank (Small one for firing). Sets to this pressure on A button
   * press
   */
  public static final double shotTankDefaultPressure = 30.0;

  /**
   * Alternate target pressure of shot tank (Small one for firing). Sets to this pressure on X
   * button press
   */
  public static final double shotTankSecondaryPressure = 50.0;

  /**
   * Amount target pressure of shot tank (Small one for firing) will change either up or down when
   * left and right bumpers are pressed
   */
  public static final double shotTankPressureChange = 10.0;

  /** Tolerance which shot tank should try to be within of target pressure. Will be */
  public static final double shotTankThresholdTolerance = 5.0;
}
