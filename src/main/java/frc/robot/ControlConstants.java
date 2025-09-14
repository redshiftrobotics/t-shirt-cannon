package frc.robot;

import frc.robot.subsystems.led.LEDPattern;

/** ALL VALUES ARE IN PSI */
public class ControlConstants {

  /**
   * Threshold which reservoir tank (Big one) will try to fill to. Once this pressure is reached the
   * compressor will turn off.
   */
  public static final double reservoirMaxThresholdPressure = 100.0;

  /**
   * Threshold which reservoir tank (Big one) will try to stay above. Once this pressure is below
   * this value the compressor will turn on.
   */
  public static final double reservoirMinThresholdPressure = 85.0;

  /**
   * Starting target pressure of shot tank (Small one for firing). Sets to this pressure on A button
   * press
   */
  public static final double shotTankDefaultPressure = 50.0;

  /**
   * Alternate target pressure of shot tank (Small one for firing). Sets to this pressure on X
   * button press
   */
  public static final double shotTankSecondaryPressure = 70.0;

  /**
   * Amount target pressure of shot tank (Small one for firing) will change either up or down when
   * left and right bumpers are pressed
   */
  public static final double shotTankPressureChange = 5.0;

  /** LED Patterns */
  public static final LEDPattern idlePattern = LEDPattern.PARTY_PALETTE;

  public static final LEDPattern loadedPattern = LEDPattern.BPM_PARTY;
  public static final LEDPattern firingPattern = LEDPattern.STROBE_RED;
}
