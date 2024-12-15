package frc.robot.subsystems.pneumatics.hardwareWrappers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;

public class Transducer {
  private final AnalogInput input;

  // Analog Inputs (Analog in)

  // Software:
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/analog-inputs-software.html

  // Hardware:
  // https://docs.wpilib.org/en/stable/docs/hardware/sensors/analog-inputs-hardware.html#analog-inputs-hardware

  /**
   * Create traducer
   *
   * @param analogInChannel Analog input channel for the transducer (Analog 0-3 are on-board)
   */
  public Transducer(int analogInChannel) {
    input = new AnalogInput(analogInChannel);

    // Set the analog average bits to 2 to get a more stable reading
    input.setAverageBits(2);
  }

  /**
   * Get pressure reading from analog input voltage
   *
   * @return pressure in psi reading from transducer
   */
  public double getTankPSI() {
    // Per what Spencer said, the voltage will range from between .5 and 4.5 volts, and the pressure
    // will range from 0 to 150 PSI.
    // They are linearly related, so we can use the interpolate function to get the PSI from the
    // voltage.
    // double volts = input.getVoltage();
    double volts = input.getAverageVoltage();
    return MathUtil.interpolate(0, 150, MathUtil.inverseInterpolate(.5, 4.5, volts));
  }
}
