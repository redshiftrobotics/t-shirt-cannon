package frc.robot.utility;

import java.util.function.BooleanSupplier;

/** Sim to try and equalize pressure between two tanks, by connecting them with a "Pipe" */
@SuppressWarnings("unused")
public class SimPneumaticsPipe {
  // TODO: Implement this class and use it in the simulation

  private final double flow_rate;
  private final BooleanSupplier isOpen;

  public SimPneumaticsPipe(double flow_rate, BooleanSupplier isOpen) {
    this.flow_rate = flow_rate;
    this.isOpen = isOpen;
  }
}
