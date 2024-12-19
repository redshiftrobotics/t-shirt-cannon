package frc.robot.subsystems.pneumatics.gateway;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThresholdController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the gateway tank and the value which fills it,
 * along with the sensors on the gateway tank.
 */
public class GatewayTank extends SubsystemBase {

  private final GatewayIO io;
  private final GatewayIOInputsAutoLogged inputs = new GatewayIOInputsAutoLogged();

  private final ThresholdController controller;

  private final InterpolatingShotTable shotTable;

  private boolean paused;
  private boolean backfill;

  private double targetShotDistance;

  /** Create a new GatewayTank subsystem */
  public GatewayTank(GatewayIO io) {
    this.io = io;

    paused = false;
    backfill = false;

    controller = new ThresholdController();

    shotTable = new InterpolatingShotTable();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GatewayTank", inputs);

    if ((controller.calculate(inputs.tankPSI) > 0 && !paused && DriverStation.isEnabled())
        || backfill) {
      io.beganFilling();
    } else {
      io.stopFilling();
    }
  }

  // --- Getters ---

  /**
   * Get whether compressor is currently active and filling Gateway tank
   *
   * @return compressor state
   */
  public boolean isFilling() {
    return inputs.isFilling;
  }

  /**
   * Get current pressure of gateway tank
   *
   * @return pressure in psi (pound per square inch)
   */
  public double getPressure() {
    return inputs.tankPSI;
  }

  /**
   * Get whether pressure is within the tolerance range
   *
   * @return true if pressure is within tolerance range, false otherwise
   */
  public boolean isPressureWithinTolerance() {
    return MathUtil.isNear(
        getPressure(),
        shotTable.getDesiredPSI(targetShotDistance),
        GatewayConstants.TOLERANCE_PRESSURE);
  }

  /**
   * Get the estimated distance the t-shirt will travel based on the current pressure in the gateway
   *
   * @return distance in meters
   */
  public double getEstimatedLaunchDistance() {
    return shotTable.getEstimatedLaunchDistance(inputs.tankPSI);
  }

  /**
   * Get the target distance the t-shirt will travel
   *
   * @return distance in meters
   */
  public double getTargetLaunchDistance() {
    return targetShotDistance;
  }

  /**
   * Get target pressure of gateway tank
   *
   * @return pressure in psi (pound per square inch)
   */
  public double getTargetPressure() {
    return controller.getUpperThreshold();
  }

  // --- Setters ---

  /**
   * Set the target distance the t-shirt will travel, changes target pressure of the gateway tank
   *
   * @param distance in meters
   */
  public void setTargetLaunchDistance(double distance) {
    setDesiredPSI(
        shotTable.getDesiredPSI(
            MathUtil.clamp(
                distance, GatewayConstants.MIN_SHOT_DISTANCE, GatewayConstants.MAX_SHOT_DISTANCE)));
  }

  /**
   * Set the target pressure of the gateway tank
   *
   * @param psi target pressure in psi (pound per square inch)
   */
  public void setDesiredPSI(double psi) {
    controller.setThresholds(
        MathUtil.clamp(
            psi - GatewayConstants.TOLERANCE_PRESSURE,
            GatewayConstants.MIN_ALLOWED_PRESSURE,
            GatewayConstants.MAX_ALLOWED_PRESSURE),
        MathUtil.clamp(
            psi, GatewayConstants.MIN_ALLOWED_PRESSURE, GatewayConstants.MAX_ALLOWED_PRESSURE));

    targetShotDistance = shotTable.getEstimatedLaunchDistance(psi);
  }

  /** Sets the setpoint pressure to none. The compressor will not activate. */
  public void stopFilling() {
    controller.setThresholds(0, 0);
  }

  // --- Sim Drain ---

  /**
   * Sets a supplier that tells the sim whether it is draining.
   *
   * @param isDrainingSupplier A supplier that returns true if the tank is draining air.
   */
  public void setSimDrain(BooleanSupplier isDrainingSupplier) {
    this.io.setSimDrain(isDrainingSupplier);
  }

  // --- Pause Conditions ---

  /** Pause the compressor */
  public void pause() {
    this.paused = true;
  }

  /** Unpause the compressor */
  public void unpause() {
    this.paused = false;
  }

  public boolean isPaused() {
    return this.paused;
  }

  // --- Backfill Conditions ---

  public void backfill() {
    this.backfill = true;
  }

  public void stopBackfill() {
    this.backfill = false;
  }

  public boolean isBackfilling() {
    return this.backfill;
  }

  // --- Status String ---

  public String getStatusString() {
    if (DriverStation.isDisabled()) {
      return "Pause: Disabled";
    }
    if (isFilling()) {
      if (isBackfilling()) {
        return String.format("Backfilling. Currently at %.2f PSI", getPressure());
      }
      return String.format("Filling to %.2f PSI (End threshold)", controller.getUpperThreshold());
    } else if (!controller.isOn() && getPressure() > controller.getLowerThreshold()) {
      return String.format(
          "Stopped till %.2f PSI (Start threshold)", controller.getLowerThreshold());
    }
    if (isPaused()) {
      return getCurrentCommand().getName();
    }
    return "Idle";
  }

  // --- Testing ---

  public void forceOpen() {
    System.out.println("Opening Gateway Tank");
    io.beganFilling();
  }

  public void forceClose() {
    System.out.println("Closing Gateway Tank");
    io.stopFilling();
  }
}
