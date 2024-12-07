package frc.robot.subsystems.gateway;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThresholdController;
import java.util.ArrayList;
import java.util.Optional;
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

  private final ArrayList<PauseCondition> pauseConditions;

  /** Create a new GatewayTank subsystem */
  public GatewayTank(GatewayIO io) {
    this.io = io;

    pauseConditions = new ArrayList<>();

    controller = new ThresholdController();

    shotTable = new InterpolatingShotTable();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GatewayTank", inputs);

    if (controller.calculate(inputs.tankPSI) > 0
        && !shouldPauseFilling()
        && DriverStation.isEnabled()) {
      io.beganFilling();
    } else {
      io.stopFilling();
    }

    Logger.recordOutput("GatewayTank/status", getStatusString());
  }

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
   * Get the estimated distance the t-shirt will travel based on the current pressure in the gateway
   * 
   * @return distance in meters
   */
  public double getDistance() {
    return shotTable.getEstimatedLaunchDistance(inputs.tankPSI);
  }

  /**
   * Set the target distance the t-shirt will travel, changes target pressure of the gateway tank
   * 
   * @param distance in meters
   */
  public void setDistance(double distance) {
    setDesiredPSI(shotTable.getDesiredPSI(distance));
  }

  /**
   * Set the target pressure of the gateway tank
   *
   * @param psi target pressure in psi (pound per square inch)
   */
  public void setDesiredPSI(double psi) {
    psi = MathUtil.clamp(psi, GatewayConstants.MIN_ALLOWED_PRESSURE, GatewayConstants.MAX_ALLOWED_PRESSURE);
    controller.setThresholds(Math.max(psi - GatewayConstants.PRESSURE_TOLERANCE, GatewayConstants.MIN_ALLOWED_PRESSURE), psi);
  }

  /** Sets the setpoint pressure to none. The compressor will not activate. */
  public void stopFilling() {
    controller.setThresholds(0, 0);
  }

  public void addPauseFillingCondition(BooleanSupplier condition, String reason) {
    pauseConditions.add(new PauseCondition(condition, reason));
  }

  public void addPauseFillingCondition(
      BooleanSupplier condition, String reason, double debounceTimeSeconds) {
    final Debouncer debouncer = new Debouncer(debounceTimeSeconds, DebounceType.kBoth);
    pauseConditions.add(
        new PauseCondition(() -> debouncer.calculate(condition.getAsBoolean()), reason));
  }

  public boolean shouldPauseFilling() {
    return pauseConditions.stream().anyMatch(PauseCondition::isActive);
  }

  public Optional<String> getPauseReason() {
    return pauseConditions.stream()
        .filter(PauseCondition::isActive)
        .map(PauseCondition::reason)
        .findFirst();
  }

  public String getStatusString() {
    if (isFilling()) {
      return String.format(
          "Filling to %.2f PSI for a distance of %.2f meters",
          controller.getUpperThreshold());
    } else if (!controller.isOn()) {
      return String.format(
          "Waiting to fill till pressure below %.2f PSI", controller.getLowerThreshold());
    }
    if (shouldPauseFilling()) {
      return "Paused: " + getPauseReason().orElse("Unknown");
    }
    return "Idle";
  }

  public static record PauseCondition(BooleanSupplier condition, String reason) {
    public boolean isActive() {
      return condition.getAsBoolean();
    }
  }
}
