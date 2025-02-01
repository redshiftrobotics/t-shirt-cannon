package frc.robot.subsystems.pneumatics.cannon;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class FiringTube extends SubsystemBase {
  private final CannonIO io;
  private final CannonIOInputsAutoLogged inputs = new CannonIOInputsAutoLogged();

  private final String name;

  private final Timer fireTimer;
  private boolean waitingToFire;

  private boolean isLoaded = false;

  private BooleanSupplier fireRequirement = () -> true;

  /** Creates a new FiringTube subsystem */
  public FiringTube(CannonIO io, String name) {
    this.io = io;
    this.name = name;
    fireTimer = new Timer();
    io.close();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FiringTube" + name, inputs);

    if (waitingToFire && fireRequirement.getAsBoolean()) {
      io.open();
      fireTimer.restart();
      waitingToFire = false;
      isLoaded = false;
    }

    if (fireTimer.hasElapsed(CannonConstants.FIRE_TUBE_OPEN_TIME_SECONDS)) {
      io.close();
      fireTimer.stop();
    }
  }

  /**
   * Returns whether the firing tube is open.
   *
   * @return true if the firing tube is open, false otherwise
   */
  public boolean isOpen() {
    return inputs.isOpen;
  }

  public void loadShirt() {
    isLoaded = true;
  }

  /**
   * For LEDs, meaningless
   *
   * @return whether it should be loaded
   */
  public boolean isShirtLoaded() {
    return isLoaded;
  }

  /**
   * Returns whether the firing tube is waiting to open
   *
   * @return true if the firing tube is waiting to open, false otherwise
   */
  public boolean isWaitingToFire() {
    return waitingToFire;
  }

  /** Queues opening the firing tube and starts the timer to close it. */
  public void fire() {
    waitingToFire = true;
  }

  /**
   * Adds a requirement for the firing tube to open. Supplier must return true for the firing tube
   * to open.
   *
   * @param fireRequirement the requirement for the firing tube to open
   */
  public void setFireRequirements(BooleanSupplier fireRequirement) {
    this.fireRequirement = fireRequirement;
  }

  /**
   * Adds a requirement for the firing tube to open. Supplier must return true for the firing tube
   * to open.
   *
   * @param fireRequirement the requirement for the firing tube to open
   * @param debounceTimeSeconds the time in seconds the fire requirement must be true before the
   *     firing tube opens
   */
  public void setFireRequirements(BooleanSupplier fireRequirement, double debounceTimeSeconds) {
    final Debouncer debouncer = new Debouncer(debounceTimeSeconds);
    this.fireRequirement = () -> debouncer.calculate(fireRequirement.getAsBoolean());
  }
}
