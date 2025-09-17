package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  private final Timer startupTimer = new Timer();

  private LEDStrip[] strips;

  public LEDSubsystem(int... pwmPorts) {

    // Create all the strip objects
    strips = new LEDStrip[pwmPorts.length];
    for (int i = 0; i < pwmPorts.length; i++) {
      strips[i] = new LEDStrip(new PWM(pwmPorts[i]), LEDConstants.OFF);
    }
  }

  @Override
  public void periodic() {

    if (DriverStation.isEnabled() && !startupTimer.isRunning()) {
      startupTimer.restart();
    }

    boolean hasSetup = startupTimer.hasElapsed(0.8);
    Logger.recordOutput("LED/hasSetup", hasSetup);

    if (hasSetup) {
      Stream.of(strips).forEach(LEDStrip::update);
    } else {
      Stream.of(strips).forEach(LEDStrip::setUp);
    }

    Logger.recordOutput(
        "LED/measuredPulses", Stream.of(strips).map(LEDStrip::getPulse).toList().toString());
    Logger.recordOutput(
        "LED/targetPulse",
        Stream.of(strips)
            .map(LEDStrip::getTargetPattern)
            .map(BlinkenLEDPattern::toString)
            .toList()
            .toString());
  }

  public Command applyColor(BlinkenLEDPattern color) {
    return run(() -> set(color));
  }

  public Command applyColor(Supplier<BlinkenLEDPattern> color) {
    return run(() -> set(color.get()));
  }

  public Command turnOff() {
    return applyColor(LEDConstants.OFF);
  }

  public void set(BlinkenLEDPattern pattern) {
    Stream.of(strips).forEach(strip -> strip.setPattern(pattern));
  }

  private class LEDStrip {
    // Todo: Make this IO layer
    private final PWM pwm;
    private BlinkenLEDPattern pattern;

    public LEDStrip(PWM pwmController, BlinkenLEDPattern initialPattern) {
      pwm = pwmController;
      pattern = initialPattern;
    }

    public void setPattern(BlinkenLEDPattern pattern) {
      this.pattern = pattern;
    }

    public void setUp() {
      pwm.setPulseTimeMicroseconds(2125);
    }

    public BlinkenLEDPattern getTargetPattern() {
      return pattern;
    }

    public int getPulse() {
      return pwm.getPulseTimeMicroseconds();
    }

    public void update() {
      pwm.setPulseTimeMicroseconds(pattern.getPulse());
    }
  }
}
