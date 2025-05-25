package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final double defaultPattern;

  private Spark pwm;
  private double pattern;

  public LEDSubsystem(int pwmPort, double defaultPattern) {
    pwm = new Spark(pwmPort);
    this.pattern = this.defaultPattern = defaultPattern;
  }

  @Override
  public void periodic() {
    pwm.set(pattern);
  }

  private void clearPattern() {
    setPattern(defaultPattern);
  }

  private void setPattern(double pattern) {
    SmartDashboard.putNumber("LED Pattern", pattern);
    this.pattern = pattern;
  }

  public Command solid(double pattern) {
    return startEnd(() -> setPattern(pattern), this::clearPattern);
  }

  public Command flash(double pattern1, double pattern2, double cycleTimeSeconds) {
    return solid(pattern1)
        .withTimeout(cycleTimeSeconds)
        .andThen(solid(pattern2).withTimeout(cycleTimeSeconds))
        .repeatedly();
  }
}
