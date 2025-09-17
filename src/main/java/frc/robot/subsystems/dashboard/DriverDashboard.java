package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.NormUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriverDashboard extends SubsystemBase {

  private final double MPS_TO_MPH = 2.23694;

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private DriverDashboard() {}

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Fields with safe defaults ---

  public Supplier<ChassisSpeeds> speedsSupplier = () -> new ChassisSpeeds(0, 0, 0);

  public Supplier<Pose2d> poseSupplier = Pose2d::new;
  public BooleanSupplier fieldRelativeSupplier = () -> false;
  public BooleanSupplier angleDrivenSupplier = () -> false;

  public BooleanSupplier reservoirTankFilling = () -> false;
  public DoubleSupplier reservoirTankPressure = () -> 0.0;
  public Supplier<String> reservoirTankStatus = () -> "Unknown";

  public BooleanSupplier gatewayTankFilling = () -> false;
  public DoubleSupplier gatewayTankPressure = () -> 0.0;
  public Supplier<String> gatewayTankStatus = () -> "Unknown";

  public BooleanSupplier readyToFireSupplier = () -> false;
  public DoubleSupplier targetPressure = () -> 0.0;
  public BooleanSupplier backfillMode = () -> false;
  public BooleanSupplier cannonOpen = () -> false;

  // --- Setters ---

  public void addSubsystem(SubsystemBase subsystem) {
    if (subsystem instanceof Drive) {
      SmartDashboard.putData("DriveSubsystem", subsystem);
    } else {
      throw new IllegalArgumentException("Unknown subsystem cannot be added to driver dashboard");
    }
  }

  public void addCommand(String name, Command command, boolean runsWhenDisabled) {
    SmartDashboard.putData(name, command.withName(name).ignoringDisable(runsWhenDisabled));
  }

  public void addChooser(LoggedDashboardChooser<?> chooser) {
    SmartDashboard.putData(chooser.getSendableChooser());
  }

  // --- Periodic updates to dashboard ---

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();
    SmartDashboard.putNumber(
        "Heading Degrees",
        pose.getRotation().getDegrees() == 0
            ? 0
            : MathUtil.inputModulus(-pose.getRotation().getDegrees(), 0, 360));

    ChassisSpeeds speeds = speedsSupplier.get();
    SmartDashboard.putNumber("Speed MPH", NormUtil.norm(speeds) * MPS_TO_MPH);

    SmartDashboard.putBoolean("Field Relative", fieldRelativeSupplier.getAsBoolean());
    SmartDashboard.putBoolean("Angle Driven", angleDrivenSupplier.getAsBoolean());

    SmartDashboard.putBoolean("Reservoir Filling", reservoirTankFilling.getAsBoolean());
    SmartDashboard.putNumber("Reservoir Pressure", reservoirTankPressure.getAsDouble());
    SmartDashboard.putString("Reservoir Status", reservoirTankStatus.get());

    SmartDashboard.putBoolean("Gateway Filling", gatewayTankFilling.getAsBoolean());
    SmartDashboard.putNumber("Gateway Pressure", gatewayTankPressure.getAsDouble());
    SmartDashboard.putString("Gateway Status", gatewayTankStatus.get());

    SmartDashboard.putBoolean("Fire Accurate", readyToFireSupplier.getAsBoolean());
    SmartDashboard.putNumber("Target Launch Pressure", targetPressure.getAsDouble());
    SmartDashboard.putBoolean("Backfill Mode", backfillMode.getAsBoolean());
    SmartDashboard.putBoolean("Cannon Open", cannonOpen.getAsBoolean());
  }
}
