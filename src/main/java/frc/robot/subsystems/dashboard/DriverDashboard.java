package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.SpeedController.SpeedLevel;
import frc.robot.subsystems.pneumatics.tank.Tank;
import frc.robot.utility.NormUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriverDashboard extends SubsystemBase {

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private DriverDashboard() {}

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Fields ---

  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;
  private Supplier<SpeedLevel> speedLevelSupplier;
  private BooleanSupplier fieldRelativeSupplier;
  private BooleanSupplier angleDrivenSupplier;

  private BooleanSupplier reservoirTankFilling;
  private Supplier<Double> reservoirTankPressure;

  private BooleanSupplier gatewayTankFilling;
  private Supplier<Double> gatewayTankPressure;

  private BooleanSupplier readyToFireSupplier;
  private DoubleSupplier targetPressure;
  private BooleanSupplier cannonOpen;

  // --- Setters ---

  public void addSubsystem(SubsystemBase subsystem) {
    if (subsystem instanceof Drive) {
      SmartDashboard.putData("Drive Subsystem", subsystem);
    } else {
      throw new IllegalArgumentException("Unknown subsystem can not be added to driver dashboard");
    }
  }

  public void addCommand(String name, Command command, boolean runsWhenDisabled) {
    SmartDashboard.putData(name, command.withName(name).ignoringDisable(runsWhenDisabled));
  }

  public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.poseSupplier = robotPoseSupplier;
  }

  public void setRobotSpeedsSupplier(Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.speedsSupplier = robotSpeedsSupplier;
  }

  public void setSpeedLevelSupplier(Supplier<SpeedLevel> speedLevelSupplier) {
    this.speedLevelSupplier = speedLevelSupplier;
  }

  public void setFieldRelativeSupplier(BooleanSupplier fieldRelativeSupplier) {
    this.fieldRelativeSupplier = fieldRelativeSupplier;
  }

  public void setAngleDrivenSupplier(BooleanSupplier angleDrivenSupplier) {
    this.angleDrivenSupplier = angleDrivenSupplier;
  }

  public void setReservoirTank(
      BooleanSupplier reservoirTankFilling,
      Supplier<Double> reservoirTankPressure,
      Tank reservoirTank) {
    this.reservoirTankFilling = reservoirTankFilling;
    this.reservoirTankPressure = reservoirTankPressure;
    SmartDashboard.putData("Reservoir Tank", reservoirTank);
  }

  public void setGatewayTank(
      BooleanSupplier gatewayTankFilling,
      Supplier<Double> gatewayTankPressure,
      Tank gatewayTank) {
    this.gatewayTankFilling = gatewayTankFilling;
    this.gatewayTankPressure = gatewayTankPressure;
    SmartDashboard.putData("Gateway Tank", gatewayTank);
  }

  public void setCannon(
      BooleanSupplier readyToFireSupplier,
      DoubleSupplier targetPressure,
      BooleanSupplier cannonOpen) {
    this.readyToFireSupplier = readyToFireSupplier;
    this.targetPressure = targetPressure;
    this.cannonOpen = cannonOpen;
  }

  @Override
  public void periodic() {
    // TODO set this up to be more programmatic, and not need a periodic method. Task for someone.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/index.html
    // Example in Commands/SwerveModuleOffsetReader.java

    if (poseSupplier != null) {
      Pose2d pose = poseSupplier.get();
      SmartDashboard.putNumber("Heading Degrees", -pose.getRotation().getDegrees());
    }

    if (speedsSupplier != null) {
      ChassisSpeeds speeds = speedsSupplier.get();

      SmartDashboard.putNumber("Speed MPH", NormUtil.norm(speeds) * 2.2369);
    }

    if (speedLevelSupplier != null) {
      SpeedLevel speedLevel = speedLevelSupplier.get();

      SmartDashboard.putString("Speed Level", speedLevel.name());
      SmartDashboard.putString(
          "Speed Transl", String.format("%.2f%%", speedLevel.getTranslationCoefficient() * 100));
      SmartDashboard.putString(
          "Speed Rot", String.format("%.2f%%", speedLevel.getRotationCoefficient() * 100));
    }

    if (fieldRelativeSupplier != null) {
      SmartDashboard.putBoolean("Field Relative", fieldRelativeSupplier.getAsBoolean());
    }

    if (angleDrivenSupplier != null) {
      SmartDashboard.putBoolean("Angle Driven", angleDrivenSupplier.getAsBoolean());
    }

    if (reservoirTankFilling != null) {
      SmartDashboard.putBoolean("Reservoir Filling", reservoirTankFilling.getAsBoolean());
    }

    if (reservoirTankPressure != null) {
      SmartDashboard.putNumber("Reservoir Pressure", reservoirTankPressure.get());
    }

    if (gatewayTankFilling != null) {
      SmartDashboard.putBoolean("Gateway Filling", gatewayTankFilling.getAsBoolean());
    }

    if (gatewayTankPressure != null) {
      SmartDashboard.putNumber("Gateway Pressure", gatewayTankPressure.get());
    }

    if (readyToFireSupplier != null) {
      SmartDashboard.putBoolean("Fire Accurate", readyToFireSupplier.getAsBoolean());
    }

    if (targetPressure != null) {
      SmartDashboard.putNumber("Target Launch Pressure", targetPressure.getAsDouble());
    }

    if (cannonOpen != null) {
      SmartDashboard.putBoolean("Cannon Open", cannonOpen.getAsBoolean());
    }
  }
}
