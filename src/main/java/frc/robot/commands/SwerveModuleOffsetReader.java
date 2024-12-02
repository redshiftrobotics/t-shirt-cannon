package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Map;

public class SwerveModuleOffsetReader extends Command {
  private static final String TAB_TITLE = "Swerve Module Offsets";

  private final Drive drive;
  private final ShuffleboardTab tab;
  private final GenericEntry speedWidget, angleWidget;

  public SwerveModuleOffsetReader(Drive drive) {
    this.drive = drive;
    this.tab = Shuffleboard.getTab(TAB_TITLE);

    speedWidget =
        tab.add("Speed - Meters Per Second", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(5, 2)
            .withPosition(6, 1)
            .withProperties(
                Map.of(
                    "min",
                    -DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
                    "max",
                    DriveConstants.DRIVE_CONFIG.maxLinearVelocity()))
            .getEntry();

    angleWidget =
        tab.add("Angle - Degrees", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(5, 2)
            .withPosition(6, 3)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

    tab.addNumber(
            "Front Left",
            () ->
                drive
                    .getWheelPositions()
                    .positions[0]
                    .angle
                    .minus(DriveConstants.FRONT_LEFT_MODULE_CONFIG.absoluteEncoderOffset())
                    .getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 1)
        .withSize(2, 2);

    tab.addNumber(
            "Front Right",
            () ->
                drive
                    .getWheelPositions()
                    .positions[1]
                    .angle
                    .minus(DriveConstants.FRONT_RIGHT_MODULE_CONFIG.absoluteEncoderOffset())
                    .getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 1)
        .withSize(2, 2);

    tab.addNumber(
            "Back Left",
            () ->
                drive
                    .getWheelPositions()
                    .positions[2]
                    .angle
                    .minus(DriveConstants.BACK_LEFT_MODULE_CONFIG.absoluteEncoderOffset())
                    .getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 3)
        .withSize(2, 2);

    tab.addNumber(
            "Back Right",
            () ->
                drive
                    .getWheelPositions()
                    .positions[3]
                    .angle
                    .minus(DriveConstants.BACK_RIGHT_MODULE_CONFIG.absoluteEncoderOffset())
                    .getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 3)
        .withSize(2, 2);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.setMotorBrakeOnCoastModeEnabled(false);
    Shuffleboard.selectTab(TAB_TITLE);
  }

  @Override
  public void execute() {

    double driveSpeed = speedWidget.getDouble(0);
    double driveAngle = angleWidget.getDouble(0);

    if (!RobotState.isDisabled()) {
      drive.setWheelSpeeds(
          new SwerveDriveWheelStates(
              new SwerveModuleState[] {
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(driveAngle)),
              }));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setMotorBrakeOnCoastModeEnabled(true);
    drive.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
