package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.cannon.CannonConstants;
import frc.robot.subsystems.cannon.CannonIO;
import frc.robot.subsystems.cannon.CannonIOHardware;
import frc.robot.subsystems.cannon.CannonIOSim;
import frc.robot.subsystems.cannon.FiringTube;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.SpeedController;
import frc.robot.subsystems.drive.controllers.SpeedController.SpeedLevel;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import frc.robot.subsystems.gateway.GatewayIO;
import frc.robot.subsystems.gateway.GatewayIOHardware;
import frc.robot.subsystems.gateway.GatewayIOSim;
import frc.robot.subsystems.gateway.GatewayTank;
import frc.robot.subsystems.reservoir.ReservoirIO;
import frc.robot.subsystems.reservoir.ReservoirIOHardware;
import frc.robot.subsystems.reservoir.ReservoirIOSim;
import frc.robot.subsystems.reservoir.ReservoirTank;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.utility.NormUtil;
import frc.robot.utility.OverrideSwitch;
import frc.robot.utility.logging.Alert;
import frc.robot.utility.logging.Alert.AlertType;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final AprilTagVision vision;

  private final ReservoirTank reservoirTank;
  private final GatewayTank gatewayTank;
  private final FiringTube firingTube;

  // Controller
  private final CommandGenericHID driverController = new CommandXboxController(0);
  private final CommandGenericHID operatorController = new CommandXboxController(1);

  // Driver Information
  private final DriverDashboard dashboard = DriverDashboard.getInstance();

  // Auto
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.getRobot()) {
      case CANNON_BOT:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new AprilTagVision();
        reservoirTank = new ReservoirTank(new ReservoirIOHardware());
        gatewayTank = new GatewayTank(new GatewayIOHardware());
        firingTube =
            new FiringTube(
                new CannonIOHardware(CannonConstants.MIDDLE_FIRING_TUBE_SOLENOID_CHANNEL), "Main");
        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        reservoirTank = new ReservoirTank(new ReservoirIOSim());
        gatewayTank = new GatewayTank(new GatewayIOSim());
        firingTube = new FiringTube(new CannonIOSim(), "Main");
        break;

      case TEST_BOT:
        // Test robot, disable IO implementations for most
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new AprilTagVision();
        reservoirTank = new ReservoirTank(new ReservoirIO() {});
        gatewayTank = new GatewayTank(new GatewayIOHardware());
        firingTube = new FiringTube(new CannonIO() {}, "Main");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new AprilTagVision();
        reservoirTank = new ReservoirTank(new ReservoirIO() {});
        gatewayTank = new GatewayTank(new GatewayIO() {});
        firingTube = new FiringTube(new CannonIO() {}, "Main");
        break;
    }

    // Set up vision
    vision.setRobotPoseSupplier(drive::getPose);
    vision.addVisionEstimateConsumer(
        (visionEstimate) -> {
          if (visionEstimate.isSuccess() && Constants.getRobot() != RobotType.SIM_BOT) {
            drive.addVisionMeasurement(
                visionEstimate.robotPose2d(),
                visionEstimate.timestampSeconds(),
                visionEstimate.standardDeviations());
          }
        });

    // Set up reservoir tank
    reservoirTank.setPressureThresholds(20, 30);
    new Trigger(
            () ->
                NormUtil.norm(drive.getRobotSpeeds()) > drive.getMaxLinearSpeedMetersPerSec() * 0.75
                    || Math.abs(drive.getRobotSpeeds().omegaRadiansPerSecond)
                        > drive.getMaxAngularSpeedRadPerSec() * 0.75)
        .debounce(0.3, DebounceType.kBoth)
        .whileTrue(
            reservoirTank
                .startEnd(reservoirTank::pause, reservoirTank::unpause)
                .withName("Pause: Drive Speed"));

    reservoirTank.setSimDrain(gatewayTank::isFilling);

    // Set up gateway tank
    gatewayTank.setDesiredPSI(20);
    new Trigger(firingTube::isOpen)
        .or(firingTube::isWaitingToFire)
        .debounce(0.5, DebounceType.kBoth)
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::pause, gatewayTank::unpause)
                .withName("Pause: Firing Tube Open"));
    gatewayTank.setSimDrain(firingTube::isOpen);

    // Set up firing tube
    firingTube.setFireRequirements(() -> !gatewayTank.isFilling());

    // Alerts for constants to avoid using them in competition
    if (Constants.TUNING_MODE) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

    // Hide controller missing warnings for sim
    if (Constants.getMode() != Mode.REAL) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Put dashboard defaults
    initDashboard();

    // Configure autos
    configureAutos();

    if (Constants.SHOW_SYS_ID_AUTOS) {
      configureSysIds();
    }

    // Configure the button bindings
    configureControllerBindings();

    // Print out robot and mode
    System.out.println(
        String.format("Robot %s, Mode: %s", Constants.getRobot(), Constants.getMode()));
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    dashboard.addSubsystem(drive);
    dashboard.setPoseSupplier(drive::getPose);
    dashboard.setRobotSpeedsSupplier(drive::getRobotSpeeds);
    dashboard.setSpeedLevelSupplier(() -> SpeedController.SpeedLevel.NO_LEVEL);
    dashboard.setFieldRelativeSupplier(() -> false);
    dashboard.setAngleDrivenSupplier(() -> false);

    dashboard.setReservoirTank(
        reservoirTank::isFilling, reservoirTank::getPressure, reservoirTank::getStatusString);

    dashboard.setGatewayTank(
        gatewayTank::isFilling, gatewayTank::getPressure, gatewayTank::getStatusString);

    dashboard.setCannon(
        gatewayTank::isPressureWithinTolerance,
        gatewayTank::getTargetLaunchDistance,
        gatewayTank::getEstimatedLaunchDistance);

    dashboard.addCommand("Reset Pose", drive.runOnce(() -> drive.resetPose(new Pose2d())), true);
    dashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
        true);
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings();
    configureOperatorControllerBindings();
  }

  private void configureDriverControllerBindings() {
    if (driverController instanceof CommandXboxController) {
      final CommandXboxController driverXbox = (CommandXboxController) driverController;

      final Trigger useFieldRelative =
          new Trigger(new OverrideSwitch(driverXbox.y(), OverrideSwitch.Mode.TOGGLE, true));

      final Trigger useAngleControlMode =
          new Trigger(
              new OverrideSwitch(driverXbox.rightBumper(), OverrideSwitch.Mode.HOLD, false));

      // Controllers

      final TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> -driverXbox.getRightY(),
              () -> -driverXbox.getRightX());

      final SpeedController speedController = new SpeedController(SpeedLevel.DEFAULT);

      final HeadingController headingController = new HeadingController(drive);

      DriverDashboard.getInstance().setSpeedLevelSupplier(speedController::getCurrentSpeedLevel);
      DriverDashboard.getInstance().setAngleDrivenSupplier(useAngleControlMode);
      DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);

      // Default command
      drive.setDefaultCommand(
          drive
              .runEnd(
                  () -> {
                    Translation2d translation = input.getTranslationMetersPerSecond();
                    double rotation = input.getOmegaRadiansPerSecond();
                    drive.setRobotSpeeds(
                        speedController.updateSpeed(
                            new ChassisSpeeds(translation.getX(), translation.getY(), rotation)),
                        useFieldRelative.getAsBoolean());
                  },
                  drive::stop)
              .withName("DefaultDrive"));

      useAngleControlMode
          .debounce(0.1)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        headingController.reset();
                        headingController.setGoal(drive.getPose().getRotation());
                      })
                  .withName("PrepareAngleDrive"))
          .whileTrue(
              drive
                  .runEnd(
                      () -> {
                        Translation2d translation = input.getTranslationMetersPerSecond();
                        Optional<Rotation2d> rotation = input.getHeadingDirection();
                        rotation.ifPresent(headingController::setGoal);
                        double omegaRadiansPerSecond = headingController.calculate();
                        drive.setRobotSpeeds(
                            speedController.updateSpeed(
                                new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    headingController.atGoal() ? 0 : omegaRadiansPerSecond)),
                            useFieldRelative.getAsBoolean());
                      },
                      drive::stop)
                  .withName("RotationAngleDrive"));

      boolean includeDiagonalPOV = true;
      for (int pov = 0; pov < 360; pov += includeDiagonalPOV ? 45 : 90) {

        // POV angles are in Clock Wise degrees, needs to be flipped to get correct rotation2d
        final Rotation2d angle = Rotation2d.fromDegrees(-pov);
        final String name = String.format("%d\u00B0", pov);

        // While the POV is being pressed and we are not in angle control mode, set the
        // chassis speeds to the Cos and Sin of the angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode)
            .whileTrue(
                drive
                    .runEnd(
                        () ->
                            drive.setRobotSpeeds(
                                speedController.updateSpeed(
                                    new ChassisSpeeds(angle.getCos(), angle.getSin(), 0))),
                        drive::stop)
                    .withName(String.format("DriveRobotRelative %s", name)));

        // While the POV is being pressed and we are angle control mode
        // Start by resetting the controller and setting the goal angle to the pov angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onTrue(
                drive
                    .runOnce(
                        () -> {
                          headingController.reset();
                          headingController.setGoal(angle);
                        })
                    .withName(String.format("PrepareLockedHeading %s", name)));

        // Then if the button is held for more than 0.2 seconds, drive forward at the
        // angle once the chassis reaches it
        driverXbox
            .pov(pov)
            .debounce(0.2)
            .and(useAngleControlMode.negate())
            .whileTrue(
                drive
                    .run(
                        () -> {
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      (headingController.atGoal() ? 1 : 0),
                                      0,
                                      headingController.atGoal() ? 0 : rotationRadians)));
                        })
                    .withName(String.format("ForwardLockedHeading %s", name)));

        // Then once the pov is let go, if we are not at the angle continue turn to it,
        // while also accepting x and y input to drive. Cancel once we get turn request
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onFalse(
                drive
                    .run(
                        () -> {
                          Translation2d translation = input.getTranslationMetersPerSecond();
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      translation.getX(),
                                      translation.getY(),
                                      headingController.atGoal() ? 0 : rotationRadians)),
                              useFieldRelative.getAsBoolean());
                        })
                    .until(() -> input.getOmegaRadiansPerSecond() != 0)
                    .withName(String.format("DriveLockedHeading %s", name)));
      }

      // While X is held down go into stop and go into the cross position to resistent movement,
      // then once X button is let go put modules forward
      driverXbox
          .x()
          .whileTrue(
              drive
                  .startEnd(drive::stopUsingBrakeArrangement, drive::stopUsingForwardArrangement)
                  .withName("StopWithX"));

      // When be is pressed stop the drivetrain then idle it, cancelling all incoming commands.
      // Also do this when robot is disabled
      driverXbox
          .b()
          .or(RobotModeTriggers.disabled())
          .whileTrue(
              drive
                  .runOnce(drive::stop)
                  .andThen(Commands.idle(drive))
                  .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                  .withName("StopCancel"));

      // When right (Gas) trigger is held down or left stick (sprint) is pressed, put in boost
      // (fast) mode
      driverXbox
          .rightTrigger(0.5)
          // .or(driverXbox.leftStick())
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.BOOST),
                  () -> speedController.removeSpeedLevel(SpeedLevel.BOOST)));

      // When left (Brake) trigger is held down or right stick (crouch) is pressed put in precise
      // (slow) mode
      driverXbox
          .leftTrigger(0.5)
          // .or(driverXbox.rightStick())
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.PRECISE),
                  () -> speedController.removeSpeedLevel(SpeedLevel.PRECISE)));

    } else if (driverController instanceof CommandJoystick) {
      final CommandJoystick driverJoystick = (CommandJoystick) driverController;

      TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverJoystick.getY(),
              () -> -driverJoystick.getX(),
              () -> -driverJoystick.getTwist(),
              () -> 0);

      drive.setDefaultCommand(
          drive.startEnd(
              () -> {
                Translation2d translation = input.getTranslationMetersPerSecond();
                double rotation = input.getOmegaRadiansPerSecond();
                drive.setRobotSpeeds(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation), false);
              },
              drive::stop));
    }
  }

  private void configureOperatorControllerBindings() {
    if (operatorController instanceof CommandXboxController) {
      final CommandXboxController operatorXbox = (CommandXboxController) operatorController;

      // TODO, this should be the other way around, pressing buttons should put command that pause
      // the filling and in general more commands should be used to greatly simplify code.

      operatorXbox
          .y()
          .whileTrue(
              reservoirTank
                  .startEnd(reservoirTank::pause, reservoirTank::unpause)
                  .withName("Pause: Operator Y Button"));

      operatorXbox
          .leftTrigger()
          .whileTrue(
              gatewayTank
                  .startEnd(gatewayTank::pause, gatewayTank::unpause)
                  .withName("Pause: Operator Prepare Fire"));

      operatorXbox.rightTrigger().onTrue(firingTube.runOnce(firingTube::fire).withName("Fire"));

      gatewayTank.setDefaultCommand(
          gatewayTank
              .run(
                  () -> {
                    gatewayTank.setTargetLaunchDistance(
                        gatewayTank.getTargetLaunchDistance()
                            - MathUtil.applyDeadband(operatorXbox.getLeftY(), 0.1) * 0.4);
                  })
              .withName("Adjust Target Distance"));
    }
  }

  public void configureSysIds() {
    // Set up SysId routines
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public void configureAutos() {
    // Set up named commands for path planner auto
    // https://pathplanner.dev/pplib-named-commands.html
    NamedCommands.registerCommand("StopWithX", drive.runOnce(drive::stopUsingBrakeArrangement));

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
    autoChooser.addOption("Triangle Auto", new PathPlannerAuto("Triangle Auto"));
    autoChooser.addOption("Rotate Auto", new PathPlannerAuto("Rotate Auto"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
