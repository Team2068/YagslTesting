package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;

public class IO extends SubsystemBase {
        // public final Swerve chassis = new Swerve();
        public final Swerve swerve = new Swerve();
        final XboxController driverController = new XboxController(0);

        public static final double DEADBAND = 0.1;

        public CommandScheduler scheduler = CommandScheduler.getInstance();

        public IO() {
                DriverStation.silenceJoystickConnectionWarning(true);

                AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(swerve,
                                // Applies deadbands and inverts controls because joysticks
                                // are back-right positive while robot
                                // controls are front-left positive
                                () -> MathUtil.applyDeadband(-driverController.getLeftY(),
                                                DEADBAND),
                                () -> MathUtil.applyDeadband(-driverController.getLeftX(),
                                                DEADBAND),
                                () -> -driverController.getRightX(),
                                () -> -driverController.getRightY(),
                                driverController::getRightBumper,
                                driverController::getLeftBumper,
                                () -> driverController.getXButton());

                swerve.setDefaultCommand(closedAbsoluteDrive);

        }

        public void configureBindings() {
                new JoystickButton(driverController, XboxController.Button.kA.value)
                                .onTrue(new InstantCommand(swerve::zeroGyro));
                new JoystickButton(driverController, XboxController.Button.kX.value)
                                .whileTrue(new RepeatCommand(new InstantCommand(swerve::lock)));
        }

        // StructPublisher<Pose2d> estimated_pose = Util.table.getStructTopic("Estimated
        // Pose", Pose2d.struct).publish();

        @Override
        public void periodic() {
        }
}