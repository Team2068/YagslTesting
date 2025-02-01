package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.utility.Util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;
  public static boolean isBlueAlliance = false;
  private boolean driverStationReady = false;

  StructArrayPublisher<SwerveModuleState> current_states = Util.table
      .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> posePublisher = Util.table
      .getStructTopic("Current pose", Pose2d.struct).publish();

  /** Creates a new YAGSL. */
  public Swerve() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1);

    swerveDrive.setMotorIdleMode(true);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        maximumSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  public void getTargetAngle() {

  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public Rotation2d getRoll() {
    return swerveDrive.getRoll();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    if (!driverStationReady) {
      driverStationReady = DriverStation.getAlliance().isPresent();
    } else {
      isBlueAlliance = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    }

    if (!driverStationReady) {
      driverStationReady = DriverStation.getAlliance().isPresent();
    } else {
      isBlueAlliance = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    }

    SmartDashboard.putNumber("X position", getPose().getX());
    SmartDashboard.putNumber("Y position", getPose().getY());

    

    current_states.set(swerveDrive.getStates());
    Pose2d pose = getPose();
    posePublisher.set(pose);
  }
}