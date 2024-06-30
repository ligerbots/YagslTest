// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveTrain extends SubsystemBase {

    private static final double MAX_SPEED = Units.feetToMeters(14.5);
    
    private static final double JOYSTICK_DEADBAND = 0.05;

    private static final double STEER_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0);

    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    // This is the L2 gearing
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    // if true, then robot is in field centric mode
    private boolean m_fieldCentric = true;

    // if true, then robot is in precision mode
    private boolean m_precisionMode = false;
    // need to remember the configured max rotation speed
    private final double m_maxRotationSpeed;
    private static final double PRECISION_MODE_SCALE_FACTOR = 1.0 / 6.0;
    private static final double OUTREACH_MODE_SCALE_FACTOR = 0.5;

    // Swerve drive object
    private final SwerveDrive m_swerveDrive;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public DriveTrain() {

        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(STEER_GEAR_RATIO);

        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        // In this case the wheel diameter is 4 inches, which must be converted to
        // meters to get meters/second.
        // The gear ratio is 6.75 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(WHEEL_DIAMETER, DRIVE_GEAR_RATIO);

        // System.out.println("\"conversionFactor\": {");
        // System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        // System.out.println("\t\"drive\": " + driveConversionFactor);
        // System.out.println("}");

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File jsonDir = new File(Filesystem.getDeployDirectory(), "swerve");
            m_swerveDrive = new SwerveParser(jsonDir).createSwerveDrive(MAX_SPEED, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // remember configured max rotation speed
        m_maxRotationSpeed = m_swerveDrive.getMaximumAngularVelocity();

        // Heading correction should only be used while controlling the robot via angle.
        m_swerveDrive.setHeadingCorrection(false);
        
        m_swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                                // for simulations since it causes discrepancies not seen in real life.
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     * Inputs are deadbanded and squared.
     *
     * @param translationX     Translation in the X direction. 
     * @param translationY     Translation in the Y direction. 
     * @param angularRotationX Angular velocity of the robot to set. 
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotation, BooleanSupplier robotCentric) {
        return run(() -> {
            // field centric: flip direction if we are Red
            // robot centric (for 2024): input is on the BACK of the robot, so flip to match the camera
            double flipDirection = (robotCentric.getAsBoolean() || isRedAlliance()) ? -1.0 : 1.0;

            m_swerveDrive.drive(
                new Translation2d(
                    flipDirection * modifyAxis(translationX.getAsDouble()) * m_swerveDrive.getMaximumVelocity(),
                    flipDirection * modifyAxis(translationY.getAsDouble()) * m_swerveDrive.getMaximumVelocity()),
                    modifyAxis(angularRotation.getAsDouble()) * m_swerveDrive.getMaximumAngularVelocity(),
                    !robotCentric.getAsBoolean(),
                    false);
        });
    }

    private double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis
        return Math.copySign(value * value, value);
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        m_swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void driveRobotOriented(ChassisSpeeds velocity) {
        m_swerveDrive.drive(velocity);
    }

        // for the beginning of auto rountines
    public void resetDrivingModes() {
        setFieldCentricMode(true);
        setPrecisionMode(false);
    }

    // toggle whether driving is field-centric
    public void toggleFieldCentric() {
        setFieldCentricMode(!m_fieldCentric);
    }

    // toggle precision mode for driving
    public void togglePrecisionMode() {
        setPrecisionMode(!m_precisionMode);
    }

    public void setFieldCentricMode(boolean fieldCentricMode) {
        m_fieldCentric = fieldCentricMode;
    }

    public void setPrecisionMode(boolean precisionMode) {
        m_precisionMode = precisionMode;
        double maxSpeed = MAX_SPEED;
        double maxAngSpeed = m_maxRotationSpeed;
        
        if (precisionMode) {
            maxSpeed = maxSpeed * PRECISION_MODE_SCALE_FACTOR;
            maxAngSpeed = maxAngSpeed * PRECISION_MODE_SCALE_FACTOR;
        } else if (Constants.OUTREACH_MODE) {
            maxSpeed = maxSpeed * OUTREACH_MODE_SCALE_FACTOR;
            maxAngSpeed = maxAngSpeed * OUTREACH_MODE_SCALE_FACTOR;
        }
        m_swerveDrive.setMaximumSpeeds(maxSpeed, maxSpeed, maxAngSpeed);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    // public SwerveDriveKinematics getKinematics() {
    //     return swerveDrive.kinematics;
    // }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this method. 
     * However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to keep working.
     *
     * @param pose The pose to set the odometry to
     */
    public void setPose(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Display a trajectory on the field.
     *
     * @param trajectory The trajectory to display.
     */
    public void postTrajectory(Trajectory trajectory) {
        m_swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroHeading() {
        m_swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    // public void zeroGyroWithAlliance() {
    //     if (isRedAlliance()) {
    //         zeroGyro();
    //         // Set the pose 180 degrees
    //         resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    //     } else {
    //         zeroGyro();
    //     }
    // }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setBrakeMode(boolean brake) {
        m_swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                MAX_SPEED);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return m_swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return m_swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        m_swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return m_swerveDrive.getPitch();
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive, 12),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive),
                3.0, 5.0, 3.0);
    }
}
