package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.math.SwerveMath;

public class DriveTrain extends SubsystemBase
{
    public SwerveDrive swerveDrive;

    public DriveTrain()
    {
        // Swerve drive initiation according to YAGSL
        try
        {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                            Units.inchesToMeters(SwerveConstants.wheelDiameter), SwerveConstants.gearRatioDriveMk4);
            SmartDashboard.putNumber("SwerveDriveConversionFactor", driveConversionFactor);
            double steeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveConstants.gearRatioSteerMk4);
            SmartDashboard.putNumber("SwerveSteeringConversionFactor", steeringConversionFactor);

            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.maxModuleSpeedMps,
                            steeringConversionFactor, driveConversionFactor);

            SmartDashboard.putNumber("swerve/baseRadius", swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters());

            setupPathPlanner();

        } catch (IOException exception)
        {
            exception.printStackTrace();
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        // Open loop is disabled since it shouldn't be used most of the time.
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Drive the robot given a chassis field oriented velocity.
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    // Drive according to the chassis robot oriented velocity.
    public void drive(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
                        this::getPose, // Robot pose supplier
                        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
                                             // pose)
                        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        new HolonomicPathFollowerConfig(
                                        PathPlannerConstants.TRANSLATION_PID,
                                        // Translation PID constants
                                        PathPlannerConstants.ANGLE_PID,
                                        // Rotation PID constants
                                        SwerveConstants.maxModuleSpeedMps,
                                        // Max module speed, in m/s
                                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                        // Drive base radius in meters. Distance from robot center to furthest module.
                                        new ReplanningConfig()
                        // Default path replanning config. See the API for the options here
                        ),
                        () ->
                        {
                            // Boolean supplier that controls when the path will be mirrored for the red
                            // alliance
                            // This will flip the path being followed to the red side of the field.
                            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                            var alliance = DriverStation.getAlliance();
                            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                        },
                        this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose
     *            The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds
     *            Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
}
