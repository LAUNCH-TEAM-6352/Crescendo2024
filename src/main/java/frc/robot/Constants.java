// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final class OperatorConstants
    {
        public static final int driverJoystickPort = 0;
        public static final int driverGamepadPort = 1;
        public static final int codriverGamepadPort = 2;
        public static final double joystickDeadband = 0.1;
        public static final double gamepadDeadband = 0.05;
    }

    public static final class CameraConstants
    {
        public static final int fps = 10;
        public static final int width = 320;
        public static final int height = 240;
    }

    public static final class IndexerConstants
    {
        public static final int lowerRollerMotorChannel = 43;
        public static final int upperRollerMotorChannel = 44;

        // TODO: Tune RPM values
        public static final double lowerRollerMotorIntakeRpm = 2100;
        public static final double upperRollerMotorIntakeRpm = 2100;

        public static final double lowerRollerMotorEjectRpm = -2100;
        public static final double upperRollerMotorEjectRpm = -2100;

        public static final double lowerRollerMotorFeedRpm = 2100;
        public static final double upperRollerMotorFeedRpm = 2100;
        
        public static final boolean isLowerRollerMotorInverted = false;
        public static final boolean isUpperRollerMotorInverted = true;

        public static final IdleMode motorIdleMode = IdleMode.kBrake;

        public static final class LowerPIDConstants
        {
            public static final double kP = 0.000024;
            public static final double kI = 3.0e-7;
            public static final double kD = 0.0;
            public static final int kIZ = 0;
            public static final double kFF = 0.0;
            public static final double minOutput = -1;
            public static final double maxOutput = 1;
        }

        public static final class UpperPIDConstants
        {
            public static final double kP = 0.00001;
            public static final double kI = 0.0000035;
            public static final double kD = 3.0;
            public static final int kIZ = 0;
            public static final double kFF = 0.00009;
            public static final double minOutput = -1;
            public static final double maxOutput = 1;
        }
    }

    public static final class IntakeConstants
    {
        public static final int largeRollerMotorChannel = 41;
        public static final int smallRollerMotorChannel = 42;

        public static final int opticalSensorPort = 0;

         // TODO: Tune motor RPM values
        public static final double largeRollerMotorIntakeRpm = 2600;
        public static final double smallRollerMotorIntakeRpm = 2100;

        public static final double largeRollerMotorEjectRpm = -2600;
        public static final double smallRollerMotorEjectRpm = -2100;

        public static final boolean isLargeRollerMotorInverted = true;
        public static final boolean isSmallRollerMotorInverted = true;

        public static final IdleMode motorIdleMode = IdleMode.kBrake;

        public static final class PIDConstants
        {
            public static final double kP = 0.000024;
            public static final double kI = 3.0e-7;
            public static final double kD = 0.0;
            public static final int kIZ = 0;
            public static final double kFF = 0.00005;
            public static final double minOutput = -1;
            public static final double maxOutput = 1;
        }
    }

    public static final class ManipulatorConstants
    {
        public static final int climbingSolenoidForwardChannel = 2;
        public static final int climbingSolenoidReverseChannel = 3;
        public static final int noteSolenoidForwardChannel = 0;
        public static final int noteSolenoidReverseChannel = 1;
        public static final int climberLockSolenoidForwardChannel = 4;
        public static final int climberLockSolenoidReverseChannel = 5;
    }

    public static final class ShooterConstants
    {
        public static final int leftMotorChannel = 45;
        public static final int rightMotorChannel = 46;

        // TODO: Tune motor RPM values
        public static final double ampRpm = 1000.0;
        public static final double speakerRpm = 10000.0;

        // Tolerance for determining if motors are at the desired velocity:
        public static final double rpmTolerance = 10;

        public static final boolean isLeftMotorInverted = true;
        public static final boolean isRightMotorInverted = false;

        public static final IdleMode motorIdleMode = IdleMode.kCoast;

        public static final class PIDConstants
        {
            public static final double kP = 0.000024;
            public static final double kI = 3.0e-7;
            public static final double kD = 0.0;
            public static final int kIZ = 0;
            public static final double kFF = 0.0;
            public static final double minOutput = -1;
            public static final double maxOutput = 1;
        }
    }

    public static final class PneumaticsConstants
    {
        public static final int moduleId = 0;
        public static final PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;
    }

    public static final class SwerveConstants
    {
        public static final double wheelDiameter = 4;
        public static final double maximumLinearVelocityMps = 5.0;
        public static final double maximumRotationRateRps = Math.PI;

        // Don't mess with this!
        public static final double maxModuleSpeedMps = 4.5;

        public static final double gearRatioDriveMk4 = 8.14;
        public static final double gearRatioDriveMk4i = 8.14;

        public static final double gearRatioSteerMk4 = 12.8;
        public static final double gearRatioSteerMk4i = 21.43;
    }

    public static class DriveConstants
    {
        public static final boolean isFieldRelative = true;
        public static final TelemetryVerbosity swerveDriveTelemetryVerbosity = TelemetryVerbosity.HIGH;   
    }

    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class DashboardConstants
    {
        public static final class IndexerKeys
        {
            public static final String lowerRollerIntakeRpm = "Idx Lw In RPM";
            public static final String upperRollerIntakeRpm = "Idx Up In RPM";

            public static final String lowerRollerEjectRpm = "Idx Lw Ej RPM";
            public static final String upperRollerEjectRpm = "Idx Up Ej RPM";

            public static final String lowerRollerFeedRpm = "Idx Lw Fd RPM";
            public static final String upperRollerFeedRpm = "Idx Up Fd RPM";
        }

        public static final class IntakeKeys
        {
            public static final String largeRollerIntakeRpm = "Itk Lg In RPM";
            public static final String smallRollerIntakeRpm = "Itk Sm In RPM";

            public static final String largeRollerEjectRpm = "Itk Lg Ej RPM";
            public static final String smallRollerEjectRpm = "Itk Sm Ej RPM";

            public static final String hasNote = "Has Note";
        }

        public static final class ShooterKeys
        {
            public static final String ampRpm = "Shoot Amp RPM";
            public static final String speakerRpm = "Shoot Spk RPM";

            public static final String rpmTolerance = "Shoot RPM Tol";
        }
    }
}
