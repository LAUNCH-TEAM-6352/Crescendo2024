// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
public static final class IndexerConstants
{
    //indexer consists of 4 shafts. 2 coupled pairs lower pair, upper pair, one motor per pair, lower pair 51 upper pair 52. NEO 550 and CAN SparkMAX
}
    public static final class IntakeConstants
    {
        public static final int largeRollerMotorChannel = 41;
        public static final int smallRollerMotorChannel = 42;
        public static final double largeRollerMotorIntakeRpm = 2600;
        public static final double smallRollerMotorIntakeRpm = 2100;
        public static final double largeRollerMotorEjectRpm = -2600;
        public static final double smallRollerMotorEjectRpm = -2100;
        public static final boolean areRollerMotorsInverted = false;

        public static final class PIDConstants
        {
            public static final double kP = 6e-5;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final int kIZ = 0;
            public static final double kFF = 0.00005;
            public static final double defaultMinOutput = -1;
            public static final double defaultMaxOutput = 1;
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
    }

    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class CameraConstants
    {
        public static final int fps = 10;
        public static final int width = 320;
        public static final int height = 240;
    }
}
