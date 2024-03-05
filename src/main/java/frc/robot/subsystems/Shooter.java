// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DashboardConstants.ShooterKeys;
import frc.robot.Constants.ShooterConstants.PIDConstants;

public class Shooter extends SubsystemBase
{
    private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.leftMotorChannel,
        MotorType.kBrushless);

    private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.rightMotorChannel,
        MotorType.kBrushless);

    private final CANSparkMax[] motors = new CANSparkMax[] { leftMotor, rightMotor };

    private boolean isSpinningUp = false;
    private boolean isAtTargetVelocity = false;

    private double targetVelocity;

    private double lastLeftVelocity;
    private double lastRightVelocity;

    private double velocityTolerance;

    /** Creates a new Shooter. */
    public Shooter()
    {
        // Apply configuration common to both large and small roller motors:
        for (CANSparkMax motor : motors)
        {
            motor.restoreFactoryDefaults();
            motor.clearFaults();
            motor.setIdleMode(ShooterConstants.motorIdleMode);

            var pidController = motor.getPIDController();
            pidController.setP(PIDConstants.kP);
            pidController.setI(PIDConstants.kI);
            pidController.setD(PIDConstants.kD);
            pidController.setIZone(PIDConstants.kIZ);
            pidController.setFF(PIDConstants.kFF);
            pidController.setOutputRange(PIDConstants.minOutput, PIDConstants.maxOutput);
        }

        leftMotor.setInverted(ShooterConstants.isLeftMotorInverted);
        rightMotor.setInverted(ShooterConstants.isRightMotorInverted);
    }

    public void setAmpSpeed()
    {
        setVelocity(SmartDashboard.getNumber(ShooterKeys.ampRpm, ShooterConstants.ampRpm));
    }

    public void setSpeakerSpeed()
    {
        setVelocity(SmartDashboard.getNumber(ShooterKeys.speakerRpm, ShooterConstants.speakerRpm));
    }

    /**
     * Sets both motors to run at the specified velocity (in RPM).
     */
    private void setVelocity(double velocity)
    {
        targetVelocity = velocity;
        velocityTolerance = SmartDashboard.getNumber(ShooterKeys.rpmTolerance, ShooterConstants.rpmTolerance);

        lastLeftVelocity = 0;
        lastRightVelocity = 0;

        for (CANSparkMax motor : motors)
        {
            motor.getPIDController().setReference(targetVelocity, CANSparkBase.ControlType.kVelocity);
        }

        isSpinningUp = true;
        isAtTargetVelocity = false;
    }

    public void stop()
    {
        for (CANSparkMax motor : motors)
        {
            motor.stopMotor();
        }
    }

    public boolean isAtTargetVelocity()
    {
        return isAtTargetVelocity;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        double leftVelocity = leftMotor.getEncoder().getVelocity();
        double rightVelocity = rightMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Shooter/Left/RPM", leftVelocity);
        SmartDashboard.putNumber("Shooter/Right/RPM", rightVelocity);

        // Determine if both shooter motors have come up to speed and stabalized:
        if (isSpinningUp)
        {
            if (Math.abs(leftVelocity - targetVelocity) < velocityTolerance &&
                Math.abs(leftVelocity - lastLeftVelocity) < velocityTolerance &&
                Math.abs(rightVelocity - targetVelocity) < velocityTolerance &&
                Math.abs(rightVelocity - lastRightVelocity) < velocityTolerance)
            {
                isSpinningUp = false;
                isAtTargetVelocity = true;
            }
            else
            {
                lastLeftVelocity = leftVelocity;
                lastRightVelocity = rightVelocity;
            }
        }
    }
}
