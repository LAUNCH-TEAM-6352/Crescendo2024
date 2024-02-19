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

    private boolean isSpinningUp = false;
    private boolean isAtTargetVelocity = false;
    private double lastVelocity;
    private double targetVelocity;

    /** Creates a new Shooter. */
    public Shooter()
    {
        // Apply configuration common to both large and small roller motors:
        for (CANSparkMax motor : new CANSparkMax[]
        { leftMotor, rightMotor })
        {
            motor.restoreFactoryDefaults();
            motor.clearFaults();
        }
        leftMotor.setInverted(ShooterConstants.isLeftMotorInverted);

        var pidController = leftMotor.getPIDController();
        pidController.setP(PIDConstants.kP);
        pidController.setI(PIDConstants.kI);
        pidController.setD(PIDConstants.kD);
        pidController.setIZone(PIDConstants.kIZ);
        pidController.setFF(PIDConstants.kFF);

        rightMotor.follow(leftMotor, ShooterConstants.isLeftMotorInverted != ShooterConstants.isRightMotorInverted);
    }

    public void setAmpSpeed()
    {

        targetVelocity = SmartDashboard.getNumber(ShooterKeys.ampRpm, ShooterConstants.ampRpm);
        lastVelocity = 0;
        leftMotor.getPIDController()
                        .setReference(targetVelocity, CANSparkBase.ControlType.kVelocity);
        isSpinningUp = true;
        isAtTargetVelocity = false;
    }

    public void setSpeakerSpeed()
    {
        targetVelocity = SmartDashboard.getNumber(ShooterKeys.speakerRpm, ShooterConstants.speakerRpm);
        lastVelocity = 0;
        leftMotor.getPIDController()
                        .setReference(targetVelocity, CANSparkBase.ControlType.kVelocity);
        isSpinningUp = true;
        isAtTargetVelocity = false;
    }

    public void stop()
    {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
    
    public boolean isAtTargetVelocity()
    {
        return isAtTargetVelocity;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run

        if (isSpinningUp)
        {
            double leftVelocity = leftMotor.getEncoder().getVelocity();
            if (Math.abs(leftVelocity - targetVelocity) < 10 && Math.abs(leftVelocity - lastVelocity) < 10)
            {
                isSpinningUp = false;
                isAtTargetVelocity = true;
            }
            else
            {
                lastVelocity = leftVelocity;
            }
        }
    }
}
