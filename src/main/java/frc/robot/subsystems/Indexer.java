// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.DashboardConstants.IndexerKeys;
import frc.robot.Constants.IndexerConstants.PIDConstants;

public class Indexer extends SubsystemBase
{
    private final CANSparkMax lowerRollerMotor = new CANSparkMax(IndexerConstants.lowerRollerMotorChannel,
                    MotorType.kBrushless);
    private final CANSparkMax upperRollerMotor = new CANSparkMax(IndexerConstants.upperRollerMotorChannel,
                    MotorType.kBrushless);

    /** Creates a new Indexer. */
    public Indexer()
    {
        // Apply configuration common to both large and small roller motors:
        for (CANSparkMax motor : new CANSparkMax[]
        { lowerRollerMotor, upperRollerMotor })
        {
            motor.restoreFactoryDefaults();
            motor.clearFaults();
            var pidController = motor.getPIDController();
            pidController.setP(PIDConstants.kP);
            pidController.setI(PIDConstants.kI);
            pidController.setD(PIDConstants.kD);
            pidController.setIZone(PIDConstants.kIZ);
            pidController.setFF(PIDConstants.kFF);
        }
        // Apply configuration unique to large and small roller motors:
        lowerRollerMotor.setInverted(IndexerConstants.isLowerRollerMotorInverted);
        upperRollerMotor.setInverted(IndexerConstants.isUpperRollerMotorInverted);
    }

    public void intake()
    {
        lowerRollerMotor.getPIDController()
                        .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerIntakeRpm,
                                        IndexerConstants.lowerRollerMotorIntakeRpm),
                                        CANSparkBase.ControlType.kVelocity);
    }

    public void eject()
    {
        lowerRollerMotor.getPIDController()
                        .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerEjectRpm,
                                        IndexerConstants.lowerRollerMotorEjectRpm),
                                        CANSparkBase.ControlType.kVelocity);
    }

    public void feed()
    {
        lowerRollerMotor.getPIDController()
                        .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerFeedRpm,
                                        IndexerConstants.lowerRollerMotorFeedRpm),
                                        CANSparkBase.ControlType.kVelocity);
        upperRollerMotor.getPIDController()
                        .setReference(SmartDashboard.getNumber(IndexerKeys.upperRollerFeedRpm,
                                        IndexerConstants.upperRollerMotorFeedRpm),
                                        CANSparkBase.ControlType.kVelocity);
    }

    public void stop()
    {
        lowerRollerMotor.stopMotor();
        upperRollerMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
