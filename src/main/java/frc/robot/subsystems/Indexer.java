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
import frc.robot.Constants.IndexerConstants.UpperPIDConstants;
import frc.robot.Constants.IndexerConstants.LowerPIDConstants;


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
        }
        
        var pidController = lowerRollerMotor.getPIDController();
        pidController.setP(LowerPIDConstants.kP);
        pidController.setI(LowerPIDConstants.kI);
        pidController.setD(LowerPIDConstants.kD);
        pidController.setIZone(LowerPIDConstants.kIZ);
        pidController.setFF(LowerPIDConstants.kFF);

        pidController = upperRollerMotor.getPIDController();
        pidController.setP(UpperPIDConstants.kP);
        pidController.setI(UpperPIDConstants.kI);
        pidController.setD(UpperPIDConstants.kD);
        pidController.setIZone(UpperPIDConstants.kIZ);
        pidController.setFF(UpperPIDConstants.kFF);

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
