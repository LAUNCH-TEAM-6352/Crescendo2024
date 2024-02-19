// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DashboardConstants.IntakeKeys;
import frc.robot.Constants.IntakeConstants.PIDConstants;

public class Intake extends SubsystemBase
{
    private final CANSparkMax largeRollerMotor = new CANSparkMax(IntakeConstants.largeRollerMotorChannel,
                    MotorType.kBrushless);
    private final CANSparkMax smallRollerMotor = new CANSparkMax(IntakeConstants.smallRollerMotorChannel,
                    MotorType.kBrushless);

    /** Creates a new Intake. */
    public Intake()
    {
        // Apply configuration common to both large and small roller motors:
        for (CANSparkMax motor : new CANSparkMax[]
        { largeRollerMotor, smallRollerMotor })
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
        largeRollerMotor.setInverted(IntakeConstants.isLargeRollerMotorInverted);
        smallRollerMotor.setInverted(IntakeConstants.isSmallRollerMotorInverted);
    }

    public void intake()
    {
        largeRollerMotor.getPIDController().setReference(SmartDashboard.getNumber(IntakeKeys.largeRollerIntakeRpm, IntakeConstants.largeRollerMotorIntakeRpm),
                        CANSparkBase.ControlType.kVelocity);
        smallRollerMotor.getPIDController().setReference(SmartDashboard.getNumber(IntakeKeys.smallRollerIntakeRpm, IntakeConstants.smallRollerMotorIntakeRpm),
                        CANSparkBase.ControlType.kVelocity);
    }

    public void eject()
    {
        largeRollerMotor.getPIDController().setReference(SmartDashboard.getNumber(IntakeKeys.largeRollerEjectRpm, IntakeConstants.largeRollerMotorEjectRpm),
                        CANSparkBase.ControlType.kVelocity);
        smallRollerMotor.getPIDController().setReference(SmartDashboard.getNumber(IntakeKeys.smallRollerEjectRpm, IntakeConstants.smallRollerMotorEjectRpm),
                        CANSparkBase.ControlType.kVelocity);
    }

    public void stop()
    {
        largeRollerMotor.stopMotor();
        smallRollerMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
