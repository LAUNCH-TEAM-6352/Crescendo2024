// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.EjectNote;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNoteIntoAmp;
import frc.robot.commands.ShootNoteIntoSpeaker;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.DashboardConstants.IndexerKeys;
import frc.robot.Constants.DashboardConstants.IntakeKeys;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shooter;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Pneumatics:
    private final Optional<Compressor> compressor;

    // Subsystems:
    private final Optional<DriveTrain> driveTrain;
    private final Optional<Intake> intake;
    private final Optional<Indexer> indexer;
    private final Optional<Manipulator> manipulator;
    private final Optional<Shooter> shooter;

    // OI devices:
    private final XboxController driverGamepad;
    private final XboxController codriverGamepad;
    private final Joystick driverJoystick;

    SendableChooser<Boolean> driveOrientationChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Get the game data message fom the driver station.
        // This message is primarily used during development to
        // construct only certain subsystems.
        // If the message is blank (or all whitespace),
        // all subsystems are constructed.
        // Otherwise, OI devices and subsystems are constructed
        // depending upon the substrings found in the message:
        // -dt- Drive train
        // -oi- Look for OI devices
        // -i- Intake
        // -s- Shooter
        // -idx- Indexer
        // -m- Manipulator
        // -p- Pneumatics
        // -cam- Camera

        var gameData = DriverStation.getGameSpecificMessage().toLowerCase();
        SmartDashboard.putString("Game Data", gameData);

        // Start a camera server for a simple USB camera:
        if (gameData.contains("-cam-") || gameData.isBlank())
        {
            var camera = CameraServer.startAutomaticCapture();
            camera.setFPS(CameraConstants.fps);
            camera.setResolution(CameraConstants.width, CameraConstants.height);
        }

        // Create OI devices:
        if (gameData.contains("-oi-"))
        {
            // Explicitly look for OI devices:
            driverGamepad = DriverStation.isJoystickConnected(OperatorConstants.driverGamepadPort)
                ? new XboxController(OperatorConstants.driverGamepadPort)
                : null;
            driverJoystick = DriverStation.isJoystickConnected(OperatorConstants.driverJoystickPort)
                ? new Joystick(OperatorConstants.driverJoystickPort)
                : null;
            codriverGamepad = DriverStation.isJoystickConnected(OperatorConstants.codriverGamepadPort)
                ? new XboxController(OperatorConstants.codriverGamepadPort)
                : null;
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
            codriverGamepad = new XboxController(OperatorConstants.codriverGamepadPort);
            driverGamepad = null;
            driverJoystick = null;
        }
        // Create pneumatics compressor:
        compressor = gameData.isBlank() || gameData.contains("-p-")
            ? Optional.of(new Compressor(PneumaticsConstants.moduleId, PneumaticsConstants.moduleType))
            : Optional.empty();

        // Create subsystems:
        driveTrain = gameData.isBlank() || gameData.contains("-dt-")
            ? Optional.of(new DriveTrain())
            : Optional.empty();

        intake = gameData.isBlank() || gameData.contains("-i-")
            ? Optional.of(new Intake())
            : Optional.empty();

        indexer = gameData.isBlank() || gameData.contains("-idx-")
            ? Optional.of(new Indexer())
            : Optional.empty();
        manipulator = gameData.isBlank() || gameData.contains("-m-")
            ? Optional.of(new Manipulator())
            : Optional.empty();
        shooter = gameData.isBlank() || gameData.contains("-s-")
            ? Optional.of(new Shooter())
            : Optional.empty();
        // Configure default commands
        configureDefaultCommands();

        // Configure the trigger bindings
        configureBindings();

        // Configure smart dashboard
        configureSmartDashboard();

    }

    /**
     * Configures the default commands.
     */
    private void configureDefaultCommands()
    {
        driveTrain.ifPresent((dt) ->
        {
            if (driverJoystick != null)
            {
                dt.setDefaultCommand(new DriveWithJoystick(dt, driverJoystick, driveOrientationChooser));
            }
            else if (driverGamepad != null)
            {
                dt.setDefaultCommand(new DriveWithGamepad(dt, driverGamepad, driveOrientationChooser));
            }

        });
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        if (intake.isPresent() && indexer.isPresent() && manipulator.isPresent())
        {
            configureBindings(intake.get(), indexer.get(), manipulator.get());
        }
        if (indexer.isPresent() && shooter.isPresent() && manipulator.isPresent())
        {
            configureBindings(indexer.get(), shooter.get(), manipulator.get());
        }
        manipulator.ifPresent(this::configureBindings);

    }

    private void configureBindings(Manipulator manipulator)
    {
        if (codriverGamepad == null)
        {
            return;
        }
        new JoystickButton(codriverGamepad, Button.kLeftBumper.value)
            .onTrue(new InstantCommand(() -> manipulator.moveToClimbingPosition()));
        new JoystickButton(codriverGamepad, Button.kBack.value)
            .onTrue(new InstantCommand(() -> manipulator.moveToIntakePosition()));
        new JoystickButton(codriverGamepad, Button.kStart.value)
            .onTrue(new InstantCommand(() -> manipulator.climb()));
    }

    // Configures bindings for intaking and ejecting notes.
    private void configureBindings(Intake intake, Indexer indexer, Manipulator manipulator)
    {
        if (codriverGamepad == null)
        {
            return;
        }
        new JoystickButton(codriverGamepad, Button.kA.value).whileTrue(new IntakeNote(intake, indexer, manipulator));
        new JoystickButton(codriverGamepad, Button.kB.value).whileTrue(new EjectNote(intake, indexer, manipulator));
    }

    // Configures bindings for shooting notes into speaker and amp.
    private void configureBindings(Indexer indexer, Shooter shooter, Manipulator manipulator)
    {
        if (codriverGamepad == null)
        {
            return;
        }
        new JoystickButton(codriverGamepad, Button.kY.value)
            .onTrue(new ShootNoteIntoSpeaker(indexer, shooter, manipulator));
        new JoystickButton(codriverGamepad, Button.kX.value)
            .onTrue(new ShootNoteIntoAmp(indexer, shooter, manipulator));
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);
        intake.ifPresent(this::configureSmartDashboard);
        indexer.ifPresent(this::configureSmartDashboard);

        // Configure chooser widgets:
        configureDriveOrientationChooser(driveOrientationChooser);
        configureAutoChooser(autoChooser);
    }

    private void configureSmartDashboard(DriveTrain driveTrain)
    {

    }

    private void configureSmartDashboard(Intake intake)
    {
        SmartDashboard.putNumber(IntakeKeys.largeRollerIntakeRpm, IntakeConstants.largeRollerMotorIntakeRpm);
        SmartDashboard.putNumber(IntakeKeys.largeRollerEjectRpm, IntakeConstants.largeRollerMotorEjectRpm);
        SmartDashboard.putNumber(IntakeKeys.smallRollerIntakeRpm, IntakeConstants.smallRollerMotorIntakeRpm);
        SmartDashboard.putNumber(IntakeKeys.smallRollerEjectRpm, IntakeConstants.smallRollerMotorEjectRpm);
    }

    private void configureSmartDashboard(Indexer indexer)
    {
        SmartDashboard.putNumber(IndexerKeys.lowerRollerIntakeRpm, IndexerConstants.lowerRollerMotorIntakeRpm);
        SmartDashboard.putNumber(IndexerKeys.lowerRollerEjectRpm, IndexerConstants.lowerRollerMotorEjectRpm);
        SmartDashboard.putNumber(IndexerKeys.lowerRollerFeedRpm, IndexerConstants.lowerRollerMotorFeedRpm);
        SmartDashboard.putNumber(IndexerKeys.upperRollerFeedRpm, IndexerConstants.upperRollerMotorFeedRpm);
    }

    private void configureDriveOrientationChooser(SendableChooser<Boolean> driveOrientationChooser)
    {
        driveOrientationChooser.setDefaultOption("Field Relative", Boolean.TRUE);
        driveOrientationChooser.addOption("Robot Relative", Boolean.FALSE);
        SmartDashboard.putData("Drive Orientation", driveOrientationChooser);
    }

    private void configureAutoChooser(SendableChooser<Command> autoChooser)
    {
        // autoChooser.setDefaultOption("Leave", new PathPlannerAuto("Leave"));
        // autoChooser.addOption("Return", new PathPlannerAuto("Return"));
        // SmartDashboard.putData("Auto Selection", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
        // An example command will be run in autonomous

    }
}
