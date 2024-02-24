package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;

/**
 * IntakeNote intakes a note from the floor
 * move manipulator into intake position
 * start indexer running for intake
 * tell Intake subsystem to intake
 */
public class IntakeNote extends Command
{
    private final Intake intake;
    private final Indexer indexer;
    private final Manipulator manipulator;

    /**
     * 
     * 
     * @param intake
     */
    public IntakeNote(Intake intake, Indexer indexer, Manipulator manipulator)
    {
        this.intake = intake;
        this.indexer = indexer;
        this.manipulator = manipulator;
        // Specify required subsystems
        addRequirements(intake, indexer, manipulator);
    }
    
    @Override
    public void initialize()
    {
        manipulator.moveToIntakePosition();
        indexer.intake();
        intake.intake();
    }

    @Override
    public void execute()
    {
        // Intentionally blank, PID Controllers are controlling motors
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted)
    {
        intake.stop();
       indexer.stop();
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
