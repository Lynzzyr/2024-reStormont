package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.subsystems.Drivetrain;

/**
 * @author Brandon Namgoong
 */
public class Jingle extends Command {

    private final Drivetrain m_drivetrain;

    public Jingle() {
        m_drivetrain = TunerConstantsComp.DriveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.playJingle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopJingle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_drivetrain.getOrchestraState();
    }
}