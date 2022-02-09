import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand {
    private Joystick m_stick = null;
    private IntakeSubsystem Intake = null ;
    public IntakeCommand(Joystick joystick, IntakeSubsystem intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_stick = joystick;
        Intake = intake;
        addRequirements(intake);
    
    }
    public void execute() {
        Intake.intake();
    }
}
