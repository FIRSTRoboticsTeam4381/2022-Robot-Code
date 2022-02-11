package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {
  /** Creates a new IntakeCommand. */

  private Joystick m_stick = null;
  private Climb ClimbToo = null ;
  public ClimbCommand(Joystick joystick, Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_stick = joystick;
    ClimbToo = climb;
    addRequirements(climb);
  }
}