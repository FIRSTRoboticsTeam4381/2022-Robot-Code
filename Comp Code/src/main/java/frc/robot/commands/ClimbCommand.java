// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {
  private Joystick input;
  private Climb climb;
  
  // BUTTONS
  private final int extendButton = 5;
  private final int retractButton = 3;

  private final int winch1F = 6;
  private final int winch1R = 4;

  private final int winch2F = 7;
  private final int winch2R = 9;

  private final int winch3F = 8;
  private final int winch3R = 10;

  // SPEEDS
  private final double winchSpeed = 0.4;
  private final double armSpeed = 0.5;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(Joystick input, Climb climb) {
    this.input = input;
    this.climb = climb;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ARMS
    climb.activateArms(input.getRawButton(extendButton) ? armSpeed : input.getRawButton(retractButton) ? -armSpeed : 0);

    // WINCH 1
    climb.activateWinch1(input.getRawButton(winch1F) ? winchSpeed : input.getRawButton(winch1R) ? -winchSpeed : 0);

    // WINCH 2
    climb.activateWinch2(input.getRawButton(winch2F) ? winchSpeed : input.getRawButton(winch2R) ? -winchSpeed : 0);

    // WINCH 3
    climb.activateWinch3(input.getRawButton(winch3F) ? winchSpeed : input.getRawButton(winch3R) ? -winchSpeed : 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
