// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick specials = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 2;
  private final int throttleAxis = 4;

  /* One Stick Driver Buttons */
  private final JoystickButton zeroSwerve1 = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton shootButton1 = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton clearBalls1 = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton intakeButton1 = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  
  /* Two Stick Driver Buttons */
  private final JoystickButton zeroSwerve2 = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton shootButton2 = new JoystickButton(specials, XboxController.Button.kX.value);
  private final JoystickButton clearBalls2 = new JoystickButton(specials, XboxController.Button.kY.value);
  private final JoystickButton intakeButton2 = new JoystickButton(specials, XboxController.Button.kB.value);

  //Testing
  private final JoystickButton winch1OutButton = new JoystickButton(specials, XboxController.Button.kX.value);
  private final JoystickButton winch1InButton = new JoystickButton(specials, XboxController.Button.kY.value);
  private final JoystickButton winch2OutButton = new JoystickButton(specials, XboxController.Button.kA.value);
  private final JoystickButton winch2InButton = new JoystickButton(specials, XboxController.Button.kB.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final IntakeIndex intakeIndex = new IntakeIndex();
  private final Shooter shooter = new Shooter();

  private final Command fourBall = new FourBall(s_Swerve, intakeIndex, shooter);
  private final Command threeBall = new ThreeBall(s_Swerve, intakeIndex, shooter);

  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
  SendableChooser<String> m_ControlChooser = new SendableChooser<>();
  private final String single = "Single";
  private final String split = "Split";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, throttleAxis, fieldRelative, openLoop));
    intakeIndex.setDefaultCommand(new IntakeIndexCommand(intakeIndex));
    shooter.setDefaultCommand(new ShootCommand(shooter));
    
    SmartDashboard.putNumber("Setpoint", 0);

    m_AutoChooser.addOption("4 Ball", fourBall);
    m_AutoChooser.addOption("3 Ball", threeBall);

    m_ControlChooser.addOption("Single", single);
    m_ControlChooser.setDefaultOption("Split", split);

    SmartDashboard.putData(m_AutoChooser);
    SmartDashboard.putData(m_ControlChooser);
    // Configure the button bindings
    configureButtonBindings(m_ControlChooser.getSelected());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(String controls) {

    if(controls.equals("Single")){
      /* One Joystick Driver Buttons */
      zeroSwerve1.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
      clearBalls1.whenHeld(new StartEndCommand(() -> intakeIndex.clearBalls(), () -> intakeIndex.zeroIntake()));
      shootButton1.whenHeld(new StartEndCommand(() -> shooter.spinUP(Constants.shooterSpeedPercent), () -> shooter.spinUP(0))
      .alongWith(new InstantCommand(() -> intakeIndex.fireBalls(true))));
      intakeButton1.whenHeld(new StartEndCommand(() -> intakeIndex.intake(), () -> intakeIndex.zeroIntake()));

      //Testing
      winch1InButton.

    }else{
      zeroSwerve2.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
      clearBalls2.whenHeld(new StartEndCommand(() -> intakeIndex.clearBalls(), () -> intakeIndex.zeroIntake()));
      shootButton2.whenHeld(new StartEndCommand(() -> shooter.spinUP(Constants.shooterSpeedPercent), () -> shooter.spinUP(0))
      .alongWith(new InstantCommand(() -> intakeIndex.fireBalls(true))));
      intakeButton2.whenHeld(new StartEndCommand(() -> intakeIndex.intake(), () -> intakeIndex.zeroIntake()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }
}
