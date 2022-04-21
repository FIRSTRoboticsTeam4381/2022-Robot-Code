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
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  private final Joystick climbTesting = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 2;

  /* One Stick Driver Buttons */
  /*
  private final JoystickButton zeroSwerve1 = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton shootButton1 = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton clearBalls1 = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton intakeButton1 = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  */
  /* Two Stick Driver Buttons */
  private final JoystickButton zeroSwerve2 = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton shootButton2 = new JoystickButton(specials, XboxController.Button.kX.value);
  private final JoystickButton clearBalls2 = new JoystickButton(specials, XboxController.Button.kY.value);
  private final JoystickButton intakeButton2 = new JoystickButton(specials, XboxController.Button.kB.value);
  private final JoystickButton oneButtonClimbMain = new JoystickButton(driver, 10);
  private final JoystickButton oneButtonClimbCancel = new JoystickButton(driver, 9);
  private final JoystickButton intakeDown = new JoystickButton(specials, XboxController.Button.kLeftBumper.value);
  private final POVButton oneButtonMidClimb = new POVButton(specials, 0);
  //Testing
  private final JoystickButton intakeUpTesting = new JoystickButton(climbTesting, 5);
  private final JoystickButton intakeDownTesting = new JoystickButton(climbTesting, 6);
  private final JoystickButton slapBarWinchOut = new JoystickButton(climbTesting, 11);
  private final JoystickButton slapBarWinchIn = new JoystickButton(climbTesting, 12);
  private final JoystickButton topHookButtonUp = new JoystickButton(climbTesting, 7);
  private final JoystickButton topHookButtonOut = new JoystickButton(climbTesting, 8);
  private final JoystickButton mainWinchButtonOut = new JoystickButton(climbTesting, 9);
  private final JoystickButton mainWinchButtonIn = new JoystickButton(climbTesting, 10);
  private final JoystickButton changeServoPos = new JoystickButton(climbTesting, 2);



  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final IntakeIndex intakeIndex = new IntakeIndex();
  private final Shooter shooter = new Shooter();
  private final Climb climb = new Climb();

  private final Command fourBall = new FourBall(s_Swerve, intakeIndex, shooter);
  private final Command threeBall = new ThreeBall(s_Swerve, intakeIndex, shooter);
  private final Command threeBallAlt = new ThreeBallAlt(s_Swerve, intakeIndex, shooter);
  private final Command twoBall = new TwoBall(s_Swerve, intakeIndex, shooter);
  private final Command fourBallAlt = new FourBallAlt(s_Swerve, intakeIndex, shooter);
  private final Command evilTwoBall = new TwoBallEvilAuto(s_Swerve, intakeIndex, shooter);

  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, openLoop));
    intakeIndex.setDefaultCommand(new IntakeIndexCommand(intakeIndex));
    shooter.setDefaultCommand(new ShootCommand(shooter));
    climb.setDefaultCommand(new ClimbCommand(climb));
    
    SmartDashboard.putNumber("Setpoint", 0);

    m_AutoChooser.setDefaultOption("4 Ball", fourBall);
    m_AutoChooser.addOption("4 Ball Alt", fourBallAlt);
    m_AutoChooser.addOption("3 Ball", threeBall);
    m_AutoChooser.addOption("3 Ball Alt", threeBallAlt);
    m_AutoChooser.addOption("2 Ball", twoBall);
    m_AutoChooser.addOption("evil 2Ball", evilTwoBall);

    SmartDashboard.putData(m_AutoChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    one stick
      
      zeroSwerve1.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
      clearBalls1.whenHeld(new StartEndCommand(() -> intakeIndex.clearBalls(), () -> intakeIndex.zeroIntake()));
      shootButton1.whenHeld(new StartEndCommand(() -> shooter.spinUP(Constants.shooterSpeedPercent), () -> shooter.spinUP(0))
      .alongWith(new InstantCommand(() -> intakeIndex.fireBalls(true))));
      intakeButton1.whenHeld(new StartEndCommand(() -> intakeIndex.intake(), () -> intakeIndex.zeroIntake()));

   */   
      // Climb Testing
      /*
      topHookButtonOut.whenHeld(new StartEndCommand(() -> climb.runTopHook(1), () -> climb.runTopHook(0)));
      topHookButtonUp.whenHeld(new StartEndCommand(() -> climb.runTopHook(-1), () -> climb.runTopHook(0)));
      mainWinchButtonIn.whenHeld(new StartEndCommand(() -> climb.runMainWinch(1), () -> climb.runMainWinch(0)));
      mainWinchButtonOut.whenHeld(new StartEndCommand(() -> climb.runMainWinch(-1), () -> climb.runMainWinch(0)));
      slapBarWinchIn.whenHeld(new StartEndCommand(() -> climb.runSlapBar(1), () -> climb.runSlapBar(0)));
      slapBarWinchOut.whenHeld(new StartEndCommand(() -> climb.runSlapBar(-1), () -> climb.runSlapBar(0)));

      changeServoPos.whenHeld(new InstantCommand(() -> shooter.moveServo(climbTesting.getThrottle())));
      */
     // intakeUpTesting.whenHeld(new StartEndCommand(() -> intakeIndex.runIntakeDeploy(-1), () -> intakeIndex.runIntakeDeploy(0)));
      //intakeDownTesting.whenHeld(new StartEndCommand(() -> intakeIndex.runIntakeDeploy(1), () -> intakeIndex.runIntakeDeploy(0)));

      intakeDown.whenPressed(new InstantCommand(() -> intakeIndex.switchIntakeDeploy()));

      oneButtonClimbMain.whenPressed(new InstantCommand(() -> climb.nextState()));
      oneButtonClimbCancel.whenPressed(new InstantCommand(() -> climb.cancelClimb()));
      
      oneButtonMidClimb.whenPressed(new InstantCommand(() -> climb.nextMidState()));

      zeroSwerve2.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
      clearBalls2.whenHeld(new StartEndCommand(() -> intakeIndex.clearBalls(), () -> intakeIndex.zeroIntake()));
      shootButton2.whenHeld(new StartEndCommand(() -> shooter.spinUP(Constants.shooterSpeedPercent), () -> shooter.spinUP(0))
      .alongWith(new InstantCommand(() -> intakeIndex.fireBalls(true))));
      intakeButton2.whenHeld(new StartEndCommand(() -> intakeIndex.intake(), () -> intakeIndex.zeroIntake())
      .alongWith(new InstantCommand(() -> intakeIndex.fireBalls(false))));
    
    
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
