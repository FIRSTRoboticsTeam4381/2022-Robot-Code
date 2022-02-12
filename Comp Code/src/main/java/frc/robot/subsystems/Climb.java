// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private WPI_TalonSRX rightArm = new WPI_TalonSRX(Constants.rSlide);
  private WPI_TalonSRX leftArm = new WPI_TalonSRX(Constants.lSlide);
  private WPI_TalonSRX topWinch = new WPI_TalonSRX(Constants.topWinch);
  private WPI_TalonSRX bottomWinch = new WPI_TalonSRX(Constants.bottomWinch);
  private WPI_TalonSRX winch3 = new WPI_TalonSRX(Constants.winch3);
  
  /** Creates a new Climb. */
  public Climb() {

  }
  
  public void activateArms(double speed){
    rightArm.set(speed);
    leftArm.set(speed);
  }

  public void activateTopWinch(double speed){
    topWinch.set(speed);
  }

  public void activateBottomWinch(double speed){
    bottomWinch.set(speed);
  }

  public void activateWinch3(double speed){
    winch3.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
