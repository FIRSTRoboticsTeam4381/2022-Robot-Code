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

public class Climb extends SubsystemBase {
  // TODO
  // FIGURE OUT WHICH MOTOR ID NUMBERS NEED TO BE USED & MOVE TO CONSTANTS
  private final int rightArmID = 0;
  private final int leftArmID = 0;
  private final int winch1id = 0;
  private final int winch2id = 0;
  private final int winch3id = 0;

  private WPI_TalonSRX rightArm = new WPI_TalonSRX(rightArmID);
  private WPI_TalonSRX leftArm = new WPI_TalonSRX(leftArmID);
  private WPI_TalonSRX winch1 = new WPI_TalonSRX(winch1id);
  private WPI_TalonSRX winch2 = new WPI_TalonSRX(winch2id);
  private WPI_TalonSRX winch3 = new WPI_TalonSRX(winch3id);
  
  /** Creates a new Climb. */
  public Climb() {
    leftArm.follow(rightArm);
  }
  
  public void activateArms(double speed){
    rightArm.set(speed);
  }

  public void activateWinch1(double speed){
    winch1.set(speed);
  }

  public void activateWinch2(double speed){
    winch2.set(speed);
  }

  public void activateWinch3(double speed){
    winch3.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
