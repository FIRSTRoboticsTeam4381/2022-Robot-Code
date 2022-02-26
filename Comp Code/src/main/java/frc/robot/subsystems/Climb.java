package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    private WPI_TalonSRX highWinch;
    private CANSparkMax lowWinch;

    public Climb(){
        highWinch = new WPI_TalonSRX(Constants.highWinchCAN);
        lowWinch = new CANSparkMax(Constants.lowWinchCAN, MotorType.kBrushless);
    }

    public void runHighWinch(int power){
        highWinch.set(power);
    }

    public void runLowWinch(int power){
        lowWinch.set(power);
    }
}
