package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    private WPI_TalonSRX topHook;
    private WPI_TalonSRX slapBar;
    private CANSparkMax mainWinch;
    private RelativeEncoder mainWinchEnc;

    private double slapBarPower = 0;
    private double mainWinchPower = 0;
    private double topWinchPower = 0;
    private double targetPos = -1000;
    private double upLimit = -2053;
    private double downLimit = 0;

    public Climb(){
        topHook = new WPI_TalonSRX(Constants.highWinchCAN);
        slapBar = new WPI_TalonSRX(Constants.slapBarCAN);
        mainWinch = new CANSparkMax(Constants.lowWinchCAN, MotorType.kBrushless);
        mainWinchEnc = mainWinch.getEncoder();

        topHook.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        slapBar.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        mainWinch.setInverted(false);

        topHook.setInverted(true);
   
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("topHook enc", topHook.getSelectedSensorPosition());
        SmartDashboard.putNumber("slapBar enc", slapBar.getSelectedSensorPosition());
        SmartDashboard.putNumber("main winch enc", mainWinchEnc.getPosition());

        if ( slapBarPower > 0 )
        {
            if ( slapBar.getSelectedSensorPosition() < -1600 )
            {
                double powerMod = Math.abs(-1865 - slapBar.getSelectedSensorPosition() )/200;
                slapBarPower = slapBarPower * powerMod;
                if ( slapBar.getSelectedSensorPosition() < -1865 )
                    slapBarPower = 0;
            }
           
        }
        slapBar.set(slapBarPower);

        if (mainWinchPower < 0 )
        {
            if ( mainWinchEnc.getPosition() < 15 )
            {
                double powerMod = Math.abs(0 - mainWinchEnc.getPosition() )/15;
                mainWinchPower = mainWinchPower * powerMod;
                if ( mainWinchEnc.getPosition() < 0 )
                mainWinchPower = 0;
            }
           
        }
        mainWinch.set(mainWinchPower);

        if (topWinchPower < 0 )
        {
            if ( topHook.getSelectedSensorPosition() > -170 )
            {
                double powerMod = Math.abs(0 - topHook.getSelectedSensorPosition() )/170;
                topWinchPower = topWinchPower * powerMod;
                if ( topHook.getSelectedSensorPosition() > 0 )
                topWinchPower = 0;
            }
           
        }
        
        topHook.set(topWinchPower);
        
    }

    public void runTopHook(double power){
        topWinchPower = power;
    }

    public void runMainWinch(double power){
        mainWinchPower = power;
    }

    public void runSlapBar(double power){
        this.slapBarPower = power;
    }
}
