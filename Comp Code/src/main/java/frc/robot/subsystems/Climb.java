package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    //Declare motors
    public WPI_TalonSRX slide1;
    public WPI_TalonSRX slide2;
    public WPI_TalonSRX topWinch;
    public WPI_TalonSRX bottomWinch;

    //Honestly still dont know what winch 3 does
    public WPI_TalonSRX winch3;

    public Climb(){
        slide1 = new WPI_TalonSRX(Constants.slide1);
        slide2 = new WPI_TalonSRX(Constants.slide2);
        topWinch = new WPI_TalonSRX(Constants.topWinch);
        bottomWinch = new WPI_TalonSRX(Constants.bottomWinch);
        winch3 = new WPI_TalonSRX(Constants.winch3);

        /*
        lSlide.configContinuousCurrentLimit(20);
        rSlide.configContinuousCurrentLimit(20);
        topWinch.configContinuousCurrentLimit(40);
        bottomWinch.configContinuousCurrentLimit(40);
        winch3.configContinuousCurrentLimit(40);
        */

    }

    public void runSlides(double power){
        slide1.set(power);
        slide2.set(power);
    }

    public void runTopWinch(double power){
        topWinch.set(power);
    }
    public void runBottomWinch(double power){
        bottomWinch.set(power);
    }
    public void runWinch3(double power){
        winch3.set(power);
    }



    //Zero all motors in default command
    public void zeroClimb(){
        slide1.set(0);
        slide2.set(0);
        topWinch.set(0);
        bottomWinch.set(0);
        winch3.set(0);
    }
}
