package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    //Declare motors
    public WPI_TalonSRX lSlide;
    public WPI_TalonSRX rSlide;
    public WPI_TalonSRX topWinch;
    public WPI_TalonSRX bottomWinch;

    //Honestly still dont know what winch 3 does
    public WPI_TalonSRX winch3;

    public Climb(){
        lSlide = new WPI_TalonSRX(Constants.lSlide);
        rSlide = new WPI_TalonSRX(Constants.rSlide);
        topWinch = new WPI_TalonSRX(Constants.topWinch);
        bottomWinch = new WPI_TalonSRX(Constants.bottomWinch);
        winch3 = new WPI_TalonSRX(Constants.winch3);

        lSlide.configContinuousCurrentLimit(20);
        rSlide.configContinuousCurrentLimit(20);
        topWinch.configContinuousCurrentLimit(40);
        bottomWinch.configContinuousCurrentLimit(40);
        winch3.configContinuousCurrentLimit(40);


    }

    public void runSlides(double power, boolean reverse){
        lSlide.set((reverse)?-power:power);
        rSlide.set((reverse)?-power:power);
    }

    public void runTopWinch(double power, boolean reverse){
        topWinch.set((reverse)?-power:power);
    }
    public void runBottomWinch(double power, boolean reverse){
        bottomWinch.set((reverse)?-power:power);
    }
    public void runWinch3(double power, boolean reverse){
        winch3.set((reverse)?-power:power);
    }

    

    //Zero all motors in default command
    public void zeroClimb(){
        lSlide.set(0);
        rSlide.set(0);
        topWinch.set(0);
        bottomWinch.set(0);
        winch3.set(0);
    }
}
