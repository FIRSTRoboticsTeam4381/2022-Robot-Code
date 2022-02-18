package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndex extends SubsystemBase {
    
    public WPI_VictorSPX intake;
    public WPI_VictorSPX index;
    public WPI_TalonSRX intakeDeploy;

    public boolean[] eyes = {false, false, false};
    public DigitalInput entrance, middle, top;
    public int state = 999;

    //TODO - Set encoder constants
    private final double INTAKE_DOWN = 0000;
    private final double INTAKE_UP = 0000;

    public IntakeIndex(){
        intake = new WPI_VictorSPX(Constants.intakeCAN);
        index = new WPI_VictorSPX(Constants.indexCAN);
        intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployCAN);
        intakeDeploy.configContinuousCurrentLimit(15);

        entrance = new DigitalInput(Constants.entranceDIO);
        middle = new DigitalInput(Constants.middleDIO);
        top = new DigitalInput(Constants.topDIO);


    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Entrance", entrance.get());
        SmartDashboard.putBoolean("Middle", middle.get());
        SmartDashboard.putBoolean("Top", top.get());
        SmartDashboard.putNumber("Intake State", state);

        updateSwitches();
    }

    public void intake(){
        intake.set(1);
    }

    public void nextBall(){
        index.set(0.5);
    }

    public void updateSwitches(){
        eyes[0] = entrance.get();
        eyes[1] = middle.get();
        eyes[2] = top.get();
    }
    
    public int getCase(){
        return 
        ((eyes[0])? 1:0)+
        ((eyes[1])? 2:0)+
        ((eyes[2])? 4:0);
    }

    public void intakeBalls(){
        switch(state){
            case 2:
                state = (runUntil(1, true)? getCase(): 2);
                break;
            case 3:
                state = (runUntil(2, true)? getCase(): 3);
                break;
            default:
                intake.set(1);
                state = getCase();
                break;

        }
    }

    public void shootBalls(double velocity){
        double range = 50;
        if(Math.abs(velocity-Constants.shooterSpeedRPM) < range){
            index.set(0.5);
        }else{
            index.set(0);
        }
    }

    private boolean runUntil(int eye, boolean desired){
        boolean isfinished;
        if(eyes[eye] == desired){
            index.set(0);
            intake.set(0);
            isfinished = true;
        }else{
            index.set(0.5);
            intake.set(0.5);
            isfinished = false;
        }

        return isfinished;
    }

    public void deployIntake(){
        intakeDeploy.set(ControlMode.Position, INTAKE_DOWN);
    }

    public void retractIntake(){
        intakeDeploy.set(ControlMode.Position, INTAKE_UP);

    }

    public void clearBalls(){
        intake.set(-1);
        index.set(-1);
    }

    public void zeroIntake(){
        intake.set(0);
    }

    public void zeroIndex(){
        index.set(0);
    }

    public void zeroBoth(){
        zeroIndex();
        zeroIntake();
    }
}
