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
    //public WPI_TalonSRX intakeDeploy;

    public DigitalInput entrance;
    public DigitalInput middle;
    public DigitalInput top;

    //TODO - Set encoder constants
    private final double INTAKE_DOWN = 0000;
    private final double INTAKE_UP = 0000;

    public int state = 999;
    public boolean[] eyes = {false, false, false};

    //Shoter Velocity, Range, Auto Shot
    private final double range = 200;
    private double shootVelocity = 0;
    private int falseTimes = 0;
    private boolean fire = false;

    public IntakeIndex(){
        intake = new WPI_VictorSPX(Constants.intakeCAN);
        index = new WPI_VictorSPX(Constants.indexCAN);
        //intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployCAN);

        entrance = new DigitalInput(Constants.entranceDIO);
        middle = new DigitalInput(Constants.middleDIO);
        top = new DigitalInput(Constants.topDIO);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Entrance", !entrance.get());
        SmartDashboard.putBoolean("Middle", !middle.get());
        SmartDashboard.putBoolean("Top", !top.get());
        SmartDashboard.putNumber("Intake State", state);

        shootVelocity = Shooter.velocity;

        updateSwitches();
        intakeBalls();
    }

    public void intake(){
        intake.set(1);
    }

    public void nextBall(){
        index.set(0.5);
    }

    public void updateSwitches(){
        eyes[0] = !entrance.get();
        eyes[1] = !middle.get();
        eyes[2] = !top.get();
    }
    
    public int getCase(){
        if(Math.abs(shootVelocity-Constants.shooterSpeedRPM) < range && fire){
            return 888;
        }else{
            return 
            ((eyes[0])? 1:0)+
            ((eyes[1])? 2:0)+
            ((eyes[2])? 4:0);
        }
    }

    public void intakeBalls(){
        switch(state){
            case 1:
                state = (runUntil(1, true)? getCase(): 1);
                break;
            case 3:
                state = (runUntil(2, true)? getCase(): 3);
                break;
            case 666:
                index.set(-0.5);
                state = getCase();
                break;
            case 888:
                index.set(0.5);
                state = getCase();
                break;
            default:
                index.set(0);
                state = getCase();
                break;
        }
    }

    private boolean runUntil(int eye, boolean desired){
        boolean isfinished;
        if(eyes[eye] == desired){
            index.set(0);
            isfinished = true;
        }else{
            index.set(0.25);
            isfinished = false;
        }

        return isfinished;
    }

    public void deployIntake(){
        //intakeDeploy.set(ControlMode.Position, INTAKE_DOWN);
    }

    public void retractIntake(){
        //intakeDeploy.set(ControlMode.Position, INTAKE_UP);

    }

    public void setVelocity(double shootVel){
        shootVelocity = shootVel;
    }

    public void clearBalls(){
        state = 666;
        intake.set(-1);
    }

    public void zeroIntake(){
        intake.set(0);
        state = getCase();
    }

    public void zeroIndex(){
        state = getCase();
    }

    public void zeroBoth(){
        zeroIndex();
        zeroIntake();
    }

    private boolean current = false;
    private boolean prevState = false;

    public boolean shotBalls(){
        if(prevState && !current)
            falseTimes++;
        
        prevState = current;
        current = eyes[2];

        return falseTimes >= 2;
    }

    public void resetShot(){
        falseTimes = 0;
    }

    public boolean getEye(int eye){
        return eyes[eye];
    }
    public void fireBalls(boolean go){
        fire = go;
    }
}
