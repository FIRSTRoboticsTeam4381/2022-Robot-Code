package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndex extends SubsystemBase {
    
    public WPI_VictorSPX intake;
    public WPI_VictorSPX index;
    public WPI_TalonSRX intakeDeploy;

    public DigitalInput entrance;
    public DigitalInput middle;
    public DigitalInput top;

    //TODO - Set encoder constants
    private final double INTAKE_DOWN = 379;
    private final double INTAKE_UP = 8711;
    private String intakePos = "down";

    public int state = 999;
    public boolean[] eyes = {false, false, false};

    // MATT TESTING VARIABLES
    public boolean intakeLifted = false;
    public double targetLift = INTAKE_DOWN;
    private double intakeLiftSpeed = 0.5;
    private final double LIFT_SLOPE = -0.00012;
    private double encOffset = 50;

    //Shoter Velocity, Range, Auto Shot
    private final double range = 200;
    private double shootVelocity = 0;
    private int falseTimes = 0;
    private boolean fire = false;

    public IntakeIndex(){
        intake = new WPI_VictorSPX(Constants.intakeCAN);
        index = new WPI_VictorSPX(Constants.indexCAN);
        intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployCAN);

        intakeDeploy.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        entrance = new DigitalInput(Constants.entranceDIO);
        middle = new DigitalInput(Constants.middleDIO);
        top = new DigitalInput(Constants.topDIO);

        intakeLifted = intakeDeploy.getSelectedSensorPosition() > INTAKE_UP - encOffset ? true : false;
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

        // Have only one of these active
        //runIntakeLift();
        liftIntakeUntilFinished();
        
        SmartDashboard.putNumber("Intake Deploy Encoder", intakeDeploy.getSelectedSensorPosition());

        SmartDashboard.putNumber("Target Value", targetLift);
        SmartDashboard.putBoolean("Lifted", intakeLifted);

        /*
        if(intakePos.equals("down")){
            if(intakeDeploy.getSelectedSensorPosition() > INTAKE_DOWN){
                intakeDeploy.set(0.5);
            }else{
                intakeDeploy.set(0);
            }
        }else{
            if(intakeDeploy.getSelectedSensorPosition() < INTAKE_UP){
                intakeDeploy.set(0.5);
            }else{
                intakeDeploy.set(0);
            }
        }
        */
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


    public void runIntakeDeploy(double power){
        intakeDeploy.set(power);
    }
    
    public void switchIntakePos(){
        if(intakePos.equals("down")){
            intakePos = "up";
        }else{
            intakePos = "down";
        }
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

    public void liftIntake(){
        targetLift = intakeLifted ? INTAKE_DOWN : INTAKE_UP; // Set the desired position to the opposite that the lift is currently in
    }

    private void runIntakeLift(){
        if(!intakeLifted && targetLift == INTAKE_UP){
            //intakeDeploy.set(intakeLiftSpeed);
            intakeDeploy.set((LIFT_SLOPE * (targetLift - intakeDeploy.getSelectedSensorPosition())) + 0.5);

            intakeLifted = intakeDeploy.getSelectedSensorPosition() < INTAKE_UP - encOffset ? false : true; // As long as the encoder is less than INTAKE_UP, keep running
        }
        else if(intakeLifted && targetLift == INTAKE_DOWN){
            //intakeDeploy.set(-intakeLiftSpeed);
            intakeDeploy.set((LIFT_SLOPE * (targetLift - intakeDeploy.getSelectedSensorPosition())) + 0.5);

            intakeLifted = intakeDeploy.getSelectedSensorPosition() > INTAKE_DOWN + encOffset ? true : false; // As long as the encoder is greater than INTAKE_DOWN, keep running
                                                                                                // Once encoder threshold is crossed, this should stop running the motor
        }
        else{
            intakeDeploy.set(0);
        }
    }

    private void liftIntakeUntilFinished(){
        // This should do the same thing as the runIntakeLift method???????????????????
        boolean finishedLifting = targetLift == INTAKE_DOWN ? (intakeDeploy.getSelectedSensorPosition() > INTAKE_DOWN + encOffset ? false : true) : 
                        targetLift == INTAKE_UP ? (intakeDeploy.getSelectedSensorPosition() < INTAKE_UP - encOffset ? false : true) : true;

        if (!finishedLifting){
            intakeDeploy.set(targetLift - intakeDeploy.getSelectedSensorPosition() > 0 ? -intakeLiftSpeed : intakeLiftSpeed);
            //intakeDeploy.set((LIFT_SLOPE * (targetLift - intakeDeploy.getSelectedSensorPosition())) + 0.5);
        }
        else{
            intakeDeploy.set(0);
        }
    }



    // USE FOR TESTING OF ENCODER VALUES &  DIRECTION
    public void constantLift(double speed){
        intakeDeploy.set(speed);
    }
    // TEST

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
