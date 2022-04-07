package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndex extends SubsystemBase {
    
    public CANSparkMax intake;
    public WPI_VictorSPX index;
    public CANSparkMax intakeDeploy;
    public RelativeEncoder intakeDeployEncoder;
    public SparkMaxPIDController intakeDeployPID;

    public DigitalInput entrance;
    public DigitalInput middle;
    public DigitalInput top;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double intakeDeployPos = 0;
    private final double INTAKE_UP = 70;
    private final double INTAKE_DOWN = 0;

    private double intakePower = 0;

    private double range = 200;
    public int state = 999;
    public boolean[] eyes = {false, false, false};

    //Shoter Velocity, Range, Auto Shot
    private double shootVelocity = 0;
    private int falseTimes = 0;
    private boolean fire = false;

    public IntakeIndex(){
        intake = new CANSparkMax(Constants.intakeCAN, MotorType.kBrushless);
        index = new WPI_VictorSPX(Constants.indexCAN);
        intakeDeploy = new CANSparkMax(Constants.intakeDeployCAN, MotorType.kBrushless);

        intakeDeployEncoder = intakeDeploy.getEncoder();
        intakeDeployEncoder.setPosition(0);
        intakeDeployPID = intakeDeploy.getPIDController();
        kP = 0.1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        intakeDeployPID.setP(kP);
        intakeDeployPID.setI(kI);
        intakeDeployPID.setD(kD);
        intakeDeployPID.setIZone(kIz);
        intakeDeployPID.setFF(kFF);
        intakeDeployPID.setOutputRange(kMinOutput, kMaxOutput);

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
        SmartDashboard.putNumber("Intake Deploy", intakeDeployEncoder.getPosition());
        SmartDashboard.putNumber("Low Shoot End", Constants.cutoffSpeed);
        SmartDashboard.putNumber("High Shoot End", Constants.shooterSpeedRPM+100);
        shootVelocity = Shooter.velocity;

        updateSwitches();
        intakeBalls();
      
        intakeDeployPID.setReference(intakeDeployPos, ControlType.kPosition);
        
        intake.set(-intakePower);

    }

    public void switchIntakeDeploy(){
        if(intakeDeployPos == INTAKE_UP){
            intakeDeployPos = INTAKE_DOWN;
        }else{
            intakeDeployPos = INTAKE_UP;
        }
    }

    public void runIntakeDeploy(double power){
        intakeDeploy.set(power);
    }

    public void intake(){
        intakePower = 1;
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
            if(eyes[1] && !eyes[2]){
                return 1001;
            }
            return 888;
        }else if(fire){
            return 999;
        }{
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
            case 555:
                index.set(0);
                break;
            case 666:
                index.set(-0.5);
                state = getCase();
                break;
            case 888:
                index.set(0.5);
                state = getCase();
                break;
            case 1001:
                index.set(0.5);
                state = getCase();
                break;
            default:
                index.set(0);
                state = getCase();
                break;
        }
    }

    public boolean startMatchReady(){
        intakePower = -1;
        state = 555;
        return !eyes[0];
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

    public void setVelocity(double shootVel){
        shootVelocity = shootVel;
    }

    public void clearBalls(){
        state = 666;
        intakePower = -1;
    }

    public void zeroIntake(){
        intakePower = 0;
        state = getCase();
    }

    public void resetState(){
        state = getCase();
    }

    public void zeroBoth(){
        resetState();
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
