package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    private WPI_TalonSRX slapBar;
    private WPI_TalonSRX topHooks;
    private CANSparkMax mainWinch;
    private RelativeEncoder mainWinchEnc;


    private SparkMaxPIDController mainWinchPID;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private int mainWinchState = 0;
    private int slapState = 0;
    private int topHookState = 0;
    private int climbState = 0;

    private double slapBarPower = 0;
    private double mainWinchPower = 0;
    private double topWinchPower = 0;
    private double targetPos = -1000;
    private double upLimit = -2053;
    private double downLimit = 0;

    public Climb(){
        slapBar = new WPI_TalonSRX(Constants.highWinchCAN);
        topHooks = new WPI_TalonSRX(Constants.slapBarCAN);
        mainWinch = new CANSparkMax(Constants.lowWinchCAN, MotorType.kBrushless);
        mainWinchEnc = mainWinch.getEncoder();

        slapBar.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        topHooks.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        mainWinchEnc.setPosition(0);
        mainWinchPID = mainWinch.getPIDController();
        kP = 0.1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        mainWinchPID.setP(kP);
        mainWinchPID.setI(kI);
        mainWinchPID.setD(kD);
        mainWinchPID.setIZone(kIz);
        mainWinchPID.setFF(kFF);
        mainWinchPID.setOutputRange(kMinOutput, kMaxOutput);



        slapBar.setInverted(true);
   

        slapBar.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        slapBar.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                            0,
				                            10);

		/* Ensure sensor is positive when output is positive */
		slapBar.setSensorPhase(true);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		slapBar.setInverted(true);

		/* Config the peak and nominal outputs, 12V means full */
		slapBar.configNominalOutputForward(1, 10);
		slapBar.configNominalOutputReverse(-1, 10);
		slapBar.configPeakOutputForward(1, 10);
		slapBar.configPeakOutputReverse(-1, 10);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		slapBar.configAllowableClosedloopError(0, 0, 10);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		slapBar.config_kF(0, 0, 10);
		slapBar.config_kP(0, 0.01, 10);
		slapBar.config_kI(0, 0, 10);
		slapBar.config_kD(0, 0, 10);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		
		/* Set the quadrature (relative) sensor to match absolute */
		slapBar.setSelectedSensorPosition(0, 0, 10);


        
        topHooks.setInverted(true);
   

        topHooks.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        topHooks.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                            0,
				                            10);

		/* Ensure sensor is positive when output is positive */
		topHooks.setSensorPhase(true);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		topHooks.setInverted(true);

		/* Config the peak and nominal outputs, 12V means full */
		topHooks.configNominalOutputForward(1, 10);
		topHooks.configNominalOutputReverse(-1, 10);
		topHooks.configPeakOutputForward(1, 10);
		topHooks.configPeakOutputReverse(-1, 10);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		topHooks.configAllowableClosedloopError(0, 0, 10);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		topHooks.config_kF(0, 0, 10);
		topHooks.config_kP(0, 0.01, 10);
		topHooks.config_kI(0, 0, 10);
		topHooks.config_kD(0, 0, 10);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		
		/* Set the quadrature (relative) sensor to match absolute */
		topHooks.setSelectedSensorPosition(0, 0, 10);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("topHook enc", slapBar.getSelectedSensorPosition());
        SmartDashboard.putNumber("slapBar enc", topHooks.getSelectedSensorPosition());
        SmartDashboard.putNumber("main winch enc", mainWinchEnc.getPosition());

        double mainPosition = 0;
        double slapPosition = 0;
        double topHookPos = 0;

        switch(climbState){
            //All down
            case 0:
                mainPosition = 0;
                slapPosition = 0;
                topHookPos = 0;
                break;
            //Main Up to go under low bar
            case 1:
                mainPosition = -333;
                slapPosition = 0;
                topHookPos = 0;
                break;
            //Main all up, slap 1/2 out, reaper out
            case 2:
                IntakeIndex.switchIntakeDeploy(1);
                mainPosition = -530;
                slapPosition = -4604;
                topHookPos = -1973;
                break;
            //Main in 1/2 way
            case 3:{
                mainPosition = -200;
                slapPosition = -4604;
                topHookPos = -1973;
                if(mainWinchEnc.getPosition() > -250){
                    climbState=4;
                }
                break;
            }
            //Slap all out
            case 4:
                mainPosition = -200;
                slapPosition = -11000;
                topHookPos = -1973;
                break;
            //Main all in
            case 5:
                mainPosition = -50;
                slapPosition = -11000;
                topHookPos = -1973;
                break;
            //Slap all in
            case 6:
                mainPosition = -50;
                slapPosition = 0;
                topHookPos = -1973;
                break;
            //Reaper clamp
            case 7:
                mainPosition = -50;
                slapPosition = 0;
                topHookPos = -1664;
                break;
            //Main all out
            case 8:
                mainPosition = -333;
                slapPosition = -1000;
                topHookPos = -1664;
                break;
            //Slap all out, reaper loose
            case 9:
                mainPosition = -333;
                slapPosition = -11000;
                topHookPos = 9999;
                break;
            // Bring in main hook
            case 10:
                mainPosition = 0;
                slapPosition = -11000;
                topHookPos = 9999;
                break;
            case 20:
                mainPosition = -333;
                slapPosition = 0;
                topHookPos = 0;
                break;
            case 21:
                mainPosition = -530;
                slapPosition = 0;
                topHookPos = 0;
                break;
            case 22:
                mainPosition = -30;
                slapPosition = 0;
                topHookPos = 0;
                break;
            case 23:
                mainPosition = 0;
                slapPosition = 0;
                topHookPos = 0;
                break;
            case 999:
                mainPosition = mainWinchEnc.getPosition();
                slapPosition = slapBar.getSelectedSensorPosition();
                topHookPos = topHooks.getSelectedSensorPosition();
                break;
        }

        mainWinchPID.setReference(mainPosition, ControlType.kPosition);
        slapBar.set(TalonSRXControlMode.Position, slapPosition);
        
        if(topHookPos == 9999){
            topHooks.setNeutralMode(NeutralMode.Coast);
            topHooks.set(0);
        }else{ 
            topHooks.setNeutralMode(NeutralMode.Brake);
            topHooks.set(TalonSRXControlMode.Position, topHookPos);
        }
        SmartDashboard.putNumber("Climb State", climbState);
    }

    public void nextState(){
        if(climbState < 10){
            climbState++;
        }else{
            climbState = 0;
        }
    }

    public void nextMidState(){
        if(climbState < 20){
            climbState = 20;
        }else if(climbState < 24){
            climbState++;
        }else{
            climbState = 0;
        }
    }

    public void cancelClimb(){
        climbState = 999;
    }

    public void runSlapBar(double power){
       //slapBar.set(power); 
       if(slapState < 2){
            slapState++;
        }else{
            slapState = 0;
        }

    }

    public void runMainWinch(double power){
        
        if(mainWinchState < 3){
            mainWinchState++;
        }else{
            mainWinchState = 0;
        }
        
        //mainWinch.set(power);
    }

    public void runTopHook(double power){
        
        if(topHookState < 3){
            topHookState++;
        }else{
            topHookState = 0;
        }
        
        //topHooks.set(power);
    }
}
