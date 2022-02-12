package frc.robot.subsystems;

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

public class Shooter extends SubsystemBase {
    /*
    private ShuffleboardTab tab = Shuffleboard.getTab("PID Tuning");
    private NetworkTableEntry sP = tab.add("P", 0).getEntry();
    */
    public CANSparkMax frontMotor;
    public CANSparkMax backMotor;
    public RelativeEncoder frontEncoder;
    public RelativeEncoder backEncoder;

    private SparkMaxPIDController frontPIDController;
    private SparkMaxPIDController backPIDController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double maxRPM;

    private double SETPOINT = 0;
    
    public Shooter(){
        frontMotor = new CANSparkMax(Constants.frontShootCAN, MotorType.kBrushless);
        backMotor = new CANSparkMax(Constants.backShootCAN, MotorType.kBrushless);
        
        frontMotor.setSmartCurrentLimit(35);
        backMotor.setSmartCurrentLimit(35);

        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();

        backMotor.setInverted(false);
        frontMotor.setInverted(false);
        frontPIDController = frontMotor.getPIDController();
        backPIDController = backMotor.getPIDController();

        kP = 0.00005;
        kI = 0.000000004;
        kD = 0;
        kIz = 0;
        kFF = 0.00018;
        kMaxOutput = 1;
        kMinOutput = -1;

        maxRPM = 5200;
        
        frontPIDController.setP(kP);
        frontPIDController.setI(kI);
        frontPIDController.setD(kD);
        frontPIDController.setIZone(kIz);
        frontPIDController.setFF(kFF);
        frontPIDController.setOutputRange(kMinOutput, kMaxOutput);
        

        backPIDController.setP(kP);
        backPIDController.setI(kI);
        backPIDController.setD(kD);
        backPIDController.setIZone(kIz);
        backPIDController.setFF(kFF);
        backPIDController.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("kFF", kFF);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Motor Speed", -frontEncoder.getVelocity());
        SmartDashboard.putNumber("Back Motor Speed", backEncoder.getVelocity());
        SmartDashboard.putNumber("Desired Speed", SETPOINT);
        
        double setPoint = SETPOINT;

        if(-frontEncoder.getVelocity() < setPoint - 50){
            frontMotor.set(-setPoint/maxRPM);
            backMotor.set(setPoint/maxRPM);
            
        }else if(setPoint != 0){
            frontPIDController.setReference(-setPoint, ControlType.kVelocity);
            backPIDController.setReference(setPoint, ControlType.kVelocity);
        }else{
            frontMotor.set(0);
            backMotor.set(0);
        }
        //0.65 for up against wall
/*
        frontPIDController.setP(SmartDashboard.getNumber("kP", kP));
        frontPIDController.setI(SmartDashboard.getNumber("kI", kI));
        frontPIDController.setD(SmartDashboard.getNumber("kD", kD));
        frontPIDController.setFF(SmartDashboard.getNumber("kFF", kFF));

        backPIDController.setP(SmartDashboard.getNumber("kP", kP));
        backPIDController.setI(SmartDashboard.getNumber("kI", kI));
        backPIDController.setD(SmartDashboard.getNumber("kD", kD));
        backPIDController.setFF(SmartDashboard.getNumber("kFF", kFF));
        */
    }

    public void spinUP(double velocity){
        SETPOINT = (velocity <= 1)? velocity*maxRPM: velocity;
        //SETPOINT = (velocity <= 1)? velocity: velocity/maxRPM;
    }
}
