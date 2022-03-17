package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
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

    /*
    public Servo leftServo;
    public Servo rightServo;
    public final double FENDER_SHOOT_POS = 0;
    public double servoPosition = FENDER_SHOOT_POS;
*/
    private SparkMaxPIDController frontPIDController;
    private SparkMaxPIDController backPIDController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double maxRPM;

    private double SETPOINT = 0;
    
    public static double velocity = 0;
    
    public Shooter(){
        frontMotor = new CANSparkMax(Constants.frontShootCAN, MotorType.kBrushless);
        backMotor = new CANSparkMax(Constants.backShootCAN, MotorType.kBrushless);
        
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

/*
        leftServo = new Servo(Constants.leftServoPort);
        rightServo = new Servo(Constants.rightServoPort);

        leftServo.setBounds(2.0, 1.8, 1.5, 1.2, 1);
        rightServo.setBounds(2.0, 1.8, 1.5, 1.2, 1);

       leftServo.set(FENDER_SHOOT_POS);
        rightServo.set(FENDER_SHOOT_POS);
*/
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Motor Speed", -frontEncoder.getVelocity());
        SmartDashboard.putNumber("Back Motor Speed", backEncoder.getVelocity());
        SmartDashboard.putNumber("Combined Speed", velocity);
        SmartDashboard.putNumber("Desired Speed", SETPOINT);

        SmartDashboard.putNumber("Cutoff", Constants.cutoffSpeed);
       // SmartDashboard.putNumber("Left Servo", leftServo.getSpeed());
      //  SmartDashboard.putNumber("Right Servo", rightServo.getSpeed());

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

        velocity = (-frontEncoder.getVelocity() + backEncoder.getVelocity())/2;

      //  leftServo.setSpeed(servoPosition);
       // rightServo.setSpeed(servoPosition);
    }

    public void moveServo(double pos){
       // servoPosition = pos;
    }

    public void spinUP(double velocity){
        SETPOINT = (velocity <= 1)? velocity*maxRPM: velocity;
        //SETPOINT = (velocity <= 1)? velocity: velocity/maxRPM;
    }
}
