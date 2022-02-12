package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndex extends SubsystemBase {
    
    public WPI_VictorSPX intake;
    public WPI_VictorSPX index;
    public WPI_TalonSRX intakeDeploy;

    //TODO - Set encoder constants
    private final double INTAKE_DOWN = 0000;
    private final double INTAKE_UP = 0000;

    public IntakeIndex(){
        intake = new WPI_VictorSPX(Constants.intakeCAN);
        index = new WPI_VictorSPX(Constants.indexCAN);
        intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployCAN);
        intakeDeploy.configContinuousCurrentLimit(15);

    }

    public void intake(){
        intake.set(1);
    }

    public void nextBall(){
        index.set(0.5);
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
}
