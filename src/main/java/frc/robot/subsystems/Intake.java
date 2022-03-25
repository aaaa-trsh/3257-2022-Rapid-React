// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.RelativeEnser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.utils.tunables.TunableNumber;

public class Intake extends SubsystemBase {
    private Spark rollerMotor = new Spark(IntakeConstants.rollerPort);
    private CANSparkMax liftMotor = new  CANSparkMax(IntakeConstants.liftPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private RelativeEncoder liftEncoder;
    private SparkMaxPIDController liftController;

    // private TunableNumber upPos = new TunableNumber("Intake/UpPosition", 6.);
    // private TunableNumber midPos = new TunableNumber("Intake/MidPosition", 5.);
    // private TunableNumber downPos = new TunableNumber("Intake/DownPosition", -2.);
    private double targetPos = 0;
    // private TunableNumber p = new TunableNumber("Intake/P", 0.6);
    // private TunableNumber d = new TunableNumber("Intake/D", 0.);

    private IntakeState intakeState = IntakeState.DOWN;

    public Intake() {
        // targetPos = upPos.get();
        liftMotor.restoreFactoryDefaults();
        liftEncoder = liftMotor.getEncoder();
        liftEncoder.setPosition(0);
        liftController = liftMotor.getPIDController();

        liftController.setP(1);
        liftController.setI(0);
        liftController.setD(0);
        liftController.setIZone(0);
        liftController.setFF(0);
        liftController.setOutputRange(-.1, .3);
    }

    // public void resetLiftEncoder() { liftEncoder.setPosition(0); }
    
    @Override
    public void periodic() {
        // if (p.hasChanged() | d.hasChanged()) {
        //     liftController.setP(p.get());
        //     liftController.setD(d.get());
        // }
        
        // switch (intakeState) {
        //     case UP:
        //         targetPos = upPos.get();
        //         break;
        //     case MID:
        //         targetPos = midPos.get();
        //         break;
        //     case DOWN:
        //         targetPos = downPos.get();
        //         break;
        //     default:
        //         targetPos = 0;
        //         break;
        //     }
        if (intakeState == IntakeState.UP) {
            liftController.setReference(7.5, CANSparkMax.ControlType.kPosition);
        } else {
            liftController.setReference(0, CANSparkMax.ControlType.kPosition);
        }
        SmartDashboard.putNumber("intake lift pos", liftEncoder.getPosition());
        SmartDashboard.putNumber("intake lift target pos", targetPos);
    }

    public void setRollerPercent(double percent) { rollerMotor.set(-percent); }
    public void setIntakePercent(double percent) { liftMotor.set(-percent); }
    
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
        // liftController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    }
    
    public static enum IntakeState {
		DOWN, MID, UP;
	}
}
