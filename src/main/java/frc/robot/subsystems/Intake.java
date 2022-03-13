// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.tunables.TunableNumber;

public class Intake extends SubsystemBase {
    private Spark rollerMotor = new Spark(IntakeConstants.rollerPort);
    private CANSparkMax liftMotor = new  CANSparkMax(IntakeConstants.liftPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private RelativeEncoder liftEncoder;
    private SparkMaxPIDController liftController;

    private TunableNumber upPos = new TunableNumber("Intake/UpPosition", 0.);
    private TunableNumber p = new TunableNumber("Intake/P", 0.);
    private TunableNumber d = new TunableNumber("Intake/D", 0.);

    private boolean intakeUp = false;

    public Intake() {
        liftMotor.restoreFactoryDefaults();
        liftEncoder = liftMotor.getEncoder();
        liftController = liftMotor.getPIDController();

        liftController.setP(p.get());
        liftController.setI(0);
        liftController.setD(d.get());
        liftController.setIZone(0);
        liftController.setFF(0);
        liftController.setOutputRange(-1, 1);
    }
    
    @Override
    public void periodic() {
        if (p.hasChanged() | d.hasChanged()) {
            liftController.setP(p.get());
            liftController.setD(d.get());
        }
        
        if (intakeUp) {
            liftController.setReference(upPos.get(), CANSparkMax.ControlType.kPosition);
        }
        SmartDashboard.putNumber("intake lift pos", liftEncoder.getPosition());
    }

    public void setRollerPercent(double percent) { rollerMotor.set(-percent); }

    public void setIntakeUp(boolean up) {
        this.intakeUp = up;
        liftController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    }
}
