package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.tunables.TunableNumber;

public class Climber extends SubsystemBase {
    private WPI_TalonSRX arm1 = new WPI_TalonSRX(ClimbConstants.arm1Port);
    // private DutyCycleEncoder arm1Encoder = new DutyCycleEncoder(ClimbConstants.arm1EncoderPort);
    private WPI_TalonSRX arm2 = new WPI_TalonSRX(ClimbConstants.arm2Port);
    // private DutyCycleEncoder arm2Encoder = new DutyCycleEncoder(ClimbConstants.arm1EncoderPort);

    private TunableNumber p = new TunableNumber("Climber/P", 0.002);
    private TunableNumber d = new TunableNumber("Climber/D", 0.);
    
    private TunableNumber upPos = new TunableNumber("Climber/UpPosition", 0.);

    private PIDController controller;

    private boolean enabled = false;
    // private boolean arm1Up = false;
    // private boolean arm2Up = false;

    public Climber() {
        arm1.setNeutralMode(NeutralMode.Brake);
        // arm1Encoder.reset();
        // arm1Encoder.setDistancePerRotation(ClimbConstants.encoderCPR);

        arm2.setNeutralMode(NeutralMode.Brake);
        // arm2Encoder.reset();
        // arm2Encoder.setDistancePerRotation(ClimbConstants.encoderCPR);
        
        controller = new PIDController(p.get(), 0, d.get());
    }

    public void setEnabled(boolean on) { this.enabled = on; }
    // public void setArm1Up(boolean up) { this.arm1Up = up; }
    public void setArm1(double speed) { arm1.set(ControlMode.PercentOutput, speed); }
    // public void setArm2Up(boolean up) { this.arm2Up = up; }
    public void setArm2(double speed) { arm2.set(ControlMode.PercentOutput, speed); }

    @Override
    public void periodic() {
        if (p.hasChanged() | d.hasChanged()) {
            controller.setP(p.get());
            controller.setD(d.get());
        }

        if (upPos.hasChanged()) { controller.setSetpoint(upPos.get()); }

        if (enabled) {
            // arm1.set(ControlMode.PercentOutput, arm1Up ? controller.calculate(arm1Encoder.getDistance()) : 0);
            // arm2.set(ControlMode.PercentOutput, arm2Up ? controller.calculate(arm2Encoder.getDistance()) : 0);
        } else {
            // arm1.set(ControlMode.PercentOutput, 0);
            // arm2.set(ControlMode.PercentOutput, 0);
        }
    }
}
