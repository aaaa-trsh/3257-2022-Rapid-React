package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Conversions.TalonFXConversions;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX lowerFlywheel = new WPI_TalonFX(ShooterConstants.lowerFlywheelPort);
    private WPI_TalonFX upperFlywheel = new WPI_TalonFX(ShooterConstants.upperFlywheelPort);
    
    // ooga booga revert to bangbang control
    private BangBangController controller = new BangBangController();
    private double lowerRPM, upperRPM;
    // private LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(ShooterConstants.flywheelV, ShooterConstants.flywheelA);

    // private KalmanFilter<N1, N1, N1> flywheel1Observer = new KalmanFilter<>(
    //     Nat.N1(), Nat.N1(),
    //     flywheelPlant,
    //     VecBuilder.fill(3.0),
    //     VecBuilder.fill(0.01),
    //     0.02
    // );
    // private KalmanFilter<N1, N1, N1> flywheel2Observer = new KalmanFilter<>(
    //     Nat.N1(), Nat.N1(),
    //     flywheelPlant,
    //     VecBuilder.fill(3.0),
    //     VecBuilder.fill(0.01),
    //     0.02
    // );

    // private LinearQuadraticRegulator<N1, N1, N1> flywheel1Controller = new LinearQuadraticRegulator<>(
    //     flywheelPlant,
    //     VecBuilder.fill(8.0),
    //     VecBuilder.fill(12.0),
    //     0.02
    // );
    // private LinearQuadraticRegulator<N1, N1, N1> flywheel2Controller = new LinearQuadraticRegulator<>(
    //     flywheelPlant,
    //     VecBuilder.fill(8.0),
    //     VecBuilder.fill(12.0),
    //     0.02
    // );

    // private LinearSystemLoop<N1, N1, N1> flywheel1Loop = new LinearSystemLoop<>(flywheelPlant, flywheel1Controller, flywheel1Observer, 12.0, 0.020);
    // private LinearSystemLoop<N1, N1, N1> flywheel2Loop = new LinearSystemLoop<>(flywheelPlant, flywheel2Controller, flywheel2Observer, 12.0, 0.020);
    // private boolean flywheelSpinning = false;

    public Shooter() {
        lowerFlywheel.configFactoryDefault();
        upperFlywheel.configFactoryDefault();

        lowerFlywheel.setInverted(InvertType.InvertMotorOutput);
        lowerFlywheel.setNeutralMode(NeutralMode.Coast);

        upperFlywheel.setInverted(InvertType.None);
        upperFlywheel.setNeutralMode(NeutralMode.Coast);
    }
    
    public setShooterSpeeds(double lowerRPM, double lowerRPM) {
        this.lowerRPM = lowerRPM;
        this.upperRPM = upperRPM;
    }

    @Override
    public void periodic() {
        lowerFlywheel.set(controller.calculate(TalonFXConversions.Native2RPM(lowerFlywheel.getSelectedSensorVelocity()), lowerRPM));
        upperFlywheel.set(controller.calculate(TalonFXConversions.Native2RPM(upperFlywheel.getSelectedSensorVelocity()), upperRPM));
        
        SmartDashboard.putNumber("lower flywheel rpm", TalonFXConversions.Native2RPM(lowerFlywheel.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("upper flywheel rpm", TalonFXConversions.Native2RPM(upperFlywheel.getSelectedSensorVelocity()));

        SmartDashboard.putNumber("lower flywheel target", lowerRPM);
        SmartDashboard.putNumber("upper flywheel target", upperRPM);

        // flywheel1Loop.setNextR(VecBuilder.fill(flywheelSpinning ? ShooterConstants.spinupRadPerSec : 0.));
        // flywheel2Loop.setNextR(VecBuilder.fill(flywheelSpinning ? ShooterConstants.spinupRadPerSec : 0.));

        // flywheel1Loop.correct(VecBuilder.fill(2. * Math.PI * flywheel1TalonFX.getSelectedSensorVelocity() / DriveConstants.encoderCountsPerRotation));
        // flywheel2Loop.correct(VecBuilder.fill(2. * Math.PI * flywheel2TalonFX.getSelectedSensorVelocity() / DriveConstants.encoderCountsPerRotation));

        // flywheel1Loop.predict(0.020);
        // flywheel2Loop.predict(0.020);

        // flywheel1TalonFX.setVoltage(flywheel1Loop.getU(0));
        // flywheel2TalonFX.setVoltage(flywheel2Loop.getU(0));

    }
}
