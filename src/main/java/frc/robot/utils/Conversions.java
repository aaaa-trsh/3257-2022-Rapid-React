package frc.robot.utils;

public class Conversions {
    public static class TalonFXConversions {
        public static double RPM2Native(double rpm) {
            return rpm * 2048. / 10. / 60.;
        }

        public static double Native2RPM(double units) {
            return units / 2048. * 600.; // 0 - 100
        }

        public static double Native2Rotations(double units) {
            return units / 2048.;
        }

        public static double Rotations2Native(double rotations) {
            return rotations * 2048.;
        }
    }
}
