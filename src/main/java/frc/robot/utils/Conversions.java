package frc.robot.utils;

public class Conversions {
    public static class TalonFXConversions {
        public static double RPM2Native(double rpm) {
            return (double)rpm * 2048 * 60 * 10;
        }

        public static double Native2RPM(double units) {
            return units * ((double)1 / (2048 * 60 * 10));
        }

        public static double Native2Rotations(int units) {
            return (double)units / 2048;
        }

        public static double Rotations2Native(double rotations) {
            return (rotations * 2048);
        }
    }
}
