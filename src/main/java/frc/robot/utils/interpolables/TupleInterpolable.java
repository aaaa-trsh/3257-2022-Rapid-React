package frc.robot.utils.interpolables;

import edu.wpi.first.math.Pair;

public class TupleInterpolable implements Interpolable<Pair<Double, Double>> {
    private Pair<Double, Double> a;
    public TupleInterpolable(double x, double y) { this.a = new Pair<Double, Double>(x, y); }
    
    @Override
    public Pair<Double, Double> interpolate(Pair<Double, Double> b, Double t) {
        return new Pair<Double, Double>(
            Interpolable.lerp(a.getFirst(), b.getFirst(), t), 
            Interpolable.lerp(a.getSecond(), b.getSecond(), t)
        );
    }

    @Override
    public Double inverseInterpolate(Pair<Double, Double> b, Pair<Double, Double> query) {
        return Interpolable.inverseLerp(a.getFirst(), b.getFirst(), query.getFirst());
    }

    @Override
    public Pair<Double, Double> get() { return a; }


    // public static void main(String[] args) {
    //     var x = new Interpolable.InterpolatingTreeMap<Pair<Double, Double>>();
    //     x.put(0., new TupleInterpolable(-2, -2));
    //     x.put(0.5, new TupleInterpolable(-1, -1));
    //     x.put(.75, new TupleInterpolable(.3, .3));
    //     x.put(1., new TupleInterpolable(1, 1));

    //     for (int i = 0; i < 20; i++) {
    //         var ey = x.interpolate((double)i/20);
    //         System.out.println(ey.getFirst() + ","+ey.getSecond());
    //     }
    // }
}
