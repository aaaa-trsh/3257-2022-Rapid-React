package frc.robot.utils.interpolables;

public class DoubleInterpolable implements Interpolable<Double>, Comparable<DoubleInterpolable> {
    private double a;
    public DoubleInterpolable(double a) { this.a = a; }
    
    @Override
    public Double interpolate(Double b, Double t) {
        return t * (b - a) + a;
    }

    @Override
    public Double inverseInterpolate(Double b, Double query) {
        return (query - a) / (b - a);
    }

    @Override
    public Double get() { return a; }
    
    @Override
    public int compareTo(DoubleInterpolable o) {
        if (o.get() < a) { return 1; } 
        else if (o.get() > a) { return -1; } 
        else { return 0; }
    }

    // public static void main(String[] args) {
    //     var x = new Interpolable.InterpolatingTreeMap<Double>();
    //     x.put(0., new DoubleInterpolable(-2));
    //     x.put(0.5, new DoubleInterpolable(-1));
    //     x.put(.75, new DoubleInterpolable(.3));
    //     x.put(1., new DoubleInterpolable(1));

    //     for (int i = 0; i < 20; i++) {
    //         System.out.println(x.interpolate((double)i/20));
    //     }
    // }
}
