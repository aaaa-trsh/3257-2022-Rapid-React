package frc.robot.utils.interpolables;

import java.util.TreeMap;

public interface Interpolable<T> {
    public static class InterpolatingTreeMap<T> {
        private TreeMap<Double, Interpolable<T>> map = new TreeMap<Double, Interpolable<T>>();
        
        public InterpolatingTreeMap() {}
        public InterpolatingTreeMap(TreeMap<Double, Interpolable<T>> map) { this.map = map; }
        public void put(Double key, Interpolable<T> value) { map.put(key, value); }
        public String toString() { return map.toString(); }
        public int size() { return map.size(); }
        public TreeMap<Double, Interpolable<T>> getMap() { return map; }

        public T interpolate(Double key) {
            var l = map.lowerEntry(key+0.000001); // uber jank
            var h = map.higherEntry(key);
            if (l == null) {
                l = map.firstEntry();
                h = map.higherEntry(l.getKey());
            }
            else if (h == null) {
                l = map.lowerEntry(l.getKey());
                h = map.lastEntry();
            }

            Double t = (key - l.getKey()) / (h.getKey() - l.getKey());
            // System.out.println(t+" "+l.getKey()+" "+ h.getKey());
            // System.out.println(l.getValue().interpolate(h.getValue().get(), t));
            // return (t * (h.getValue() - l.getValue())) + l.getValue();
            return l.getValue().interpolate(h.getValue().get(), t);
        }   
    }

    public T interpolate(T b, Double t);
    public Double inverseInterpolate(T b, T query);
    public T get();

    public static double lerp(double a, double b, double t) {
        return t * (b - a) + a;
    }

    public static double inverseLerp(double a, double b, double c) {
        return (c - a) / (b - a);
    }
}
