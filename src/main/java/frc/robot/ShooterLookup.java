package frc.robot;

public final class ShooterLookup {

  public static final class Point {
    public final double distM;
    public final double rpm;
    public final double pitchRot;

    public Point(double distM, double rpm, double pitchRot) {
      this.distM = distM;
      this.rpm = rpm;
      this.pitchRot = pitchRot;
    }
  }

  private static final Point[] TABLE = new Point[] {
      new Point(1.409223242, 2200.0, -0.4462890625 + 0.2255859375),//2200
      new Point(2.082485830, 2480.0, -0.52294921875 + 0.2255859375),//2480
      new Point(2.424831302, 2600.0, -0.5255859375 + 0.2255859375),//2600
      new Point(3.320675450, 3050.0, -0.3435),//3050
      new Point(3.45675450, 3350.0, -0.3435)//3350
  };

  private ShooterLookup() {}

  public static Point sample(double distM) {

    if (!Double.isFinite(distM)) return TABLE[0];

    if (distM <= TABLE[0].distM) return TABLE[0];

    int last = TABLE.length - 1;
    if (distM >= TABLE[last].distM) return TABLE[last];

    for (int i = 0; i < last; i++) {
      Point a = TABLE[i];
      Point b = TABLE[i + 1];

      if (distM >= a.distM && distM <= b.distM) {
        double t = (distM - a.distM) / (b.distM - a.distM);
        double rpm = lerp(a.rpm, b.rpm, t);
        double pitch = lerp(a.pitchRot, b.pitchRot, t);
        return new Point(distM, rpm, pitch);
      }
    }

    return TABLE[last];
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}