package frc.robot.utils;

public class Vector2D {

  public double x;
  public double y;
  
  public Vector2D() { }

  public Vector2D(double x, double y) {
      this.x = x;
      this.y = y;
  }

  public Vector2D(Vector2D v) {
      set(v);
  }

  public Vector2D set(double x, double y) {
      this.x = x;
      this.y = y;
      return this;
  }

  public Vector2D set(Vector2D v) {
      this.x = v.x;
      this.y = v.y;
      return this;
  }

  public void setZero() {
      x = 0;
      y = 0;
  }

  public double[] getComponents() {
      return new double[]{x, y};
  }

  public double getMagnitude() {
      return Math.sqrt(x * x + y * y);
  }

  public Vector2D setMagnitude(double magnitude) {
    x = x / getMagnitude() * magnitude;
    y = y / getMagnitude() * magnitude;
    return this;
  }

  public double getMagnitudeSq() {
      return (x * x + y * y);
  }

  public double distanceSq(double vx, double vy) {
      vx -= x;
      vy -= y;
      return (vx * vx + vy * vy);
  }

  public double distanceSq(Vector2D v) {
      double vx = v.x - this.x;
      double vy = v.y - this.y;
      return (vx * vx + vy * vy);
  }

  public double distance(double vx, double vy) {
      vx -= x;
      vy -= y;
      return Math.sqrt(vx * vx + vy * vy);
  }

  public double distance(Vector2D v) {
      double vx = v.x - this.x;
      double vy = v.y - this.y;
      return Math.sqrt(vx * vx + vy * vy);
  }

  public double getAngle() {
      return Math.atan2(y, x);
  }

  public Vector2D normalize() {
      double magnitude = getMagnitude();
      x /= magnitude;
      y /= magnitude;
      return this;
  }

  public Vector2D getNormalized() {
      double magnitude = getMagnitude();
      return new Vector2D(x / magnitude, y / magnitude);
  }

  public static Vector2D toCartesian(double magnitude, double angle) {
      return new Vector2D(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
  }

  public Vector2D add(Vector2D v) {
      this.x += v.x;
      this.y += v.y;
      return this;
  }

  public Vector2D add(double vx, double vy) {
      this.x += vx;
      this.y += vy;
      return this;
  }

  public static Vector2D add(Vector2D v1, Vector2D v2) {
      return new Vector2D(v1.x + v2.x, v1.y + v2.y);
  }

  public Vector2D getAdded(Vector2D v) {
      return new Vector2D(this.x + v.x, this.y + v.y);
  }

  public Vector2D subtract(Vector2D v) {
      this.x -= v.x;
      this.y -= v.y;
      return this;
  }

  public Vector2D subtract(double vx, double vy) {
      this.x -= vx;
      this.y -= vy;
      return this;
  }

  public static Vector2D subtract(Vector2D v1, Vector2D v2) {
      return new Vector2D(v1.x - v2.x, v1.y - v2.y);
  }

  public Vector2D getSubtracted(Vector2D v) {
      return new Vector2D(this.x - v.x, this.y - v.y);
  }

  public Vector2D multiply(double scalar) {
      x *= scalar;
      y *= scalar;
      return this;
  }

  public Vector2D getMultiplied(double scalar) {
      return new Vector2D(x * scalar, y * scalar);
  }

  public Vector2D divide(double scalar) {
      x /= scalar;
      y /= scalar;
      return this;
  }

  public Vector2D getDivided(double scalar) {
      return new Vector2D(x / scalar, y / scalar);
  }

  public Vector2D getPerp() {
      return new Vector2D(-y, x);
  }

  public double dot(Vector2D v) {
      return (this.x * v.x + this.y * v.y);
  }

  public double dot(double vx, double vy) {
      return (this.x * vx + this.y * vy);
  }

  public static double dot(Vector2D v1, Vector2D v2) {
      return v1.x * v2.x + v1.y * v2.y;
  }

  public double cross(Vector2D v) {
      return (this.x * v.y - this.y * v.x);
  }

  public double cross(double vx, double vy) {
      return (this.x * vy - this.y * vx);
  }

  public static double cross(Vector2D v1, Vector2D v2) {
      return (v1.x * v2.y - v1.y * v2.x);
  }

  public double project(Vector2D v) {
      return (this.dot(v) / this.getMagnitude());
  }

  public double project(double vx, double vy) {
      return (this.dot(vx, vy) / this.getMagnitude());
  }

  public static double project(Vector2D v1, Vector2D v2) {
      return (dot(v1, v2) / v1.getMagnitude());
  }

  public Vector2D getProjectedVector(Vector2D v) {
      return this.getNormalized().getMultiplied(this.dot(v) / this.getMagnitude());
  }

  public Vector2D getProjectedVector(double vx, double vy) {
      return this.getNormalized().getMultiplied(this.dot(vx, vy) / this.getMagnitude());
  }

  public static Vector2D getProjectedVector(Vector2D v1, Vector2D v2) {
      return v1.getNormalized().getMultiplied(Vector2D.dot(v1, v2) / v1.getMagnitude());
  }

  public Vector2D rotateBy(double angle) {
      double cos = Math.cos(angle);
      double sin = Math.sin(angle);
      double rx = x * cos - y * sin;
      y = x * sin + y * cos;
      x = rx;
      return this;
  }

  public Vector2D getRotatedBy(double angle) {
      double cos = Math.cos(angle);
      double sin = Math.sin(angle);
      return new Vector2D(x * cos - y * sin, x * sin + y * cos);
  }

  public Vector2D rotateTo(double angle) {
      set(toCartesian(getMagnitude(), angle));
      return this;
  }

  public Vector2D getRotatedTo(double angle) {
      return toCartesian(getMagnitude(), angle);
  }

  public Vector2D reverse() {
      x = -x;
      y = -y;
      return this;
  }

  public Vector2D getReversed() {
      return new Vector2D(-x, -y);
  }

  @Override
  public Vector2D clone() {
      return new Vector2D(x, y);
  }

  @Override
  public boolean equals(Object obj) {
      if (obj == this) {
          return true;
      }
      if (obj instanceof Vector2D) {
          Vector2D v = (Vector2D) obj;
          return (x == v.x) && (y == v.y);
      }
      return false;
  }

  @Override
  public String toString() {
      return "Vector2d[" + x + ", " + y + "]";
  }
}