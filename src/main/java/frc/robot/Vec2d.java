package frc.robot;

//credit: this class is based on code from FRC 5818 (https://github.com/Team5818/DiffSwerve)

public class Vec2d {

    //Vector constants
    final static Vec2d FORWARD = new Vec2d(0, 1),
            BACKWARD = new Vec2d(0, -1),
            LEFT = new Vec2d(-1, 0),
            RIGHT = new Vec2d(1, 0),
            ZERO = new Vec2d(0, 0);

    private double x;
    private double y;
    
    public Vec2d(double x, double y) {
        this.x = x;
        this.y = y;
        this.fixFloatingPointErrors();
    }

    //makes a unit vector with a certain angle
    public Vec2d(Angle angle) {
        this.x = Math.cos(Math.toRadians(angle.convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN).getAngle()));
        this.y = Math.sin(Math.toRadians(angle.convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN).getAngle()));
        this.fixFloatingPointErrors();
    }

    public Vec2d(Vec2d other) {
        this.x = other.x;
        this.y = other.y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX (double x) { this.x = x; }

    public void setY (double y) { this.y = y; }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public void fixFloatingPointErrors() {
        if (Math.abs(this.x) < 1e-5) {
            this.x = 0;
        }
        if (Math.abs(this.y) < 1e-5) {
            this.y = 0;
        }
    }

    //returns Angle object
    public Angle getAngle() {
        double angRad = Math.atan2(y, x);
        return new Angle(Math.toDegrees(angRad), Angle.AngleType.NEG_180_TO_180_CARTESIAN);
    }

    //returns numerical value for angle in specified type
    public double getAngleDouble(Angle.AngleType type) {
        return this.getAngle().convertAngle(type).getAngle();
    }

    public Vec2d add(Vec2d other) {
        return new Vec2d(x + other.getX(), y + other.getY());
    }

    public Vec2d scale(double scale) {
        return new Vec2d(getX() * scale, getY() * scale);
    }

    public Vec2d getUnitVector() {
        return normalize(1);
    }

    //returns a Vec2d in the same direction with magnitude of "target"
    public Vec2d normalize(double target) {
        if (getMagnitude() == 0) return ZERO; //avoid dividing by zero
        return scale(target / getMagnitude());
    }

    //returns Vec2d rotated by ang degrees
    public Vec2d rotateBy(double ang, Angle.Direction direction) {
        double angRads;
        if (direction == Angle.Direction.COUNTER_CLOCKWISE) {
            angRads = Math.toRadians(ang); //default vector rotation direction is CCW
        } else {
            angRads = -1 * Math.toRadians(ang);
        }
        return new Vec2d(x * Math.cos(angRads) - y * Math.sin(angRads), x * Math.sin(angRads) + y * Math.cos(angRads));
    }

    //returns Vec2d with the same magnitude as this but at the same angle as an Angle object
    public Vec2d rotateTo (Angle ang) {
        return new Vec2d(ang).scale(this.getMagnitude());
    }

    //dot product
    public double dot(Vec2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    //returns Vec2d reflected into 1st quadrant
    public Vec2d abs() {
        return new Vec2d(Math.abs(x), Math.abs(y));
    }

    //flips the signs of both components
    public Vec2d reflect () {
        return new Vec2d(-x, -y);
    }

    //projection of current vector onto v
    public Vec2d projection (Vec2d v) {
        return v.scale(dot(v)/(Math.pow(v.getMagnitude(), 2))); // u dot v over mag(v)^2 times v
    }

    //normalizes a group of vectors so that they maintain the same relative magnitudes and ...
    // the vector of largest magnitude now has a magnitude equal to limit
    public static Vec2d[] batchNormalize(double limit, Vec2d... vecs) {
        double maxMag = 0;
        for (Vec2d v : vecs) {
            if (v.getMagnitude() > maxMag) {
                maxMag = v.getMagnitude();
            }
        }
        if (limit >= maxMag) {
            return vecs;
        }
        Vec2d[] normed = new Vec2d[vecs.length];
        for (int i = 0; i < vecs.length; i++) {
            normed[i] = vecs[i].scale(limit / maxMag);
        }
        return normed;
    }


    public Vec2d clone() {
        return new Vec2d(x,y);
    }

    @Override
    public String toString() {
        return String.format("(%s, %s)", x, y);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vec2d)) {
            return false;
        }
        Vec2d other = (Vec2d) obj;
        if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x)) {
            return false;
        }
        if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y)) {
            return false;
        }
        return true;
    }
}