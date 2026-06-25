package frc.lib.helpers;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Minimal geometry helpers for Pose2d-based checks.
 * Triangle/quad helpers delegate to the general winding-number test.
 */
public final class geometryUtils {
    private geometryUtils() {}

    /** Alias for inZone with 3 vertices. */
    public static boolean inTriangle(Pose2d point, Pose2d a, Pose2d b, Pose2d c) {
        return inZone(point, new Pose2d[] { a, b, c });
    }

    /** Alias for inZone with 4 vertices. */
    public static boolean inQuad(Pose2d point, Pose2d a, Pose2d b, Pose2d c, Pose2d d) {
        return inZone(point, new Pose2d[] { a, b, c, d });
    }

    /** Alias for inZone with 4 vertices. */
    public static boolean inQuadrilateral(Pose2d point, Pose2d a, Pose2d b, Pose2d c, Pose2d d) {
        return inZone(point, new Pose2d[] { a, b, c, d });
    }

    // 2D cross product (z component) for (b - a) x (c - a)
    private static double crossZ(Pose2d a, Pose2d b, Pose2d c) {
        double abx = b.getX() - a.getX();
        double aby = b.getY() - a.getY();
        double acx = c.getX() - a.getX();
        double acy = c.getY() - a.getY();
        return abx * acy - aby * acx;
    }

    // Check if point p lies on segment a-b (inclusive), with small tolerance.
    private static boolean isPointOnSegment(Pose2d p, Pose2d a, Pose2d b) {
        final double EPS = 1e-9;
        double cross = crossZ(a, b, p);
        if (Math.abs(cross) > EPS) {
            return false;
        }
        double pax = p.getX() - a.getX();
        double pay = p.getY() - a.getY();
        double bax = b.getX() - a.getX();
        double bay = b.getY() - a.getY();
        double dot = pax * bax + pay * bay;
        if (dot < -EPS) {
            return false;
        }
        double len2 = bax * bax + bay * bay;
        return dot <= len2 + EPS;
    }

    /**
     * Winding-number point-in-polygon test. Returns true if point is inside or on the edge
     * of the simple polygon given by verts (clockwise or counter-clockwise).
     */
    public static boolean inZone(Pose2d point, Pose2d[] verts) {
        final double EPS = 1e-9;
        int n = verts == null ? 0 : verts.length;
        if (n < 3) {
            return false;
        }

        // boundary check
        for (int i = 0; i < n; ++i) {
            Pose2d a = verts[i];
            Pose2d b = verts[(i + 1) % n];
            if (isPointOnSegment(point, a, b)) {
                return true;
            }
        }

        int wn = 0;
        double py = point.getY();

        for (int i = 0; i < n; ++i) {
            Pose2d vi = verts[i];
            Pose2d vj = verts[(i + 1) % n];
            double yi = vi.getY();
            double yj = vj.getY();

            if (yi <= py) {
                if (yj > py) { // upward crossing
                    if (crossZ(vi, vj, point) > EPS) {
                        wn++;
                    }
                }
            } else {
                if (yj <= py) { // downward crossing
                    if (crossZ(vi, vj, point) < -EPS) {
                        wn--;
                    }
                }
            }
        }
        return wn != 0;
    }
}