package org.firstinspires.ftc.teamcode.system.paths.splines;


import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class BelzierCurve
{
    private final double interval = 100;
    private final int n;
    private final Point[] points;
    private final double MAX_DISTANCE = 12; // in

    public BelzierCurve(Point[] points)
    {
        n = points.length - 1;
        this.points = points;
    }

    public Point parametric(double t)
    {
        if (!(t >= 0 && t <= 1))
        {
            throw new RuntimeException("Outside of domain");
        }

        double x = 0, y = 0;
        for(int r = 0; r <= n; r++)
        {
            x += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].x;
            y += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].y;
        }

        return new Point(x, y);
    }
    private double computeBinomial(double n, int r)
    {
        if (r == 0 || r == n) return 1;

        double a = 1;
        for(double v = 1; v <= r; v++)
        {
            a *= (n - v + 1) / v;
        }
        return a;
    }

    public ArrayList<Point> returnCurve()
    {
        ArrayList<Point> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            curve.add(parametric(t));
            t += 1.0 / interval;
        }
        curve.add(parametric(1));
        return curve;
    }
   /* public ArrayList<Point> returnCurveP()
    {
        ArrayList<Point> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            curve.add(parametric(t));
            t += 1.0 / interval;
        }
        curve.add(parametric(1));
        return curve;
    }*/

    private Point parametricDerived(double t)
    {
        if (!(t > 0 && t < 1))
        {
            if (t == 0) return parametricDerived(0.01);
            return new Point();
        }

        double x = 0, y = 0;
        for(int r = 0; r <= n; r++)
        {
            //x += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].x;
            x += computeBinomial(n, r) * points[r].x * Math.pow(1-t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);

            //y += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].y;
            y += computeBinomial(n, r) * points[r].y * Math.pow(1 - t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
        }

        return new Point(x, y);
    }
    /*    private Point parametricDerived2(double t)
        {
            if (!(t >= 0 && t <= 1))
            {
                throw new RuntimeException("Outside of domain");
            }

            double x = 0, y = 0;
            for(int r = 0; r <= n; r++)
            {
                //x += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].x;
                //x += computeBinomial(n, r) * points[r].x * Math.pow(1-t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
                x += computeBinomial(n, r) * (-(n - r) * Math.pow((1 - t), (n - r - 1)) * Math.pow(t, r)
                        + r * Math.pow(t, (r - 1)) * Math.pow((1 - t), (n - r))) * points[r].x;

                y += computeBinomial(n, r) * (-(n - r) * Math.pow((1 - t), (n - r - 1)) * Math.pow(t, r)
                        + r * Math.pow(t, (r - 1)) * Math.pow((1 - t), (n - r))) * points[r].y;
                //y += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].y;
                //y += computeBinomial(n, r) * points[r].y * Math.pow(1 - t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
            }

            return new Point(x, y);
        }*/
    public ArrayList<Point> returnDerivedCurve()
    {
        ArrayList<Point> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(parametricDerived(t));
            else curve.add(parametric(t + 0.1));
            t += 1.0 / interval;
        }
        curve.add(new Point(0, 0));
        return curve;
    }

    public Vector getTangentialVector(double t)
    {
        Point point = parametricDerived(t);
        return new Vector(point.x, point.y);
    }
    public double getTangentialHeading(double t)
    {
        Point point = parametricDerived(t);
        return Math.atan2(point.y, point.x);
    }

    /**Keep in mind result in rad **/
    public ArrayList<Double> returnTangentialHeadingList()
    {
        ArrayList<Double> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(getTangentialHeading(t));
            t += 1.0 / interval;
        }
        return curve;
    }
    public ArrayList<Vector> returnTangentialVectors()
    {
        ArrayList<Vector> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(getTangentialVector(t));
            t += 1.0 / interval;
        }
        return curve;
    }

    public Vector returnClosestPointOnCurve(Point point)
    {
        ArrayList<Point> curve = returnCurve();
        Vector closest = new Vector(curve.get(0).x, curve.get(0).y);
        double smallerDistance = Double.POSITIVE_INFINITY;
        for(Point p: curve)
        {
            double d = Math.abs(Math.sqrt(Math.pow(p.x - point.x, 2) + Math.pow(p.y - point.y, 2)));
            if (d <= smallerDistance)
            {
                closest = new Vector(p.x, p.y);
                smallerDistance = d;
            }
        }
        return closest;
    }
    public double returnClosestDistance(Point point)
    {
        ArrayList<Point> curve = returnCurve();
        double smallerDistance = Double.POSITIVE_INFINITY;
        for(Point p: curve)
        {
            double d = Math.abs(Math.sqrt(Math.pow(p.x - point.x, 2) + Math.pow(p.y - point.y, 2)));
            if (d <= smallerDistance)
            {
                smallerDistance = d;
            }
        }
        return smallerDistance;
    }
    /** This shit returns a point where x = t and y = dist **/
    public Point returnClosestDistanceAndT(Point point)
    {
        double closest = 0;
        double smallerDistance = Double.POSITIVE_INFINITY;
        for(int i = 0; i <= interval; i++)
        {
            Point p = parametric(i * (1/ interval));
            double d = Math.hypot(p.x - point.x, p.y - point.y) * Math.hypot(p.x - point.x, p.y - point.y);
            if (d <= smallerDistance)
            {
                closest = i * (1/interval);
                smallerDistance = d;
            }
        }
        return new Point(closest, smallerDistance); // x = t, y = dis
    }

    @Deprecated
    public Vector getCorrectedVector(Point point)
    {

        // point is the robot position in the xy plane

        // this is the points
        ArrayList<Point> curve = returnCurve();


        Point closest = curve.get(0);
        double smallestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        for(int i = 0; i <= curve.size()-1; i++) // this can probably be n
        {
            double d = Math.sqrt(Math.abs(Math.pow(curve.get(i).x - point.x, 2) + Math.pow(curve.get(i).y - point.y, 2)));
            if (d <= smallestDistance)
            {
                closest = curve.get(i);
                smallestDistance = d;
                t = i * (1 / interval);
            }
        }

        Vector correctionVector = new Vector(closest.x - point.x, closest.y - point.y);
        Vector pathVector;
        if (t == 0 || t == 1) pathVector = new Vector(0, 0);
            //if (t == 1) pathVector = new Vector(0, 0);
        else pathVector = getTangentialVector(t);
        pathVector.scaleBy(0.05);
        Vector resultantVector = correctionVector.plus(pathVector);

        return resultantVector;
    }


    public double returnClosesT(Point point)
    {
        double closest = 0;
        double smallerDistance = Double.POSITIVE_INFINITY;
        for(int i = 0; i <= interval; i++)
        {
            Point p = parametric(i * (1/ interval));
            double d = Math.hypot(p.x - point.x, p.y - point.y) * Math.hypot(p.x - point.x, p.y - point.y);
            if (d <= smallerDistance)
            {
                closest = i * (1/interval);
                smallerDistance = d;
            }
        }
        return closest;
    }



    public Vector getEndPoint()
    {
        return new Vector(points[n].x, points[n].y);
    }
    public Vector getStartPoint()
    {
        return new Vector(points[0].x, points[0].y);
    }

    public double getInterval()
    {
        return interval;
    }
}

