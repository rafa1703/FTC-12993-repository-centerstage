package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

public class BezierCurveTrajectorySegment extends TrajectorySegment
{
    BezierCurve curve;
    public BezierCurveTrajectorySegment(BezierCurve curve)
    {
        this.curve = curve;
    }


    @Override
    public Vector getTangentVector(double t)
    {
        return curve.getTangentialVector(t);
    }

    @Override
    public Vector getStartPoint()
    {
        return curve.getStartPoint();
    }

    @Override
    public Vector getEndPoint()
    {
        return curve.getEndPoint();
    }

    @Override
    public double getClosestT(Point point)
    {
        return curve.returnClosesT(point);
    }

    @Override
    public double getClosestDistance(Point point)
    {
        return curve.returnClosestDistance(point);
    }

    @Override
    public Point getClosestDistanceAndT(Point point)
    {
        return curve.returnClosestDistanceAndT(point);
    }

    @Override
    public BezierCurve returnCurve()
    {
        return curve;
    }
}
