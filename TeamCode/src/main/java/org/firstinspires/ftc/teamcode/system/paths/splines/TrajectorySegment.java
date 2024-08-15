package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

public abstract class TrajectorySegment
{
    // this will effectively just take a curve, and control the max velocity of the curve
    public double size = 100;
    public double maxSpeed = 1;

    public abstract Vector getTangentVector(double t);
    public abstract Vector getStartPoint();
    public abstract Vector getEndPoint();
    public abstract double getClosestT(Point point);
    public abstract double getClosestDistance(Point point);
    public abstract Point getClosestDistanceAndT(Point point);
    public abstract BezierCurve returnCurve();
    public abstract double getMaxSpeed();
    public abstract void setMaxSpeed(double speed);

}