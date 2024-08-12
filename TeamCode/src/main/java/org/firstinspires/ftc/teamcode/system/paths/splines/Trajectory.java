package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Trajectory
{
    private ArrayList<TrajectorySegment> segments;
    private int numberOfSegments;
    private double u = 0;
    private double lastTFollowed;
    GVFLogic gvfLogic = new GVFLogic();
    public boolean pid;
    ArrayList<Point> fullCurve;

    public Trajectory(TrajectorySegment segment)
    {
        segments.add(segment);
    }
    public Trajectory(ArrayList<TrajectorySegment> segments)
    {
        this.segments = segments;
        init();
    }


    public void init()
    {
        numberOfSegments = segments.size() - 1;
        fullCurve = returnFullCurve();
    }

    public Vector getPowerVector(Pose pose) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(false);
        gvfLogic.setFollowTangentially(false);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        for (TrajectorySegment segment : segments)
        {
            Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
            if (distAndT.y < closestDistance)
            {
                closestDistance = distAndT.x;
                t = distAndT.x + segments.indexOf(segment);
            }
        }
        //u = t;
        BelzierCurve curve = segments.get((int) t).returnCurve();
        Vector powerVector = gvfLogic.calculate(curve, pose);

        return powerVector;
    }
    public Vector getTangentPowerVector(Pose pose, boolean reverse) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(reverse);
        gvfLogic.setFollowTangentially(true);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        for (TrajectorySegment segment : segments)
        {
            Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
            if (distAndT.x <= closestDistance)
            {
                closestDistance = distAndT.x;
                t = distAndT.y + segments.indexOf(segment);
            }
        }
        //u = t;
        BelzierCurve curve = segments.get((int) Math.floor(t)).returnCurve();
        Vector powerVector = gvfLogic.calculate(curve, pose);

        return powerVector;
    }
    private ArrayList<Point> returnFullCurve()
    {
        ArrayList<Point> wholeCurve = new ArrayList<>();
        for (TrajectorySegment segment : segments)
        {
            ArrayList<Point> curve = segment.returnCurve().returnCurve();
            wholeCurve.addAll(curve);
        }
        return wholeCurve;
    }

    public ArrayList<Point> getFullCurve()
    {
        return fullCurve;
    }


}
