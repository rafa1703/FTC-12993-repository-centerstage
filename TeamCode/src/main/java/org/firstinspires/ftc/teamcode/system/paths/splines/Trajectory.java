package org.firstinspires.ftc.teamcode.system.paths.splines;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Trajectory
{
    private ArrayList<TrajectorySegment> segments;
    private int numberOfSegments;
    GVFLogic gvfLogic = new GVFLogic();
    ArrayList<Point> fullCurve;
    private double threshold = 0.5; // this is the default
    private boolean isFinished = false;
    private boolean usePID = false;
    private Pose finalPose;
    private final double SPATIAL_MARKER_THRESHOLD = 0.5; //in, this might be too big idk

    private ArrayList<SpatialMarker> spatialMarkers;

    public Trajectory(TrajectorySegment segment)
    {
        segments.add(segment);
    }
    public Trajectory(ArrayList<TrajectorySegment> segments)
    {
        this.segments = segments;

        init();
    }
    public Trajectory(ArrayList<TrajectorySegment> segments,@NonNull Pose finalPose)
    {
        this.segments = segments;
        this.finalPose = finalPose;
        init();
    }
    public Trajectory(ArrayList<TrajectorySegment> segments,@NonNull Pose finalPose, ArrayList<SpatialMarker> spatialMarkers)
    {
        this.segments = segments;
        this.finalPose = finalPose;
        this.spatialMarkers = spatialMarkers;
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
        int u = 0;
        for (TrajectorySegment segment : segments)
        {
            Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
            if (distAndT.y < closestDistance)
            {
                closestDistance = distAndT.y;
                t = distAndT.x;
                u = segments.indexOf(segment);
            }
        }

        BezierCurve curve = segments.get(u).returnCurve();
        boolean lastCurve = u == numberOfSegments;


        Vector powerVector = gvfLogic.calculate(curve, pose, lastCurve);
        // if we less then the threshold we can say we are finished
        if (segments.get(numberOfSegments).getEndPoint().subtract(pose.toPoint()).getMagnitude() < threshold)
        {
            isFinished = true;
        }

        if(!spatialMarkers.isEmpty()) // hope this doesn't break shit
        {
            for(SpatialMarker marker : spatialMarkers)
            {
                if(pose.getDistance(marker.spatialPoint) < SPATIAL_MARKER_THRESHOLD)
                {
                    marker.callback.onMarker();
                    spatialMarkers.remove(marker);
                }
            }
        }
        usePID = gvfLogic.usePID() && lastCurve && finalPose != null;

        return powerVector;
    }
    public Vector getTangentPowerVector(Pose pose, boolean reverse) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(reverse);
        gvfLogic.setFollowTangentially(true);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        int u = 0;
        for (TrajectorySegment segment : segments)
        {
            Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
            if (distAndT.y <= closestDistance)
            {
                closestDistance = distAndT.y;
                t = distAndT.x;
                u = segments.indexOf(segment);
            }
        }

        BezierCurve curve;
        boolean lastCurve = u == numberOfSegments;
        /*if (u == segments.size())
        {
             curve = segments.get(segments.size() -1).returnCurve();
        }
        else*/
        curve = segments.get(u).returnCurve();

        Vector powerVector = gvfLogic.calculate(curve, pose, lastCurve); //TODO i can probably already pass the t value here
        // if we less then the threshold we can say we are finished
        if (segments.get(numberOfSegments).getEndPoint().subtract(pose.toPoint()).getMagnitude() < threshold)
        {
            isFinished = true;
        }
        if(!spatialMarkers.isEmpty())
        {
            for(SpatialMarker marker : spatialMarkers)
            {
                if(pose.getDistance(marker.spatialPoint) < SPATIAL_MARKER_THRESHOLD)
                {
                    marker.callback.onMarker();
                    spatialMarkers.remove(marker);
                }
            }
        }
        usePID = gvfLogic.usePID() && lastCurve && finalPose != null;
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

    public boolean isFinished()
    {
        return isFinished;
    }

    public ArrayList<Point> getFullCurve()
    {
        return fullCurve;
    }

    public void setThreshold(double threshold)
    {
        this.threshold = threshold;
    }

    public boolean usePid()
    {
        return usePID;
    }

    public Pose getFinalPose()
    {
        return finalPose;
    }
}
