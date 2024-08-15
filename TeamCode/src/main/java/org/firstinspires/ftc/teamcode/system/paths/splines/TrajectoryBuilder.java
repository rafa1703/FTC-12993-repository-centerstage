package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;

import java.util.ArrayList;

public class TrajectoryBuilder
{
    private final ArrayList<TrajectorySegment> segments = new ArrayList<>();
    private Pose finalPose = null;
    private Pose startPose = null;
    private final ArrayList<SpatialMarker> spatialMarkers = new ArrayList<>();

    public TrajectoryBuilder(Pose startPose){
        this.startPose = startPose;
    }

    public TrajectoryBuilder addSegment(TrajectorySegment segment){
        segments.add(segment);
        return this;
    }
    public TrajectoryBuilder addFinalPose(Pose pose)
    {
        finalPose = pose;
        return this;
    }
    public TrajectoryBuilder addSpatialMarker(Pose markerPose, Callback callback)
    {
        spatialMarkers.add(new SpatialMarker(markerPose, callback));
        return this;
    }

    public Trajectory build(){
        return new Trajectory(segments, startPose, finalPose, spatialMarkers);
    }
    //public Trajectory end() {return new Trajectory(segments, finalPose);}
}
