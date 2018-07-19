import com.graphhopper.matching.MatchResult;

import java.util.List;
import java.util.Map;

public class Main {

    public static void main(String[] args) {

        TrajectoryReader reader = new TrajectoryReader("cfixed_taxi_hot_10_60min.csv");
        List<Trajectory> trajectories = reader.readTrajectories();

        Map<Long, MatchResult> results = MapMatcher.getInstance().run(trajectories);

        MapMatcher.getInstance().generateNodeTimestamps(results, "nodesTimestamps.csv");

    }

}
