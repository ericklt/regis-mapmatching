import com.bmwcarit.barefoot.matcher.MatcherCandidate;
import com.bmwcarit.barefoot.matcher.MatcherKState;

import java.util.List;

public class Main {

    public static void main(String[] args) {

        TrajectoryReader reader = new TrajectoryReader("cfixed_taxi_hot_10_60min.csv");
        List<Trajectory> trajectories = reader.readTrajectories();

        List<MatcherKState> matcherKStates = MapMatcher.run(trajectories);


        for (MatcherCandidate cand : matcherKStates.get(0).sequence()) {
            cand.point().edge().base().refid(); // OSM id
            cand.point().edge().base().id(); // road id
            cand.point().edge().heading(); // heading
            cand.point().geometry(); // GPS position (on the road)
            if (cand.transition() != null)
                cand.transition().route().geometry(); // path geometry from last matching candidate
        }

    }

}
