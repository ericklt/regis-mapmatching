import com.bmwcarit.barefoot.matcher.Matcher;
import com.bmwcarit.barefoot.matcher.MatcherKState;
import com.bmwcarit.barefoot.roadmap.Loader;
import com.bmwcarit.barefoot.roadmap.RoadMap;
import com.bmwcarit.barefoot.roadmap.TimePriority;
import com.bmwcarit.barefoot.spatial.Geography;
import com.bmwcarit.barefoot.topology.Dijkstra;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

public class MapMatcher {

    public static List<MatcherKState> run(List<Trajectory> trajectories) {

        try {
            // Load and construct road map
            RoadMap map = Loader.roadmap(Main.class.getResource("fortaleza.osm.xml").getFile(), true).construct();

            // Instantiate matcher and state data structure
            Matcher matcher = new Matcher(map, new Dijkstra<>(), new TimePriority(), new Geography());

            return trajectories.stream().map(t -> matcher.mmatch(t.getSamples(), 1, 500)).collect(Collectors.toList());

        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;

    }

}
