
import com.bmwcarit.barefoot.matcher.MatcherSample;
import com.esri.core.geometry.Point;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {

    private int id;
    private int taxiId;
    private List<MatcherSample> samples = new ArrayList<>();

    public Trajectory(int id, int taxiId) {
        this.id = id;
        this.taxiId = taxiId;
    }

    public void addSample(double lat, double lng, long timestamp) {
        samples.add(new MatcherSample(timestamp, new Point(lat, lng)));
    }

    public int getId() {
        return id;
    }

    public int getTaxiId() {
        return taxiId;
    }

    public List<MatcherSample> getSamples() {
        return samples;
    }
}
