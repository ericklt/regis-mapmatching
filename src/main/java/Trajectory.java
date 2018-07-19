
import com.graphhopper.util.GPXEntry;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {

    private int id;
    private int taxiId;
    private List<GPXEntry> points = new ArrayList<>();

    public Trajectory(int id, int taxiId) {
        this.id = id;
        this.taxiId = taxiId;
    }

    public void addPoint(double lat, double lng, long timestamp) {
        points.add(new GPXEntry(lat, lng, timestamp));
    }

    public int getId() {
        return id;
    }

    public int getTaxiId() {
        return taxiId;
    }

    public List<GPXEntry> getPoints() {
        return points;
    }
}
