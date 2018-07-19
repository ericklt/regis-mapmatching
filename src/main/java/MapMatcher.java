import com.graphhopper.GHRequest;
import com.graphhopper.PathWrapper;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.GPXExtension;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.shapes.GHPoint3D;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.Map.Entry;

public class MapMatcher {

    private static MapMatcher instance = new MapMatcher();

    private MyGraphHopper hopper;
    private MapMatching mapMatching;
    private NodeAccess nodeAccess;

    public static MapMatcher getInstance() {
        return instance;
    }

    private MapMatcher() {
        // import OpenStreetMap data
        hopper = new MyGraphHopper();
        hopper.setDataReaderFile(MapMatcher.class.getResource("fortaleza.osm.xml").getPath());
        hopper.setGraphHopperLocation("tmp/hopper/");
        hopper.setMinNetworkSize(200, 200);
        CarFlagEncoder encoder = new CarFlagEncoder();
        hopper.setEncodingManager(new EncodingManager(encoder));
        hopper.getCHFactoryDecorator().setEnabled(false);
        hopper.importOrLoad();

        nodeAccess = hopper.getGraphHopperStorage().getNodeAccess();

        String algorithm = Parameters.Algorithms.DIJKSTRA_BI;
        Weighting weighting = new FastestWeighting(encoder);
        AlgorithmOptions algoOptions = new AlgorithmOptions(algorithm, weighting);
        mapMatching = new MapMatching(hopper, algoOptions);
    }

    public Map<Long, MatchResult> run(List<Trajectory> trajectories) {
        Map<Long, MatchResult> map = new HashMap<>();
        for (Trajectory t : trajectories) {
            try {
                map.put((long) t.getId(), mapMatching.doWork(t.getPoints()));
            } catch (IllegalArgumentException e) {
                System.out.println("No matching for trajectory");
            }
        }
        return map;
    }

    private class RelevantPointIterator implements Iterator<GPXExtension> {

        private Iterator<EdgeMatch> it;
        private EdgeMatch current;
        private int currentEdgePosition = 0;
        private int nextEdgePosition = 0;
        private GPXExtension nextPoint;

        public RelevantPointIterator(List<EdgeMatch> matches) {
            it = matches.iterator();
            current = it.next();
            while(it.hasNext() && current.isEmpty()) {
                current = it.next();
                nextEdgePosition++;
            }
            nextPoint = current.isEmpty() ? null : current.getGpxExtensions().get(0);
        }

        public int getCurrentEdgePosition() {
            return currentEdgePosition;
        }

        @Override
        public boolean hasNext() {
            return nextPoint != null;
        }

        @Override
        public GPXExtension next() {
            GPXExtension next = nextPoint;
            currentEdgePosition = nextEdgePosition;

            List<GPXExtension> gpxExtensions = current.getGpxExtensions();
            if (nextPoint.equals(gpxExtensions.get(gpxExtensions.size()-1))) {
                if (!it.hasNext()) nextPoint = null;
                else {
                    current = it.next();
                    nextEdgePosition++;
                    while(it.hasNext() && current.isEmpty()) {
                        current = it.next();
                        nextEdgePosition++;
                    }
                    if (current.isEmpty()) nextPoint = null;
                    else nextPoint = gpxExtensions.get(0);
                }
            } else nextPoint = gpxExtensions.get(gpxExtensions.size()-1);

            return next;
        }
    }

    public void generateNodeTimestamps(Map<Long, MatchResult> mapMatchResult, String filePath) {
        System.out.println("Started generating timestamps for nodes");
        StringBuilder builder = new StringBuilder();
        builder.append("tid;nodeId;lat;lng;timestamp\n");

        for (Entry<Long, MatchResult> mapEntry : mapMatchResult.entrySet()) {
            List<EdgeMatch> matches = mapEntry.getValue().getEdgeMatches();
            long tid = mapEntry.getKey();
            int nodeToProcess = 0;
            RelevantPointIterator it = new RelevantPointIterator(matches);
            GPXExtension lastPoint = it.next(), current = null;
            while(it.hasNext()) {
                current = it.next();
                double currentLat = current.getQueryResult().getQueryPoint().getLat();
                double currentLng = current.getQueryResult().getQueryPoint().getLon();
                double speed = getSpeedMS(lastPoint, current);

                boolean hasProblems = false;
                while (nodeToProcess < it.getCurrentEdgePosition()) {
                    int nodeId = matches.get(nodeToProcess).getEdgeState().getBaseNode();
                    double lat = nodeAccess.getLatitude(nodeId);
                    double lng = nodeAccess.getLongitude(nodeId);
                    PathWrapper path = getPath(lat, lng, currentLat, currentLng);
                    if (path.hasErrors()) {
                        hasProblems = true;
                        System.err.println("(" + lat + ", " + lng + ") -> (" + current.getEntry().getLat() + ", " + current.getEntry().getLon() + ")");
                        break;
                    }
                    double distance = path.getDistance();
                    if (distance == 0) System.out.println(String.format("d: %.2f, v: %.2f", distance, speed));
                    long estimatedTimestamp = lastPoint.getEntry().getTime() - (long) (distance / speed);
                    builder.append(generateLine(tid, nodeId, lat, lng, estimatedTimestamp));
                    nodeToProcess++;
                }
                if (!hasProblems)
                    lastPoint = current;
            }

            if (current != null) {
                double currentLat = current.getQueryResult().getQueryPoint().getLat();
                double currentLng = current.getQueryResult().getQueryPoint().getLon();
                double speed = getSpeedMS(lastPoint, current);
                int nodeId = matches.get(matches.size()-1).getEdgeState().getAdjNode();
                double lat = nodeAccess.getLatitude(nodeId);
                double lng = nodeAccess.getLongitude(nodeId);
                PathWrapper path = getPath(currentLat, currentLng, lat, lng);
                if (!path.hasErrors()) {
                    double distance = path.getDistance();
                    if (distance == 0) System.out.println(String.format("d: %.2f, v: %.2f", distance, speed));
                    long estimatedTimestamp = lastPoint.getEntry().getTime() + (long) (distance / speed);
                    builder.append(generateLine(tid, nodeId, lat, lng, estimatedTimestamp));
                } else
                    System.err.println("lat: " + current.getEntry().getLat() + "lng: " + current.getEntry().getLon() + " -> " + "lat: " + lat + "lng: " + lng);
            }

        }

        saveOnFile(filePath, builder);
    }

    /**
     * Save map-matching result in format of id;latitude;longitude;order
     *
     * @param mapMatchResult
     * @param filePath
     */
    public void saveMapMatchingEdges(Map<Long, MatchResult> mapMatchResult, String filePath) {
        StringBuilder builder = new StringBuilder();

        builder.append("id;latitude;longitude;order\n");

        for (Entry<Long, MatchResult> mapEntry : mapMatchResult.entrySet()) {
            int order = 1;
            for (EdgeMatch edgeMatch : mapEntry.getValue().getEdgeMatches()) {
                int nodeId = edgeMatch.getEdgeState().getBaseNode();
                builder.append(generateLine(mapEntry.getKey(), nodeAccess.getLatitude(nodeId), nodeAccess.getLongitude(nodeId), order++));
            }
        }

        saveOnFile(filePath, builder);
    }

    public void saveMapMatchingSegmentsWithoutEmptyEdges(Map<Long, MatchResult> mapMatchResult, String filePath) {

        StringBuilder builder = new StringBuilder();
        EdgeMatch previousEdge = null;
        double speed;

        builder.append("trajId;ledgeId;timestamp;speed\n");

        // iterate over all trajectories
        for (Entry<Long, MatchResult> mapEntry : mapMatchResult.entrySet()) {
            GPXExtension previousGPS = null;
            Long trajId = mapEntry.getKey();// output traj_id

            // for each edge in trajectory map-matching
            for (EdgeMatch edgeMatch : mapEntry.getValue().getEdgeMatches()) {

                int edgeId = edgeMatch.getEdgeState().getEdge(); // output edge_id

                if(edgeMatch.isEmpty()) {
                    previousGPS = null;
                }
                // iterate over points on the edge
                for (GPXExtension gpsExtension : edgeMatch.getGpxExtensions()) {
                    if (previousGPS == null) {
                        previousGPS = gpsExtension;
                    } else if (previousEdge.equals(edgeMatch) || previousEdge.getEdgeState().getAdjNode() == edgeMatch.getEdgeState().getBaseNode()) {
                        speed = getSpeedMS(previousGPS, gpsExtension);
                        builder.append(generateLine(trajId, edgeId, gpsExtension.getEntry().getTime(), speed));
                    }
                    previousGPS = gpsExtension;
                    previousEdge = edgeMatch;
                }
            }
        }

        saveOnFile(filePath, builder);
    }

    //TODO refactoring
    public void saveMapMatchingSegmentsWithEmpyEdges(Map<Long, MatchResult> mapMatchResult, String filePath) throws IOException {
        FileWriter writer = new FileWriter(new File(filePath));
        StringBuilder builder = new StringBuilder();
        List<EdgeMatch> edgesWithoutPoints = new ArrayList<>();
        EdgeMatch previousEdge = null;
        long timestamp, timestamp2;
        double speed;

        try {
            builder.append("id;latitude;longitude;timestamp;speed;edge_id;osm_id\n");

            // iterate over all trajectories
            for (Entry<Long, MatchResult> mapEntry : mapMatchResult.entrySet()) {
                GPXExtension previousGPS = null;
                Long trajId = mapEntry.getKey();// output traj_id
                mapEntry.getValue().getMatchLength();
                double totalDelta = 0;
                // for each edge in trajectory map-matching
                for (EdgeMatch edgeMatch : mapEntry.getValue().getEdgeMatches()) {
                    int edgeId = edgeMatch.getEdgeState().getEdge();// output edge_id

                    if (edgeMatch.getGpxExtensions().isEmpty())
                        edgesWithoutPoints.add(edgeMatch);
                    // iterate over points on the edge
                    for (GPXExtension gpsExtension : edgeMatch.getGpxExtensions()) {
                        if (previousGPS != null) {
                            if (previousEdge == edgeMatch) {
                                speed = getSpeedMS(previousGPS, gpsExtension);
                                builder.append(generateLine(trajId, edgeId, previousGPS.getEntry().getTime(),
                                        gpsExtension.getEntry().getTime(), speed));

                            } else {

                                // line of previous point
                                speed = getSpeedMS(previousGPS, gpsExtension);

                                // first segment
                                timestamp = previousGPS.getEntry().getTime();
                                int node = previousEdge.getEdgeState().getAdjNode();
                                nodeAccess.getLatitude(node);
                                nodeAccess.getLongitude(node);

                                double distance = getPath(previousGPS.getEntry().getLat(), previousGPS.getEntry().getLon(),
                                        nodeAccess.getLatitude(node), nodeAccess.getLongitude(node))
                                        .getDistance();
                                double deltaTime = distance / speed;// time in milliseconds

                                timestamp2 = timestamp + (long) deltaTime;
                                generateLine(trajId, edgeId, timestamp, timestamp2, speed);
                                totalDelta += deltaTime;

                                // empty edges
                                if (!edgesWithoutPoints.isEmpty()) {
                                    for (EdgeMatch empty : edgesWithoutPoints) {
                                        distance = getPath(previousGPS.getEntry().getLat(),
                                                previousGPS.getEntry().getLon(),
                                                nodeAccess.getLatitude(empty.getEdgeState().getAdjNode()),
                                                nodeAccess.getLongitude(empty.getEdgeState().getAdjNode()))
                                                .getDistance();
                                        deltaTime = distance / speed;// time in milliseconds

                                        builder.append(generateLine(trajId, edgeId, timestamp2, timestamp + deltaTime, speed));
                                        timestamp2 = timestamp + (long) deltaTime;
                                        totalDelta += deltaTime;
                                    }
                                }

                                // last segment
                                node = edgeMatch.getEdgeState().getBaseNode();
                                nodeAccess.getLatitude(node);
                                nodeAccess.getLongitude(node);

                                distance = getPath(
                                        hopper.getGraphHopperStorage().getNodeAccess().getLatitude(node),
                                        hopper.getGraphHopperStorage().getNodeAccess().getLongitude(node),
                                        gpsExtension.getEntry().getLat(), gpsExtension.getEntry().getLon())
                                        .getDistance();
                                deltaTime = distance / speed;// time in milliseconds

                                totalDelta += (long) deltaTime;
                                builder.append(generateLine(trajId, edgeId, timestamp2, timestamp + deltaTime, speed));

                            }

                            edgesWithoutPoints.clear();
                        }

                        previousGPS = gpsExtension;
                        previousEdge = edgeMatch;
                    }
                }
            }

            writer.write(builder.toString());
            writer.flush();
        } finally {
            writer.close();
        }
    }

    private void saveOnFile(String filePath, StringBuilder builder) {
        try {
            FileWriter writer = new FileWriter(new File(filePath));
            writer.write(builder.toString());
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private String generateLine(Object ...objects) {
        if (objects.length == 0) return "\n";
        return Arrays.stream(objects).map(String::valueOf).reduce((s1, s2) -> s1 + ";" + s2).get() + "\n";
    }

    private double getSpeedMS(GPXExtension from, GPXExtension to) {
        PathWrapper res = getPath(from, to);
        return res.getDistance() / (to.getEntry().getTime() - from.getEntry().getTime())*1000;
    }

    private PathWrapper getPath(GPXExtension from, GPXExtension to) {
        GHPoint3D fromP = from.getQueryResult().getSnappedPoint();
        GHPoint3D toP = to.getQueryResult().getSnappedPoint();
        return getPath(fromP.getLat(), fromP.getLon(), toP.getLat(), toP.getLon());
    }

    private PathWrapper getPath(double latFrom, double lonFrom, double latTo, double lonTo) {
        GHRequest req = new GHRequest(latFrom, lonFrom, latTo, lonTo);
        req.setVehicle("car").setAlgorithm("dijkstra");
        return hopper.route(req).getBest();
    }

    private List<PathWrapper> getPathAll(double latFrom, double lonFrom, double latTo, double lonTo) {
        GHRequest req = new GHRequest(latFrom, lonFrom, latTo, lonTo);
        req.setVehicle("car").setAlgorithm("dijkstra");
        return hopper.route(req).getAll();
    }


}
