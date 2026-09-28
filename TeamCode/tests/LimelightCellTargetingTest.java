package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.json.JSONArray;
import org.json.JSONObject;
import java.net.InetAddress;
import static org.firstinspires.ftc.teamcode.LimelightCellTargeting.*;

/** Real SDK parser, synthetic decoded corners. No OpenCV or camera needed. */
public final class LimelightCellTargetingTest {
    private static int checks;
    static class Frame extends LLResult {
        long stale;
        Frame(double ts, JSONObject... tags) throws Exception {
            super(new JSONObject().put("v", 1).put("ts", ts).put("Fiducial", new JSONArray(tags)));
        }
        @Override public long getStaleness() { return stale; }
        @Override public double getParseLatency() { return 0; }
    }
    static LimelightCellTargeting tracker(Alliance alliance) throws Exception {
        return new LimelightCellTargeting(new Limelight3A(null, "test", InetAddress.getLoopbackAddress()),
                alliance);
    }
    static JSONObject tag(int id, double angle) {
        JSONArray points = new JSONArray();
        double a = Math.toRadians(angle);
        int first = id < 34 ? 30 : id < 38 ? 34 : id < 42 ? 38 : 42;
        double offset = (id-first)*60;
        for (double[] p : new double[][]{{-20,20},{20,20},{20,-20},{-20,-20}}) {
            points.put(new JSONArray().put(320 + Math.cos(a)*(p[0]+offset)-Math.sin(a)*p[1])
                    .put(240 + Math.sin(a)*(p[0]+offset)+Math.cos(a)*p[1]));
        }
        return new JSONObject().put("fID", id).put("pts", points);
    }
    static void expect(Observation o, State s, String label) {
        if (o.state != s) throw new AssertionError(label + ": " + o.state + "/" + o.reason);
        checks++;
    }
    static JSONObject[] pair(int id, double angle) { return new JSONObject[]{tag(id,angle),tag(id+1,angle)}; }
    public static void main(String[] args) throws Exception {
        LimelightCellTargeting red = tracker(Alliance.RED);
        expect(red.evaluate(new Frame(1,pair(34,0)),0),State.UNKNOWN,"first frame");
        expect(red.evaluate(new Frame(1,pair(34,0)),60),State.UNKNOWN,"duplicate");
        expect(red.evaluate(new Frame(2,pair(34,0)),70),State.UNKNOWN,"second frame");
        Observation o=red.evaluate(new Frame(3,pair(34,0)),120);
        expect(o,State.AUDIENCE_UP,"confirmed");
        if(!o.isUp(Side.AUDIENCE)||o.isUp(Side.SCORING_TABLE)) throw new AssertionError("side gate");
        expect(red.evaluate(new Frame(4,tag(34,0)),180),State.UNKNOWN,"single tag is unknown");
        expect(red.evaluate(new Frame(5,pair(34,180)),240),State.UNKNOWN,"inverted not inferred");
        expect(red.evaluate(new Frame(6,pair(34,90)),300),State.UNKNOWN,"vertical uncertain");
        expect(red.evaluate(new Frame(7,pair(38,0)),360),State.UNKNOWN,"wrong alliance");
        expect(red.evaluate(new Frame(8,tag(34,0),tag(35,0),tag(30,0),tag(31,0)),420),State.UNKNOWN,"both up conflict");
        expect(red.evaluate(new Frame(9,tag(34,0),tag(35,0),tag(36,180)),480),State.UNKNOWN,"pair disagreement");
        expect(red.evaluate(new Frame(10,tag(34,0),tag(34,0)),540),State.UNKNOWN,"duplicate id");
        expect(red.evaluate(new Frame(11,new JSONObject().put("fID",34)),600),State.UNKNOWN,"missing corners");
        for(int i=12;i<=14;i++) o=red.evaluate(new Frame(i,pair(34,0)),i*60);
        expect(o,State.AUDIENCE_UP,"recovery");
        expect(red.evaluate(null,850),State.UNKNOWN,"disconnect");
        expect(red.evaluate(new Frame(14,pair(34,0)),860),State.UNKNOWN,"old after disconnect");
        Frame old=new Frame(15,pair(34,0));old.stale=500;
        expect(red.evaluate(old,900),State.UNKNOWN,"SDK stale");
        for(int i=16;i<=18;i++) o=red.evaluate(new Frame(i,pair(34,0)),i*60);
        expect(o,State.AUDIENCE_UP,"fresh recovery");
        expect(red.evaluate(new Frame(1,pair(34,0)),1140),State.UNKNOWN,"reboot");
        expect(red.evaluate(new Frame(0,pair(34,0)),1150),State.UNKNOWN,"missing timestamp");
        for(int id:new int[]{30,34,38,42}) {
            LimelightCellTargeting tr=tracker(id<38?Alliance.RED:Alliance.BLUE);
            for(int i=1;i<=3;i++) o=tr.evaluate(new Frame(i,pair(id,20)),i*60);
            expect(o,(id==34||id==38)?State.AUDIENCE_UP:State.SCORING_TABLE_UP,"mapping");
        }
        red=tracker(Alliance.RED);
        for(int i=1;i<=3;i++) o=red.evaluate(new Frame(i,pair(34,0)),i);
        expect(o,State.UNKNOWN,"minimum time");
        expect(red.evaluate(new Frame(3,pair(34,0)),150),State.UNKNOWN,"duplicate time not confirmation");
        expect(red.evaluate(new Frame(4,pair(34,0)),160),State.AUDIENCE_UP,"new confirms");
        expect(red.evaluate(new Frame(4,pair(34,0)),300),State.AUDIENCE_UP,"fresh repeated");
        expect(red.evaluate(new Frame(4,pair(34,0)),370),State.UNKNOWN,"frozen expires");
        red=tracker(Alliance.RED);
        JSONObject[] arbitrary=pair(34,0);
        for(JSONObject t:arbitrary) {
            JSONArray c=t.getJSONArray("pts");
            t.put("pts",new JSONArray().put(c.get(2)).put(c.get(0)).put(c.get(3)).put(c.get(1)));
        }
        for(int i=1;i<=3;i++) o=red.evaluate(new Frame(i,arbitrary),i*60);
        expect(o,State.AUDIENCE_UP,"arbitrary corner order");
        expect(red.evaluate(new Frame(4,arbitrary),600),State.UNKNOWN,"long gap rebuilds");
        red=tracker(Alliance.RED);
        LimelightCellTargeting blue=tracker(Alliance.BLUE);
        Observation redState=null, blueState=null;
        for(int i=1;i<=3;i++) {
            Frame shared=new Frame(i,tag(34,0),tag(35,0),tag(42,0),tag(43,0));
            redState=red.evaluate(shared,i*60);
            blueState=blue.evaluate(shared,i*60);
        }
        expect(redState,State.AUDIENCE_UP,"shared view red");
        expect(blueState,State.SCORING_TABLE_UP,"shared view blue");
        Frame blueOnly=new Frame(4,pair(42,0));
        expect(red.evaluate(blueOnly,240),State.UNKNOWN,"pan away from red clears state");
        expect(blue.evaluate(blueOnly,240),State.SCORING_TABLE_UP,"blue remains visible");
        System.out.println("PASS: " + checks + " cell-state checks");
    }
}
