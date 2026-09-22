package com.camera.dualstream;
import java.util.*;
public class CaptureTimestampMatcherTest {
    static void check(boolean ok) { if(!ok) throw new AssertionError(); }
    public static void main(String[] args) {
        List<String> out=new ArrayList<>();
        CaptureTimestampMatcher m=new CaptureTimestampMatcher(2,
            (i,t,b) -> out.add(i+":"+t+":"+b[0]));
        // Image before result; another result arrives first with a very different offset.
        m.image(0,100,new byte[]{1},0);
        m.result(200,new Long[]{120L,200L},1);
        check(out.isEmpty());
        m.result(100,new Long[]{30L,100L},2);
        check(out.equals(Arrays.asList("0:30:1")));
        // Result before image, and no reuse on duplicate image.
        m.image(0,200,new byte[]{2},3);
        m.image(0,200,new byte[]{2},4);
        m.image(1,100,new byte[]{3},5);
        check(out.equals(Arrays.asList("0:30:1","0:120:2","1:100:3")));
        // Missing physical metadata must never inherit the previous offset.
        m.result(300,new Long[]{null,300L},6);
        m.image(0,300,new byte[]{4},7);
        check(out.size()==3 && m.dropped==1);
        // Missing result expires, and cannot emit with some other capture's metadata.
        m.image(0,400,new byte[]{5},8);
        m.result(500,new Long[]{400L,500L},1_000_000_010L);
        check(out.size()==3 && m.dropped==2);
        // No unbounded copied image accumulation if capture callbacks stop.
        for(int i=0;i<100;i++) m.image(0,1000+i,new byte[]{6},1_000_000_020L+i);
        check(m.size()<=24 && m.dropped>=78);
        m.clear(); check(m.size()==0);
        m.result(100,new Long[]{20L,100L},2_000_000_000L);
        check(out.size()==3); // reset cannot reuse an old image
        System.out.println("CaptureTimestampMatcher tests passed");
    }
}
