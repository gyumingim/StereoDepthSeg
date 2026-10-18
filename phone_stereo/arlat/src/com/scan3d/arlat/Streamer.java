package com.scan3d.arlat;

import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Sends JSON lines to the PC bench server (scan3d/tools/vio_bench_server.py) at 127.0.0.1:8798 — reached through
 * `adb reverse tcp:8798 tcp:8798`. Reconnects every second; drops lines when the queue is full (never blocks the caller).
 */
class Streamer {
    static final int PORT = 8798;
    private final LinkedBlockingQueue<String> q = new LinkedBlockingQueue<>(4000);
    private volatile boolean running = true;
    volatile boolean connected;
    volatile long dropped;

    Streamer() {
        Thread t = new Thread(this::loop, "streamer");
        t.setDaemon(true);
        t.start();
    }

    void send(String line) {
        if (!q.offer(line)) dropped++;
    }

    void close() { running = false; }

    private void loop() {
        while (running) {
            try (Socket s = new Socket()) {
                s.connect(new InetSocketAddress("127.0.0.1", PORT), 1000);
                s.setTcpNoDelay(true);
                connected = true;
                OutputStream o = s.getOutputStream();
                while (running) {
                    String l = q.poll(200, TimeUnit.MILLISECONDS);
                    if (l == null) continue;
                    StringBuilder b = new StringBuilder(l).append('\n');
                    String more;
                    while (b.length() < 60000 && (more = q.poll()) != null) b.append(more).append('\n');
                    o.write(b.toString().getBytes(StandardCharsets.UTF_8));
                    o.flush();
                }
            } catch (Exception e) {
                connected = false;
                try { Thread.sleep(1000); } catch (InterruptedException ignored) { }
            }
        }
    }
}
