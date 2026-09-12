package core;

/** Tracks how long an active path has continuously remained at a stuck speed. */
final class FollowerStuckWatchdog {
    private static final double TIMEOUT_SECONDS = 0.5;
    private long stuckSinceNanos = -1L;

    boolean update(boolean stuck, long nowNanos) {
        if (!stuck) {
            reset();
            return false;
        }
        if (stuckSinceNanos < 0L || nowNanos < stuckSinceNanos) {
            stuckSinceNanos = nowNanos;
            return false;
        }
        return nowNanos - stuckSinceNanos >= (long) (TIMEOUT_SECONDS * 1e9);
    }

    void reset() {
        stuckSinceNanos = -1L;
    }
}
