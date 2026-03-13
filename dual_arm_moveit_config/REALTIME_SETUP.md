# Realtime Setup

`ros2_control_node` requests FIFO realtime scheduling for lower latency and more stable control timing. On a normal desktop shell this often fails with:

`Could not enable FIFO RT scheduling policy: Operation not permitted`

This is expected until the Linux user has realtime privileges. For real hardware, configure them once:

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
```

Create `/etc/security/limits.d/99-realtime.conf` with:

```conf
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

Then log out and log back in. Verify with:

```bash
ulimit -r
ulimit -l
id
```

`ulimit -r` should be greater than `0`, and your user should be in the `realtime` group.

This is an OS-level requirement, not a launch-file issue. The fake `mock_components` setup works without it, but topic-based real hardware control benefits from enabling it.
