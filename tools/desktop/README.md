# Send to desktop

The iPhone app ships session and project folders over TCP using the same
bytes as PhoneStreamer. This receiver writes them under `incoming/`.

```
py capture_receiver.py
```

Or double-click `capture-server.cmd`. Listens on `0.0.0.0:9999`.

On the phone: home screen → PC IP (Tailscale or LAN) and port 9999, then
**Send to desktop** on a session or a project.

PhoneStreamer's `capture-server.cmd` speaks the same protocol, so either
receiver works.
