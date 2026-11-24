"""Small, bounded node heartbeats, independent of successful fusion output."""

import os
import time
import uuid

from company_dds import Topics


class NodeDiagnostics:
    def __init__(self, node, bus):
        self.node, self.bus = node, bus
        self.instance_id = uuid.uuid4().hex
        self.started_at = time.time()
        self.last_publish = 0.0
        self.values = {}

    def publish(self, force=False, **values):
        self.values.update(values)
        now = time.monotonic()
        if not force and now - self.last_publish < 1.0:
            return
        self.bus.write(Topics.DIAGNOSTICS.name, {
            "type": "diagnostics", "node": self.node,
            "instance_id": self.instance_id, "pid": os.getpid(),
            "started_at": self.started_at, "heartbeat_at": time.time(),
            **self.values,
        }, kind="diagnostics")
        self.last_publish = now
