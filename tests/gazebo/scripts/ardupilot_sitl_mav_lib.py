#!/usr/bin/env python3
"""
FALCON V2 - safe MAVLink helper for the ARDUPLANE_SITL_TRANSPORT_AND_
ACTUATOR_MAPPING_VALIDATION live test suite (gazebo-testing, 2026-08-27).

WHY THIS EXISTS (do not "simplify" back to bare pymavlink.mavutil calls):
pymavlink's mavutil.mavtcp.recv() calls self.handle_eof() -> self.reconnect()
on ANY zero-length TCP read. With autoreconnect=False, reconnect() is a
no-op (does NOT close/reopen the socket) - so if the remote end ever closes
the connection (observed live this task: ArduPlane's own SITL scheduler can
be timing-sensitive around the very first FDM JSON packet exchange when
launched WITHOUT a debugger - see this task's test report sec 2 for the
full live evidence trail), every subsequent recv_match()/wait_heartbeat()
call spins in a tight, unbounded, no-sleep loop calling port.recv() and
printing "EOF on TCP socket" as fast as the CPU allows (measured this task:
~300k iterations/sec, tens of megabytes of output per second, no bound
until the caller's own wall-clock timeout elapses - by which point it has
usually produced 10s of MB of output for nothing). This helper NEVER calls
recv_match()/wait_heartbeat() without first confirming (via select.select()
on the raw socket) that data is actually pending, so a dead connection
degrades to "no messages received" (a clean, reportable, bounded outcome)
instead of a runaway spin.

No aircraft physics parameter is read, written, or persisted anywhere in
this file.
"""
import select
import time

from pymavlink import mavutil


class SafeMav:
    def __init__(self, device="tcp:127.0.0.1:5760", source_system=255):
        # source_system=255 matches this SITL's MAV_GCS_SYSID=255 (confirmed
        # live via full param dump, this task) - REQUIRED for
        # RC_CHANNELS_OVERRIDE to be accepted (ArduPlane only accepts RC
        # override messages from the configured GCS system id; a mismatched
        # source_system is silently ignored, confirmed empirically this
        # task before this was discovered).
        self.m = mavutil.mavlink_connection(
            device, autoreconnect=False, source_system=source_system)

    def wait_heartbeat(self, timeout=15):
        t0 = time.time()
        while time.time() - t0 < timeout:
            r, _, _ = select.select([self.m.port], [], [], 0.5)
            if not r:
                continue
            msg = self.m.recv_match(type="HEARTBEAT", blocking=False)
            if msg:
                return msg
        return None

    def drain(self, duration, types=None):
        """Collect every message received in `duration` wall-clock seconds
        (or all messages if types is None), using select() to never call
        recv() on an idle/dead socket."""
        out = []
        t0 = time.time()
        while time.time() - t0 < duration:
            r, _, _ = select.select([self.m.port], [], [], 0.2)
            if not r:
                continue
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                continue
            if types is None or msg.get_type() in types:
                out.append(msg)
        return out

    def send_rc_override(self, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000):
        self.m.mav.rc_channels_override_send(
            self.m.target_system, self.m.target_component,
            int(rc1), int(rc2), int(rc3), int(rc4), int(rc5), 0, 0, 0)

    def hold_rc_override(self, duration, period=0.05, **kw):
        """Republish continuously - RC_OVERRIDE_TIME=3.0s on this SITL
        (confirmed via param dump) means a stale override reverts to trim
        after 3s of no fresh RC_CHANNELS_OVERRIDE traffic."""
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send_rc_override(**kw)
            time.sleep(period)

    def command_long(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                      wait_ack=True, timeout=4):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            command, 0, p1, p2, p3, p4, p5, p6, p7)
        if not wait_ack:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            r, _, _ = select.select([self.m.port], [], [], 0.3)
            if not r:
                continue
            msg = self.m.recv_match(type="COMMAND_ACK", blocking=False)
            if msg and msg.command == command:
                return msg
        return None

    def fetch_all_params(self, timeout=30, idle_cutoff=3.0):
        self.m.mav.param_request_list_send(self.m.target_system, self.m.target_component)
        params = {}
        t0 = time.time()
        last_recv = time.time()
        while time.time() - t0 < timeout:
            if time.time() - last_recv > idle_cutoff and len(params) > 10:
                break
            r, _, _ = select.select([self.m.port], [], [], 0.3)
            if not r:
                continue
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=False)
            if msg is None:
                continue
            last_recv = time.time()
            params[msg.param_id] = msg.param_value
        return params

    def close(self):
        self.m.close()
