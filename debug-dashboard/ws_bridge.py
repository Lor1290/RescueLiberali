"""
WebSocket Bridge for Erebus Robot Controller
=============================================
Streams sensor data from your Webots robot to the debug dashboard via WebSocket.
Runs a WS server in a background thread — does not block the robot's main loop.

Usage in your robot controller:
    from ws_bridge import start_bridge, send_frame
    start_bridge()             # once at startup
    while robot.step(TIMESTEP) != -1:
        send_frame(robot)      # every timestep
"""

import json
import math
import struct
import threading
import asyncio
import base64
import websockets

_clients = set()
_loop = None
_cmd_callback = None
_last_game_request_time = -1.0
_last_game_state = None


def _finite(v, default=0.0):
    """Return a finite float, falling back to default for NaN/Inf."""
    try:
        f = float(v)
        return f if math.isfinite(f) else default
    except Exception:
        return default


def _sanitize_range_image(values, max_len):
    out = []
    for d in list(values[:max_len]):
        out.append(_finite(d, 0.0))
    return out


def set_command_callback(fn):
    """Register a callback(cmd: dict) to handle dashboard commands."""
    global _cmd_callback
    _cmd_callback = fn


async def _handler(ws):
    _clients.add(ws)
    try:
        async for msg in ws:
            try:
                cmd = json.loads(msg)
                print(f"[WS Bridge] Command: {cmd}")
                if _cmd_callback:
                    _cmd_callback(cmd)
            except json.JSONDecodeError:
                pass
    finally:
        _clients.discard(ws)


async def _serve():
    async with websockets.serve(_handler, "localhost", 8765):
        await asyncio.Future()


def start_bridge():
    """Start the WebSocket server in a background thread."""

    def run():
        global _loop
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
        _loop.run_until_complete(_serve())

    t = threading.Thread(target=run, daemon=True)
    t.start()
    print("[WS Bridge] Server started on ws://localhost:8765")


def _camera_to_base64_bmp(cam):
    """Convert a Webots Camera BGRA image to a base64 BMP string."""
    try:
        img_bytes = cam.getImage()
        if not img_bytes:
            return None
        w, h = cam.getWidth(), cam.getHeight()
        row_size = (w * 3 + 3) & ~3
        bmp_size = 54 + row_size * h
        bmp = bytearray(bmp_size)
        struct.pack_into("<2sIHHI", bmp, 0, b"BM", bmp_size, 0, 0, 54)
        struct.pack_into(
            "<IiiHHIIiiII", bmp, 14, 40, w, -h, 1, 24, 0, row_size * h, 2835, 2835, 0, 0
        )
        for y in range(h):
            for x in range(w):
                src = (y * w + x) * 4
                dst = 54 + y * row_size + x * 3
                bmp[dst] = img_bytes[src]
                bmp[dst + 1] = img_bytes[src + 1]
                bmp[dst + 2] = img_bytes[src + 2]
        return base64.b64encode(bytes(bmp)).decode()
    except Exception:
        return None


def send_frame(robot):
    """
    Collect sensor data and broadcast to connected dashboard clients.
    Devices: gps, lidar, inertial_unit, colour_sensor, camera1, camera2
    """
    if not _clients:
        return

    frame = {}

    # GPS
    try:
        gps = robot.getDevice("gps")
        if gps:
            v = gps.getValues()
            frame["gps"] = {
                "x": _finite(v[0]),
                "y": _finite(v[1]),
                "z": _finite(v[2]),
            }
    except Exception:
        pass

    # Inertial Unit → roll/pitch/yaw (sent as "gyro" for the dashboard)
    try:
        imu = robot.getDevice("inertial_unit")
        if imu:
            rpy = imu.getRollPitchYaw()
            frame["gyro"] = {
                "x": _finite(rpy[0]),
                "y": _finite(rpy[1]),
                "z": _finite(rpy[2]),
            }
    except Exception:
        pass

    # Colour sensor (ground)
    try:
        colour = robot.getDevice("colour_sensor")
        if colour:
            img = colour.getImage()
            if img:
                r = colour.imageGetRed(img, 1, 0, 0)
                g = colour.imageGetGreen(img, 1, 0, 0)
                b = colour.imageGetBlue(img, 1, 0, 0)
                frame["color_sensor"] = {"r": r, "g": g, "b": b}
    except Exception:
        pass

    # LIDAR
    try:
        lidar_dev = robot.getDevice("lidar")
        if lidar_dev:
            ri = lidar_dev.getRangeImage()
            if ri:
                resolution = lidar_dev.getHorizontalResolution()
                num_layers = lidar_dev.getNumberOfLayers()
                layer_data = _sanitize_range_image(ri, resolution)
                point_cloud = []
                if num_layers >= 2:
                    layer2 = ri[resolution : 2 * resolution]
                    fov = lidar_dev.getFov()
                    max_rng = lidar_dev.getMaxRange()
                    for i, dist in enumerate(layer2):
                        d = _finite(dist, max_rng)
                        if d < max_rng - 0.05:
                            angle = -fov / 2.0 + (fov * i / resolution) - math.pi / 2.0
                            point_cloud.append(
                                {
                                    "x": _finite(d * math.cos(angle)),
                                    "y": 0,
                                    "z": _finite(d * math.sin(angle)),
                                    "layer": 1,
                                }
                            )
                frame["lidar"] = {
                    "range_image": layer_data,
                    "horizontal_resolution": resolution,
                    "num_layers": num_layers,
                    "point_cloud": point_cloud,
                }
    except Exception:
        pass

    # Cameras — camera1 = right, camera2 = left
    try:
        cameras = {}
        for dev_name, label in [("camera1", "right"), ("camera2", "left")]:
            cam = robot.getDevice(dev_name)
            if cam:
                b64 = _camera_to_base64_bmp(cam)
                if b64:
                    cameras[label] = b64
        if cameras:
            frame["camera"] = cameras
    except Exception:
        pass

    # Game state from Erebus receiver queue
    global _last_game_request_time, _last_game_state
    try:
        emitter = robot.getDevice("emitter")
        receiver = robot.getDevice("receiver")
        now = robot.getTime()
        if emitter and now - _last_game_request_time > 1.0:
            emitter.send(struct.pack("c", b"G"))
            _last_game_request_time = now

        if receiver:
            while receiver.getQueueLength() > 0:
                packet = receiver.getBytes()
                if len(packet) == 16:
                    tup = struct.unpack("c f i i", packet)
                    if tup[0].decode("utf-8") == "G":
                        _last_game_state = {
                            "score": float(tup[1]),
                            "time_remaining": max(0, int(tup[2])),
                            "time_elapsed": max(0.0, now),
                            "victims_found": 0,
                        }
                receiver.nextPacket()

        if _last_game_state is not None:
            frame["game"] = _last_game_state
    except Exception:
        pass

    # Broadcast
    if frame and _loop:
        try:
            msg = json.dumps(frame, allow_nan=False)
        except ValueError:
            return
        asyncio.run_coroutine_threadsafe(_broadcast(msg), _loop)


async def _broadcast(msg):
    if _clients:
        await asyncio.gather(
            *[c.send(msg) for c in _clients.copy()], return_exceptions=True
        )
