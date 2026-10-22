"""Phone controller server for C.A.R.L. (Camera-Assisting Robotic Locomotive). Standard library only.

Run on the UNO Q Linux side:  python3 server.py
Phone joins the UNO Q hotspot (or shared Wi-Fi) and opens http://<ip>:8000
Phone sends joystick values 10x/second. If they stop for 0.3 s the dolly stops.
"""
import json, os, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import dolly_core as d

HERE = os.path.dirname(os.path.abspath(__file__))
ICON = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512"><rect width="512" height="512" rx="112" fill="#10151a"/>'
        '<rect x="96" y="212" width="320" height="110" rx="26" fill="#4cc2ff"/><circle cx="170" cy="340" r="42" fill="#e8eef2"/>'
        '<circle cx="342" cy="340" r="42" fill="#e8eef2"/><rect x="236" y="150" width="104" height="66" rx="14" fill="#e8eef2"/></svg>')


class StubMotors:
    """No hardware: used on a laptop and while the car is not wired."""
    link = True
    def apply(self, speed, steer_deg, pan=0.0, tilt=0.0):
        left, right = d.rear_wheel_speeds(speed, steer_deg)
        inner, outer = d.ackermann_angles(steer_deg)
        self.last = (left, right, inner, outer, pan, tilt)
    def read_tof_m(self): return None
    def read_batt_v(self): return None
    def stop(self): self.apply(0.0, 0.0)


def make_motors():
    """CARL_MOTORS=router|stub|auto (default auto: the real MCU link if the router socket exists)."""
    mode = os.environ.get("CARL_MOTORS", "auto")
    if mode == "auto":
        import mcu_link
        mode = "router" if os.path.exists(mcu_link.ROUTER_SOCK) else "stub"
    if mode == "router":
        import mcu_link
        return mcu_link.RouterMotors(require_tof=os.environ.get("CARL_NO_TOF") != "1")
    return StubMotors()


class Dolly:
    def __init__(self, motors):
        self.m = motors
        self.f = d.SafetyFilter()
        self.g = d.Gimbal()
        self.joy = (0.0, 0.0)
        self.gjoy = (0.0, 0.0)
        self.recenter = False
        self.cmd = d.Command(0.0, 0.0)
        self.lock = threading.Lock()
        self.error = None
        threading.Thread(target=self.loop, daemon=True).start()

    def set_input(self, t, s, estop, p=0.0, l=0.0, center=False):
        with self.lock:
            self.joy = (t, s)
            self.gjoy = (p, l)
            self.recenter = self.recenter or bool(center)
            self.f.estop = bool(estop)
            self.f.deadman.beat()

    def loop(self):
        last = time.monotonic()
        while True:
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                try:
                    self.f.tof_m = self.m.read_tof_m()
                    v = self.m.read_batt_v()
                    self.f.low_battery = v is not None and d.is_low_battery(v)
                    self.cmd = self.f.process(*self.joy)
                    if self.recenter:
                        self.g.recenter()
                        self.recenter = False
                    # gimbal holds still when the phone is silent, same deadman as the drive
                    self.g.update(*(self.gjoy if self.f.deadman.alive() else (0.0, 0.0)), dt)
                    self.m.apply(self.cmd.speed, self.cmd.steer_deg, self.g.pan, self.g.tilt)
                    self.error = None
                except Exception as e:           # never let the control thread die: stop and keep trying
                    self.error = repr(e)
                    self.cmd = d.Command(0.0, 0.0)
                    try:
                        self.m.stop()
                    except Exception:
                        pass
            time.sleep(0.02)

    def status(self):
        with self.lock:
            return {"speed": self.cmd.speed, "steer": self.cmd.steer_deg,
                    "tof": self.f.tof_m, "batt": self.m.read_batt_v(),
                    "low": self.f.low_battery, "estop": self.f.estop,
                    "pan": self.g.pan, "tilt": self.g.tilt,
                    "link": bool(getattr(self.m, "link", True)), "error": self.error}


def make_handler(dolly):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = open(os.path.join(HERE, "web", "index.html"), "rb").read()
                self.send_response(200); self.send_header("Content-Type", "text/html")
            elif self.path == "/manifest.json":
                body = json.dumps({"name": "C.A.R.L. Controller", "short_name": "CARL", "start_url": "/",
                    "display": "fullscreen", "orientation": "landscape", "background_color": "#10151a",
                    "theme_color": "#10151a", "icons": [{"src": "/icon.svg", "sizes": "any", "type": "image/svg+xml"}]}).encode()
                self.send_response(200); self.send_header("Content-Type", "application/manifest+json")
            elif self.path == "/icon.svg":
                body = ICON.encode()
                self.send_response(200); self.send_header("Content-Type", "image/svg+xml")
            elif self.path == "/status":
                body = json.dumps(dolly.status()).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json")
            else:
                self.send_response(404); body = b"not found"
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
        def do_POST(self):
            if self.path != "/ctl":
                self.send_response(404); self.end_headers(); return
            try:
                j = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                dolly.set_input(float(j["t"]), float(j["s"]), j.get("estop", False),
                                float(j.get("p", 0.0)), float(j.get("l", 0.0)), j.get("center", False))
            except Exception:
                self.send_response(400); self.end_headers(); return
            time.sleep(0.03)  # let one control tick run so the reply is fresh
            body = json.dumps(dolly.status()).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    return H


if __name__ == "__main__":
    motors = make_motors()
    dolly = Dolly(motors)
    print(f"C.A.R.L. controller on http://0.0.0.0:8000  (motors: {type(motors).__name__})", flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", 8000), make_handler(dolly)).serve_forever()
    finally:
        motors.stop()
