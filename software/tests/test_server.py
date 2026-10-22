import json, os, sys, threading, time, urllib.request
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import server
from http.server import ThreadingHTTPServer

def post(port, t, s, estop=False, **extra):
    body = {"t": t, "s": s, "estop": estop, **extra}
    r = urllib.request.Request(f"http://127.0.0.1:{port}/ctl", json.dumps(body).encode(), method="POST")
    return json.loads(urllib.request.urlopen(r).read())

def start(motors):
    dolly = server.Dolly(motors)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(dolly))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return dolly, srv, srv.server_address[1]

def test_end_to_end():
    dolly, srv, port = start(server.StubMotors())
    assert b"C.A.R.L. Controller" in urllib.request.urlopen(f"http://127.0.0.1:{port}/").read()
    assert b"CARL" in urllib.request.urlopen(f"http://127.0.0.1:{port}/manifest.json").read()
    assert b"<svg" in urllib.request.urlopen(f"http://127.0.0.1:{port}/icon.svg").read()
    assert post(port, 1, 0)["speed"] == 1.0
    assert post(port, 1, 0, estop=True)["speed"] == 0.0
    post(port, 1, 0)
    time.sleep(0.5)                      # phone went silent
    st = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/status").read())
    assert st["speed"] == 0.0            # deadman stopped it
    srv.shutdown()

def test_gimbal_moves_holds_and_recentres():
    m = server.StubMotors()
    dolly, srv, port = start(m)
    for _ in range(8):                   # keep the heartbeat alive, camera stick full right
        post(port, 0, 0, p=1.0, l=0.0)
        time.sleep(0.05)
    st = post(port, 0, 0, p=0.0, l=0.0)
    assert st["pan"] > 5.0 and st["tilt"] == 0.0
    held = st["pan"]
    time.sleep(0.5)                      # phone silent: gimbal holds, never runs away
    assert abs(json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/status").read())["pan"] - held) < 1e-6
    assert m.last[4] == held             # the same angle reached the motor backend
    post(port, 0, 0, center=True)
    time.sleep(0.1)
    assert json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/status").read())["pan"] == 0.0
    srv.shutdown()

class Exploding(server.StubMotors):
    def apply(self, *a, **k):
        raise RuntimeError("bus fault")

def test_control_loop_survives_motor_errors():
    dolly, srv, port = start(Exploding())
    st = post(port, 1, 0)
    assert st["speed"] == 0.0 and "bus fault" in st["error"]       # stopped, error visible on the phone
    time.sleep(0.1)
    assert dolly.error and threading.active_count() >= 2            # loop thread still alive
    srv.shutdown()

def test_status_reports_link():
    class Down(server.StubMotors):
        link = False
    dolly, srv, port = start(Down())
    assert post(port, 0, 0)["link"] is False
    srv.shutdown()

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print("ok", name)
