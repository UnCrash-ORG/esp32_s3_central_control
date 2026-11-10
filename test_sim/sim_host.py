#!/usr/bin/env python3
"""
sim_host.py —— 电脑端模拟 ESP32 Web 控制台

起两个服务（都在同一进程，不同线程）：
  HTTP  :8080  提供三个 HTML + mock JSON API（/api/pid, /api/battery 等）
  WS    :8080  通过 /ws 路径（WebServer 原生不支持 path 级 WS，
          这里用 aiohttp 起一个纯 WS server :8081，前端会自动回退到 :8081）

用法：
  cd /workspace
  python3 test_sim/sim_host.py

然后浏览器打开：
  http://localhost:8080/       ← 姿态图
  http://localhost:8080/pid   ← PID
  http://localhost:8080/tools ← 综合

注意：前端默认连当前 HTTP host 的 /ws，这里 aiohttp 的 WS 和 http.server 不是一个进程，
      所以模拟时会在 HTTP 响应里用 JS 替换 WS URL 为 ws://host:8081/ws。
      更简单的做法：直接让前端在本地模拟时改 host；这里给它自动替换。
"""

import json, math, os, sys, time, threading, http.server, urllib.parse
from pathlib import Path

HTML_DIR = Path(__file__).resolve().parent.parent / "html"
PORT_HTTP = 8080
PORT_WS   = 8081

# ---------- 模拟状态 ----------
STATE = {
    "pid": {
        "wheels": [
            {"kp":1.20,"ki":0.15,"kd":0.05,"out_min":-300.0,"out_max":300.0},
            {"kp":1.20,"ki":0.15,"kd":0.05,"out_min":-300.0,"out_max":300.0},
            {"kp":1.20,"ki":0.15,"kd":0.05,"out_min":-300.0,"out_max":300.0},
            {"kp":1.20,"ki":0.15,"kd":0.05,"out_min":-300.0,"out_max":300.0},
        ]
    },
    "bat_v": 11.8,
}

# ---------- HTTP 服务 ----------
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass  # 安静点

    def _send_html(self, name):
        p = HTML_DIR / name
        if not p.exists():
            self.send_response(404); self.end_headers(); return
        data = p.read_bytes()
        # 让前端知道 WS 跑在 :8081
        marker = b"connect();"
        inject = marker + b"""/* sim_host: override WS port */
(function(){
  const _orig = WebSocket;
  WebSocket = function(u, p){
    if (typeof u==='string' && u.indexOf('/ws')>=0 && u.indexOf('8081')<0) {
      u = u.replace(/\\/ws$/, ':8081/ws');
    }
    return new _orig(u, p);
  };
  for (const k in _orig) WebSocket[k] = _orig[k];
})();
"""
        data = data.replace(marker, inject)
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        p = u.path
        if p in ("/", "/index.html"):  self._send_html("index.html"); return
        if p == "/pid":                self._send_html("pid.html");   return
        if p == "/tools":              self._send_html("tools.html"); return
        if p == "/api/pid":            self._send_json(STATE["pid"]); return
        if p == "/api/battery":        self._send_json({"voltage":STATE["bat_v"]}); return
        self.send_response(404); self.end_headers()

    def _read_body(self):
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(n) if n else b""

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        p = u.path
        q = urllib.parse.parse_qs(u.query)

        if p == "/api/pid":
            wi = int(q.get("wheel",[0])[0])
            j = json.loads(self._read_body())
            w = STATE["pid"]["wheels"][wi]
            for k in ("kp","ki","kd","out_min","out_max"):
                if k in j: w[k] = float(j[k])
            self._send_json({"ok":True})
            return
        if p == "/api/pid/all":
            j = json.loads(self._read_body())
            if "wheels" in j:
                for i, w in enumerate(j["wheels"][:4]):
                    for k in ("kp","ki","kd","out_min","out_max"):
                        if k in w: STATE["pid"]["wheels"][i][k] = float(w[k])
            self._send_json({"ok":True})
            return
        if p == "/api/calibrate":
            print("[sim] /api/calibrate  ← 假装重新校准了 IMU")
            self._send_json({"ok":True})
            return
        if p == "/api/ota":
            print(f"[sim] /api/ota  stub — 收到 {int(self.headers.get('Content-Length',0))} 字节，假装升级成功")
            self._send_json({"ok":True,"note":"simulated OTA ok"})
            return
        self.send_response(404); self.end_headers()


def run_http():
    srv = http.server.ThreadingHTTPServer(("0.0.0.0", PORT_HTTP), Handler)
    print(f"[sim] HTTP →  http://localhost:{PORT_HTTP}/   /pid   /tools")
    srv.serve_forever()

# ---------- WebSocket 服务（aiohttp） ----------
def run_ws():
    import asyncio
    try: from aiohttp import web
    except ImportError:
        print("[sim] ⚠ 未安装 aiohttp，跳过 WS 模拟。前端无法接收实时数据。")
        print("      pip install aiohttp")
        return

    t0 = time.time()
    clients = set()

    async def ws_handler(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        clients.add(ws)
        print(f"[sim] WS +1  (total {len(clients)})")
        try:
            async for _ in ws: pass
        finally:
            clients.discard(ws)
            print(f"[sim] WS -1  (total {len(clients)})")
        return ws

    async def broadcaster():
        while True:
            await asyncio.sleep(0.1)   # 100ms
            t = time.time() - t0
            msg = {
                "type": "imu",
                "ts": int(t*1000),
                "acc": [
                    0.15*math.sin(t*0.7) + 0.02*(hash(t)%100-50)/100,
                    0.10*math.cos(t*0.5) + 0.02*(hash(t+1)%100-50)/100,
                    1.0
                ],
                "gyr": [
                    2.0*math.cos(t*0.6),
                    3.0*math.sin(t*0.4),
                    1.5*math.cos(t*0.2),
                ],
                "att": [
                    5.0*math.sin(t*0.3),
                    4.0*math.cos(t*0.35),
                    t*20.0 % 360,
                ],
                "bat": 11.8 + 0.3*math.sin(t*0.05),
            }
            dead = []
            for c in clients:
                try: await c.send_str(json.dumps(msg))
                except Exception: dead.append(c)
            for c in dead: clients.discard(c)

    async def main():
        app = web.Application()
        app.router.add_get("/ws", ws_handler)
        print(f"[sim] WS   →  ws://localhost:{PORT_WS}/ws")
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", PORT_WS)
        await site.start()
        await broadcaster()  # 挂在一个长期 task 上

    asyncio.run(main())

# ---------- 入口 ----------
def main():
    threading.Thread(target=run_http, daemon=True).start()
    run_ws()

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nbye")
