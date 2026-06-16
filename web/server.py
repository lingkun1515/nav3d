#!/usr/bin/env python3
"""Dog3DNav Web 静态文件服务 + 文件浏览 API"""

import json
import os
import sys
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler


class APIHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"

        if path == "/api/list":
            self.handle_list(parsed)
        elif path == "/api/roots":
            self.handle_roots()
        elif path.startswith("/api/"):
            self.send_json({"error": f"unknown API endpoint: {path}"}, 404)
        else:
            # serve static files
            super().do_GET()

    def handle_roots(self):
        roots = [{"path": "/home", "label": "/home"}]
        candidates = [
            os.path.expanduser("~/Projects/NavProject/Dog3DNav/src/bringup/maps"),
            os.path.expanduser("~/Projects/NavProject/Dog3DNav/maps"),
        ]
        for d in candidates:
            if os.path.isdir(d) and d not in [r["path"] for r in roots]:
                roots.append({"path": d, "label": d})
        self.send_json(roots)
        print(f"[API] /api/roots -> {len(roots)} roots")

    def handle_list(self, parsed):
        qs = urllib.parse.parse_qs(parsed.query)
        raw = (qs.get("dir", [None])[0] or "").strip()
        if not raw:
            self.send_json({"error": "missing dir parameter"}, 400)
            return

        path = os.path.abspath(os.path.expanduser(raw))
        if not os.path.isdir(path):
            self.send_json({"error": f"not a directory: {path}"}, 404)
            return

        entries = []
        try:
            names = sorted(
                os.listdir(path),
                key=lambda n: (0 if os.path.isdir(os.path.join(path, n)) else 1, n.lower()),
            )
        except PermissionError:
            self.send_json({"error": "Permission denied"}, 403)
            return

        parent = os.path.dirname(path)
        entries.append({"name": "..", "type": "dir", "path": parent, "size": 0})

        for name in names:
            full = os.path.join(path, name)
            if name.startswith("."):
                continue  # skip hidden
            try:
                st = os.stat(full)
            except OSError:
                continue
            if os.path.isdir(full):
                entries.append({"name": name, "type": "dir", "path": full, "size": 0})
            else:
                ext = os.path.splitext(name)[1].lower()
                if ext in (".bt", ".pcd", ".ot", ".world", ".sdf"):
                    entries.append({
                        "name": name, "type": "file",
                        "path": full, "size": st.st_size, "ext": ext,
                    })

        self.send_json({"path": path, "entries": entries, "parent": parent})
        print(f"[API] /api/list dir={path} -> {len(entries)} entries")

    def send_json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        line = fmt % args
        if "/api/" in line:
            pass  # already printed in handler
        elif "200" in line or "304" in line:
            pass  # skip noise
        else:
            print(f"[HTTP] {line}")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    server = HTTPServer(("0.0.0.0", port), APIHandler)
    print(f"Dog3DNav Web Server @ http://0.0.0.0:{port}")
    print(f"  API: /api/roots  /api/list?dir=<path>")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.server_close()
