"""Minimal HTTP + WebSocket (RFC 6455) server using only the standard library.

Serves static files from one directory and upgrades GET /ws to a WebSocket.
Every plain HTTP response closes the connection (HTTP/1.0 style), which keeps
the code small and is plenty for a dashboard with a handful of files.
"""

import asyncio
import base64
import hashlib
import mimetypes
import os
import struct
import urllib.parse

_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_MESSAGE = 1 << 20  # browser -> robot messages are tiny commands

mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("text/css", ".css")


def _unmask(data, mask):
    if not data:
        return data
    n = len(data)
    key = (mask * (n // 4 + 1))[:n]
    return (int.from_bytes(data, "big") ^ int.from_bytes(key, "big")).to_bytes(n, "big")


class WebSocket:
    def __init__(self, reader, writer, peer):
        self.reader = reader
        self.writer = writer
        self.peer = peer
        self.closed = False
        self._send_lock = asyncio.Lock()

    async def _send_frame(self, opcode, payload, force=False):
        if self.closed and not force:
            raise ConnectionError("websocket closed")
        n = len(payload)
        if n < 126:
            header = struct.pack("!BB", 0x80 | opcode, n)
        elif n < 65536:
            header = struct.pack("!BBH", 0x80 | opcode, 126, n)
        else:
            header = struct.pack("!BBQ", 0x80 | opcode, 127, n)
        async with self._send_lock:
            self.writer.write(header)
            self.writer.write(payload)
            await self.writer.drain()

    async def send_text(self, text):
        await self._send_frame(0x1, text.encode("utf-8"))

    async def send_bytes(self, data):
        await self._send_frame(0x2, data)

    async def receive(self):
        """Next text (str) or binary (bytes) message; None once closed."""
        buf = bytearray()
        msg_op = None
        try:
            while True:
                b1, b2 = await self.reader.readexactly(2)
                fin, op = b1 & 0x80, b1 & 0x0F
                masked, n = b2 & 0x80, b2 & 0x7F
                if n == 126:
                    n = struct.unpack("!H", await self.reader.readexactly(2))[0]
                elif n == 127:
                    n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
                if n + len(buf) > MAX_MESSAGE:
                    await self.close()
                    return None
                mask = await self.reader.readexactly(4) if masked else None
                payload = await self.reader.readexactly(n)
                if mask:
                    payload = _unmask(payload, mask)
                if op == 0x8:  # close
                    await self.close()
                    return None
                if op == 0x9:  # ping
                    await self._send_frame(0xA, payload)
                    continue
                if op == 0xA:  # pong
                    continue
                if op in (0x1, 0x2):
                    msg_op, buf = op, bytearray(payload)
                elif op == 0x0:
                    buf += payload
                if fin and msg_op is not None:
                    return buf.decode("utf-8") if msg_op == 0x1 else bytes(buf)
        except (asyncio.IncompleteReadError, ConnectionError, OSError, UnicodeDecodeError):
            self.closed = True
            return None

    async def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            await asyncio.wait_for(self._send_frame(0x8, b"", force=True), 1.0)
        except Exception:
            pass
        try:
            self.writer.close()
        except Exception:
            pass


class WebServer:
    def __init__(self, host, port, static_dir, ws_handler, log=print):
        self.host = host
        self.port = port
        self.static_dir = os.path.realpath(static_dir)
        self.ws_handler = ws_handler
        self.log = log
        self._server = None

    async def start(self):
        self._server = await asyncio.start_server(
            self._handle, self.host, self.port, reuse_address=True)

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    async def _handle(self, reader, writer):
        peer = writer.get_extra_info("peername")
        try:
            request = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 10.0)
        except (asyncio.TimeoutError, asyncio.IncompleteReadError,
                asyncio.LimitOverrunError, ConnectionError, OSError):
            writer.close()
            return
        lines = request.decode("latin-1").split("\r\n")
        try:
            method, target, _ = lines[0].split(" ", 2)
        except ValueError:
            await self._respond(writer, 400, b"bad request")
            return
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        path = urllib.parse.unquote(urllib.parse.urlsplit(target).path)

        if path == "/ws" and "websocket" in headers.get("upgrade", "").lower():
            key = headers.get("sec-websocket-key")
            if not key:
                await self._respond(writer, 400, b"missing Sec-WebSocket-Key")
                return
            accept = base64.b64encode(
                hashlib.sha1(key.encode() + _GUID).digest()).decode()
            writer.write((
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
            await writer.drain()
            ws = WebSocket(reader, writer, peer)
            try:
                await self.ws_handler(ws)
            except Exception as exc:  # never kill the server for one client
                self.log(f"websocket handler error: {exc!r}")
            finally:
                await ws.close()
            return

        if method not in ("GET", "HEAD"):
            await self._respond(writer, 405, b"method not allowed")
            return
        rel = path.lstrip("/") or "index.html"
        full = os.path.realpath(os.path.join(self.static_dir, rel))
        if not full.startswith(self.static_dir + os.sep) or not os.path.isfile(full):
            await self._respond(writer, 404, b"not found")
            return
        with open(full, "rb") as f:
            body = f.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if ctype.startswith("text/"):
            ctype += "; charset=utf-8"
        await self._respond(writer, 200, body, ctype, head_only=(method == "HEAD"))

    @staticmethod
    async def _respond(writer, code, body, ctype="text/plain; charset=utf-8",
                       head_only=False):
        reason = {200: "OK", 400: "Bad Request", 404: "Not Found",
                  405: "Method Not Allowed"}.get(code, "")
        writer.write((
            f"HTTP/1.1 {code} {reason}\r\n"
            f"Content-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
            "Cache-Control: no-cache\r\nConnection: close\r\n\r\n").encode())
        if not head_only:
            writer.write(body)
        try:
            await writer.drain()
        except (ConnectionError, OSError):
            pass
        writer.close()
