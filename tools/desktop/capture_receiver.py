"""Tiny capture receiver — no Docker, no dependencies.

Receives session/project folders from the BetterSplats iPhone app (same
on-wire protocol as PhoneStreamer) and writes them under incoming/<name>/.

Protocol (length-prefixed, big-endian): [u32 len][u8 type][payload]
  type 20: file   — [u16 pathLen][relPath utf8][u32 size][bytes]
  type 21: done   — [u16 nameLen][packageName utf8]

Run:  py capture_receiver.py   (listens on 0.0.0.0:9999)
Phone connects to this PC's Tailscale IP or LAN IP, port 9999.

PhoneStreamer's capture-server.cmd speaks the same bytes, so either
receiver works.
"""
import os
import re
import socket
import struct
import sys
import zipfile

PORT = 9999
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "incoming")


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(min(n - len(buf), 1 << 16))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def safe_rel(path):
    path = path.replace("\\", "/")
    parts = [p for p in path.split("/") if p and p not in (".", "..")]
    return os.path.join(*parts) if parts else None


def handle(conn, addr):
    print(f"[+] {addr[0]} connected")
    session = "session"
    files = 0
    total = 0
    try:
        while True:
            struct.unpack(">I", recv_exact(conn, 4))
            mtype = recv_exact(conn, 1)[0]

            if mtype == 20:
                (plen,) = struct.unpack(">H", recv_exact(conn, 2))
                rel = safe_rel(recv_exact(conn, plen).decode("utf-8", "replace"))
                (size,) = struct.unpack(">I", recv_exact(conn, 4))
                remaining = size
                out_f = None
                if rel is not None:
                    session = rel.split(os.sep)[0]
                    dst = os.path.join(ROOT, rel)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    out_f = open(dst, "wb")
                try:
                    while remaining > 0:
                        chunk = conn.recv(min(remaining, 1 << 20))
                        if not chunk:
                            raise ConnectionError("closed mid-file")
                        if out_f:
                            out_f.write(chunk)
                        remaining -= len(chunk)
                finally:
                    if out_f:
                        out_f.close()
                files += 1
                total += size
                if files % 20 == 0 or size > 20e6:
                    print(f"    {files} files, {total / 1e6:.1f} MB...")

            elif mtype == 21:
                (nlen,) = struct.unpack(">H", recv_exact(conn, 2))
                name = recv_exact(conn, nlen).decode("utf-8", "replace")
                out = os.path.join(ROOT, re.sub(r"[^\w\-.]", "_", name))
                print(f"[✓] '{name}' complete: {files} files, {total / 1e6:.1f} MB")
                print(f"    → {out}")
                extract_zips(out)
                return
    except ConnectionError:
        if files:
            print(f"[!] connection ended early — kept {files} files ({total / 1e6:.1f} MB)")
        else:
            print("[-] connection closed (no data — check the phone's PC IP / port)")
    finally:
        conn.close()


def extract_zips(session_dir):
    if not os.path.isdir(session_dir):
        return
    for root, _, names in os.walk(session_dir):
        for n in names:
            if not n.lower().endswith(".zip"):
                continue
            zpath = os.path.join(root, n)
            dest = os.path.join(root, os.path.splitext(n)[0])
            try:
                with zipfile.ZipFile(zpath) as z:
                    count = 0
                    for m in z.infolist():
                        rel = safe_rel(m.filename)
                        if rel is None or m.is_dir():
                            continue
                        target = os.path.join(dest, rel)
                        os.makedirs(os.path.dirname(target), exist_ok=True)
                        with z.open(m) as src, open(target, "wb") as f:
                            f.write(src.read())
                        count += 1
                print(f"    unzipped {n} → {dest} ({count} files)")
            except Exception as e:
                print(f"    could not unzip {n}: {e}")


def main():
    os.makedirs(ROOT, exist_ok=True)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(2)
    print(f"Capture receiver listening on 0.0.0.0:{PORT}")
    print(f"Sessions land in: {ROOT}")
    print("Phone → home screen → PC IP (Tailscale or LAN), then Send to desktop.")
    while True:
        conn, addr = srv.accept()
        handle(conn, addr)
        print("ready for next send...\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
