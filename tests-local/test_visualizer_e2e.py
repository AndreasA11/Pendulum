#!/usr/bin/env python3
import urllib.request
import urllib.parse
import subprocess
import threading
import socket
import select
import struct
import base64
import hashlib
import time
import json
import sys
import os

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

def make_masked_ws_frame(message: str) -> bytes:
    payload = bytearray(message.encode('utf-8'))
    mask = b'\x1a\x2b\x3c\x4d'
    for i in range(len(payload)):
        payload[i] ^= mask[i % 4]
    
    length = len(payload)
    if length <= 125:
        header = bytes([0x81, 0x80 | length]) + mask
    elif length <= 65535:
        header = struct.pack('!BBH', 0x81, 0x80 | 126, length) + mask
    else:
        header = struct.pack('!BBQ', 0x81, 0x80 | 127, length) + mask
    return header + payload

def recv_ws_frame_simple(sock):
    header = sock.recv(2)
    if not header or len(header) < 2: return None
    b1, b2 = header[0], header[1]
    length = b2 & 0x7F
    if length == 126:
        ext = sock.recv(2)
        length = struct.unpack('!H', ext)[0]
    elif length == 127:
        ext = sock.recv(8)
        length = struct.unpack('!Q', ext)[0]
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk: return None
        data.extend(chunk)
    return data.decode('utf-8', errors='replace')

def run_e2e_test():
    print("========================================")
    print("Starting Visualizer End-to-End Test")
    print("========================================")

    port = 8085
    rb_port = 9099
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # 1. Launch visualizer server
    cmd = [
        sys.executable,
        os.path.join(root_dir, "scripts", "visualizer.py"),
        "--port", str(port),
        "--rosbridge-port", str(rb_port),
        "--links", "2",
        "--no-open"
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        # Wait for server to start
        server_ready = False
        for _ in range(50):
            time.sleep(0.1)
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=0.5)
                server_ready = True
                break
            except Exception:
                pass
        assert server_ready, "Visualizer server failed to start within 5 seconds"
        
        # Test 1: Static Files (HTTP GET)
        print("[TEST 1] HTTP GET Static Assets...")
        for path in ["/", "/style.css", "/app.js"]:
            url = f"http://127.0.0.1:{port}{path}"
            req = urllib.request.urlopen(url, timeout=3.0)
            assert req.status == 200, f"Expected 200 for {path}"
            content = req.read()
            assert len(content) > 100, f"File {path} too short"
            print(f"  -> {path} OK ({len(content)} bytes)")
        
        # Test 2: Status API
        print("[TEST 2] API /api/status...")
        req = urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=3.0)
        status_data = json.loads(req.read().decode('utf-8'))
        assert status_data.get("backend_listening") is True, f"Backend not listening: {status_data}"
        assert status_data.get("links") == 2, f"Expected 2 links: {status_data}"
        print(f"  -> Status verified: {status_data}")

        # Test 3: WebSocket Connection & Handshake
        print("[TEST 3] WebSocket Handshake (RFC 6455)...")
        ws_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ws_sock.settimeout(4.0)
        ws_sock.connect(('127.0.0.1', port))
        
        key = base64.b64encode(os.urandom(16)).decode('utf-8')
        handshake = (
            f"GET /ws HTTP/1.1\r\n"
            f"Host: 127.0.0.1:{port}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        ws_sock.sendall(handshake.encode('utf-8'))
        
        resp = ws_sock.recv(1024).decode('utf-8')
        assert "101 Switching Protocols" in resp, f"Handshake failed: {resp}"
        expected_accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode('utf-8')).digest()).decode('utf-8')
        assert expected_accept in resp, f"Invalid Sec-WebSocket-Accept in: {resp}"
        print("  -> WebSocket handshake SUCCESSFUL!")

        # Test 4: Rosbridge Pub/Sub over WebSocket
        print("[TEST 4] Topic Subscription (/joint_states)...")
        sub_req = json.dumps({"op": "subscribe", "topic": "/joint_states"})
        ws_sock.sendall(make_masked_ws_frame(sub_req))

        received_joint_states = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            frame_text = recv_ws_frame_simple(ws_sock)
            if not frame_text: continue
            data = json.loads(frame_text)
            if data.get("op") == "publish" and data.get("topic") == "/joint_states":
                pos = data.get("msg", {}).get("position", [])
                assert len(pos) == 2, f"Expected 2 joint angles, got {pos}"
                print(f"  -> Received /joint_states with 2 links: {pos}")
                received_joint_states = True
                break
        assert received_joint_states, "Failed to receive /joint_states message"

        # Test 5: Service Call (/ik/solve) over WebSocket
        print("[TEST 5] Service Call (/ik/solve)...")
        call_msg = json.dumps({
            "op": "call_service",
            "id": "test_ik_1",
            "service": "/ik/solve",
            "args": {"x": 1.0, "y": 0.5}
        })
        ws_sock.sendall(make_masked_ws_frame(call_msg))

        received_ik_resp = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            frame_text = recv_ws_frame_simple(ws_sock)
            if not frame_text: continue
            data = json.loads(frame_text)
            if data.get("op") == "service_response" and data.get("id") == "test_ik_1":
                assert data.get("result") is True, f"IK solve failed: {data}"
                positions = data.get("values", {}).get("positions", [])
                assert len(positions) == 2, f"Expected 2 joint angles: {positions}"
                print(f"  -> /ik/solve returned positions: {positions}")
                received_ik_resp = True
                break
        assert received_ik_resp, "Failed to receive /ik/solve response"

        # Test 6: Simulation Pause & Reset
        print("[TEST 6] Pause and Reset services...")
        pause_msg = json.dumps({
            "op": "call_service",
            "id": "test_pause",
            "service": "/arm_sim/pause",
            "args": {"data": True}
        })
        ws_sock.sendall(make_masked_ws_frame(pause_msg))
        
        paused_ok = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            frame_text = recv_ws_frame_simple(ws_sock)
            if not frame_text: continue
            data = json.loads(frame_text)
            if data.get("op") == "service_response" and data.get("id") == "test_pause":
                assert data.get("result") is True
                assert data.get("values", {}).get("data") is True
                print("  -> /arm_sim/pause verified!")
                paused_ok = True
                break
        assert paused_ok, "Failed to receive pause response"
        ws_sock.close()

        # Test 7: Mode Switching to 3-Link Arm via API
        print("[TEST 7] Switch to 3-Link Mode via API...")
        req = urllib.request.urlopen(f"http://127.0.0.1:{port}/api/backend/restart?links=3", timeout=5.0)
        restart_data = json.loads(req.read().decode('utf-8'))
        assert restart_data.get("success") is True, f"Restart failed: {restart_data}"
        assert restart_data.get("links") == 3, f"Expected 3 links: {restart_data}"
        time.sleep(0.5)

        # Connect fresh WebSocket and check 3 links
        ws_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ws_sock2.settimeout(4.0)
        ws_sock2.connect(('127.0.0.1', port))
        key2 = base64.b64encode(os.urandom(16)).decode('utf-8')
        ws_sock2.sendall((
            f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key2}\r\n\r\n"
        ).encode('utf-8'))
        resp2 = ws_sock2.recv(1024).decode('utf-8')
        assert "101 Switching Protocols" in resp2

        sub_req2 = json.dumps({"op": "subscribe", "topic": "/joint_states"})
        ws_sock2.sendall(make_masked_ws_frame(sub_req2))

        received_3links = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            frame_text = recv_ws_frame_simple(ws_sock2)
            if not frame_text: continue
            data = json.loads(frame_text)
            if data.get("op") == "publish" and data.get("topic") == "/joint_states":
                pos = data.get("msg", {}).get("position", [])
                assert len(pos) == 3, f"Expected 3 links, got {pos}"
                print(f"  -> Verified 3-Link Mode live data: {pos}")
                received_3links = True
                break
        assert received_3links, "Failed to receive 3-link joint states"
        ws_sock2.close()

        print("========================================")
        print("ALL VISUALIZER TESTS PASSED SUCCESSFULLY!")
        print("========================================")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

if __name__ == "__main__":
    run_e2e_test()
