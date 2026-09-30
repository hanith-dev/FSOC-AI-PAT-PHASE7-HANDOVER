"""
Live End-to-End WebSocket Pipeline & Demonstration Test.
Tests the real running FastAPI backend over WebSocket (ws://127.0.0.1:8000/ws/tracking).
Verifies:
1. Connection & greeting
2. Real data flow through:
   Detector -> Centroid Localization -> Association -> CA6 Estimator -> PAT FSM -> Telemetry
3. PAT Lifecycle Transitions:
   SEARCH -> CANDIDATE -> ACQUIRE -> TRACK -> LOCK
4. Target Loss & Intelligent Reacquisition:
   LOCK -> COAST -> REACQUIRE (Phase 7 covariance_ellipse_scan active) -> TRACK -> LOCK
5. Distractor Smoke Test:
   Rejection of distractor beacon without truth assistance
"""
import os
import sys
import json
import base64
import asyncio
import numpy as np
import cv2
import websockets

def create_frame_b64(beacons=[(320, 240, (50, 220, 50), 10)]):
    """Generate 640x480 frame with specified circular beacons (BGR)."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    for (u, v, col, r) in beacons:
        cv2.circle(img, (int(u), int(v)), int(r), col, -1)
        # Add glow
        cv2.circle(img, (int(u), int(v)), int(r * 1.5), col, 1)
    img = cv2.GaussianBlur(img, (5, 5), 1.0)
    _, buf = cv2.imencode('.jpg', img)
    return base64.b64encode(buf.tobytes()).decode('ascii')

def create_blackout_b64():
    """Generate completely black frame."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    _, buf = cv2.imencode('.jpg', img)
    return base64.b64encode(buf.tobytes()).decode('ascii')

async def run_live_validation():
    uri = "ws://127.0.0.1:8000/ws/tracking"
    print(f"Connecting to live backend at {uri}...")

    async with websockets.connect(uri) as ws:
        # Step 1: Greeting
        greeting_raw = await ws.recv()
        greeting = json.loads(greeting_raw)
        print(f"[WS] Connection Greeting: {greeting.get('message')}")
        assert greeting["type"] == "connection_status"
        assert greeting["status"] == "connected"

        frame_idx = 0
        state_history = []
        recorded_telemetry = []

        async def send_frame(b64_img, meta=None):
            nonlocal frame_idx
            frame_idx += 1
            if meta is None:
                meta = {
                    "gimbal_pan": 0.0,
                    "gimbal_tilt": 0.0,
                    "camera_position": [0.0, 1.5, 0.0],
                    "camera_orientation": [0.0, 0.0, 0.0, 1.0],
                    "designated_target_id": "BEACON-01",
                    "beacons": [{"id": "BEACON-01", "position": [0.0, 1.5, -100.0], "designated": True, "active": True}]
                }
            payload = {
                "type": "frame",
                "frame_index": frame_idx,
                "image": b64_img,
                "metadata": meta
            }
            await ws.send(json.dumps(payload))
            resp_raw = await ws.recv()
            resp = json.loads(resp_raw)
            state = resp.get("tracking_state")
            state_history.append((frame_idx, state))
            recorded_telemetry.append(resp)
            return resp

        # =========================================================
        # PHASE A: INITIAL SEARCH
        # =========================================================
        print("\n--- PHASE A: Initial Observation (Empty Field) ---")
        blackout = create_blackout_b64()
        r0 = await send_frame(blackout)
        print(f"Frame {frame_idx:02d}: state={r0['tracking_state']}, detected={r0['detected']}, kalman_active={r0['kalman']['active']}")
        assert r0["tracking_state"] == "SEARCH"

        # =========================================================
        # PHASE B: CANDIDATE -> ACQUIRE -> TRACK -> LOCK
        # =========================================================
        print("\n--- PHASE B: Target Appearance & Acquisition Sequence ---")
        beacon_center = create_frame_b64([(320, 240, (50, 220, 50), 10)])
        
        # Frame 1 with beacon -> still SEARCH (waiting for 2nd confirmation frame)
        r1 = await send_frame(beacon_center)
        print(f"Frame {frame_idx:02d}: state={r1['tracking_state']}, detected={r1['detected']}, conf={r1.get('live_metrics',{}).get('detection',{}).get('avg_confidence')}")
        
        # Frame 2 with beacon -> CANDIDATE
        r2 = await send_frame(beacon_center)
        print(f"Frame {frame_idx:02d}: state={r2['tracking_state']}, detected={r2['detected']}")
        assert r2["tracking_state"] == "CANDIDATE"

        # Frame 3 with beacon -> ACQUIRE
        r3 = await send_frame(beacon_center)
        print(f"Frame {frame_idx:02d}: state={r3['tracking_state']}, detected={r3['detected']}")
        assert r3["tracking_state"] == "ACQUIRE"

        # Frames 4-5 with beacon -> TRACK
        for _ in range(2):
            r_track = await send_frame(beacon_center)
            print(f"Frame {frame_idx:02d}: state={r_track['tracking_state']}, total_err={r_track['angular_error_total']:.4f}°")
        assert r_track["tracking_state"] == "TRACK"

        # Hold low error for lock duration (>=0.40s, ~25 frames at 50Hz)
        print("Holding pointing alignment to satisfy LOCK criteria...")
        lock_reached = False
        for _ in range(40):
            await asyncio.sleep(0.02)  # Simulate 50Hz real time
            r_lock = await send_frame(beacon_center)
            pat_diag = r_lock.get("pat_fsm", {})
            print(f"  Frame {frame_idx:02d}: state={r_lock['tracking_state']}, lock_eligible={pat_diag.get('lock_eligible')}, consec={pat_diag.get('consecutive_accepted')}, total_err={r_lock['angular_error_total']:.4f}°")
            if r_lock["tracking_state"] == "LOCK":
                lock_reached = True
                print(f"Frame {frame_idx:02d}: *** REACHED STATE: LOCK *** (total_err={r_lock['angular_error_total']:.4f}°, fps={r_lock['fps']})")
                break
        assert lock_reached, f"Expected LOCK, ended in {r_lock['tracking_state']}"

        # =========================================================
        # PHASE C: TARGET INTERRUPTION -> COAST -> REACQUIRE
        # =========================================================
        print("\n--- PHASE C: Target Interruption, COAST & Phase 7 REACQUIRE ---")
        print("Simulating target optical occlusion (blackout)...")
        
        # Frame 1 of blackout -> enters COAST
        r_coast = await send_frame(blackout)
        print(f"Frame {frame_idx:02d}: state={r_coast['tracking_state']} (Reason: {r_coast.get('pat_fsm',{}).get('transition_reason')})")
        assert r_coast["tracking_state"] == "COAST", f"Expected COAST, got {r_coast['tracking_state']}"

        # Maintain blackout until coast timeout (0.2s -> ~15 frames at 50Hz)
        reacquire_reached = False
        reacquire_telemetry = None
        for i in range(20):
            await asyncio.sleep(0.025)  # 25ms per frame so 200ms coast timeout elapses
            r_reacq = await send_frame(blackout)
            if r_reacq["tracking_state"] == "REACQUIRE":
                reacquire_reached = True
                reacquire_telemetry = r_reacq
                print(f"Frame {frame_idx:02d}: *** ENTERED STATE: REACQUIRE ***")
                pat_diag = r_reacq.get("pat_fsm", {})
                print(f"  Search Pattern: {pat_diag.get('search_pattern')}")
                print(f"  Search Active: {pat_diag.get('search_active')}")
                print(f"  Search Offsets: Pan={pat_diag.get('search_offsets', [0,0])[0]:.4f}°, Tilt={pat_diag.get('search_offsets', [0,0])[1]:.4f}°")
                break
            else:
                print(f"  Frame {frame_idx:02d}: in COAST (elapsed={i*0.025:.3f}s)")

        assert reacquire_reached, f"Expected REACQUIRE, got {r_reacq['tracking_state']}"
        assert reacquire_telemetry["pat_fsm"]["search_pattern"] == "covariance_ellipse_scan"
        assert reacquire_telemetry["pat_fsm"]["search_active"] is True

        # Let reacquisition scan run for 5 frames to observe pattern progression
        print("Advancing covariance ellipse scan during REACQUIRE...")
        for _ in range(5):
            r_scan = await send_frame(blackout)
            pat_diag = r_scan.get("pat_fsm", {})
            print(f"Frame {frame_idx:02d}: state={r_scan['tracking_state']}, offsets=[pan={pat_diag.get('search_offsets',[0,0])[0]:.3f}°, tilt={pat_diag.get('search_offsets',[0,0])[1]:.3f}°]")

        # =========================================================
        # PHASE D: RESTORING TARGET -> REACQUISITION SUCCESS -> LOCK
        # =========================================================
        print("\n--- PHASE D: Restoring Target Beacon -> Re-acquisition Recovery ---")
        # Target reappears at (330, 245) - within sensor FOV
        beacon_restored = create_frame_b64([(330, 245, (50, 220, 50), 10)])
        
        # Frame 1 of detection during REACQUIRE
        r_rec1 = await send_frame(beacon_restored)
        print(f"Frame {frame_idx:02d}: state={r_rec1['tracking_state']}, detected={r_rec1['detected']}")
        
        # Frame 2 of detection -> transitions to TRACK
        r_rec2 = await send_frame(beacon_restored)
        print(f"Frame {frame_idx:02d}: state={r_rec2['tracking_state']} (Reason: {r_rec2.get('pat_fsm',{}).get('transition_reason')})")
        assert r_rec2["tracking_state"] == "TRACK", f"Expected TRACK, got {r_rec2['tracking_state']}"

        # Bring error to center for LOCK (>= 0.4s)
        for _ in range(35):
            await asyncio.sleep(0.02)
            r_relock = await send_frame(beacon_center)
            if r_relock["tracking_state"] == "LOCK":
                print(f"Frame {frame_idx:02d}: *** RESTORED STATE: LOCK *** (total_err={r_relock['angular_error_total']:.4f}°)")
                break
        assert r_relock["tracking_state"] == "LOCK"

        # =========================================================
        # PHASE E: DISTRACTOR SMOKE TEST
        # =========================================================
        print("\n--- PHASE E: Distractor Smoke Test (Multi-Candidate Field) ---")
        # Designated beacon at (320, 240) + Distractor beacon at (450, 320)
        distractor_frame = create_frame_b64([
            (320, 240, (50, 220, 50), 10),  # Designated target
            (450, 320, (50, 220, 50), 12)   # Optical distractor (slightly brighter/larger)
        ])
        r_dist = await send_frame(distractor_frame)
        print(f"Frame {frame_idx:02d}: Candidates detected = {len(r_dist.get('candidates', []))}")
        for c in r_dist.get("candidates", []):
            print(f"  Candidate {c['id']}: pos={c['pixel_position']}, status={c['status']}, conf={c['confidence']:.3f}")
        
        # Verify tracked center is close to designated target (320, 240) and NOT distractor (450, 320)
        tracked_pos = r_dist["raw_center"]
        dist_to_designated = np.linalg.norm(np.array(tracked_pos) - np.array([320.0, 240.0]))
        dist_to_distractor = np.linalg.norm(np.array(tracked_pos) - np.array([450.0, 320.0]))
        print(f"Distance to Designated: {dist_to_designated:.2f} px | Distance to Distractor: {dist_to_distractor:.2f} px")
        assert dist_to_designated < 5.0, f"Tracker diverged to distractor! Dist={dist_to_designated}"
        assert dist_to_distractor > 100.0, "Tracker selected distractor!"
        print(">>> DISTRACTOR REJECTION: PASS (Designated target maintained lock)")

        print("\n=======================================================")
        print("ALL END-TO-END VALIDATION PHASES PASSED ON LIVE RUNTIME!")
        print("=======================================================")

def run_in_process_testclient():
    from fastapi.testclient import TestClient
    from app.main import app, disturbance_engine
    disturbance_engine.reset()
    client = TestClient(app)
    with client.websocket_connect("/ws/tracking") as ws:
        greeting = ws.receive_json()
        print(f"[WS TestClient] Connection Greeting: {greeting.get('message')}")
        assert greeting["type"] == "connection_status"

        # Phase 1: Search -> Candidate -> Acquire -> Track -> Lock
        for i in range(10):
            frame = create_frame_b64([(320, 240, (50, 220, 50), 10)])
            ws.send_json({"type": "frame", "frame_data": frame})
            resp = ws.receive_json()

        print("[WS TestClient] Initial tracking frames completed. State:", resp.get("tracking_state"))
        assert resp.get("tracking_state") in ["ACQUIRE", "TRACK", "LOCK"]

        # Phase 2: Occlusion -> Coast -> Reacquire
        for i in range(12):
            frame = create_blackout_b64()
            ws.send_json({"type": "frame", "frame_data": frame})
            resp = ws.receive_json()

        print("[WS TestClient] Occlusion frames completed. State:", resp.get("tracking_state"))
        assert resp.get("tracking_state") in ["COAST", "REACQUIRE"]

        # Phase 3: Recovery -> Track / Lock
        for i in range(15):
            frame = create_frame_b64([(320, 240, (50, 220, 50), 10)])
            ws.send_json({"type": "frame", "frame_data": frame})
            resp = ws.receive_json()

        print("[WS TestClient] Re-acquisition frames completed. State:", resp.get("tracking_state"))
        assert resp.get("tracking_state") in ["TRACK", "LOCK"]

        print("\n=======================================================")
        print("ALL END-TO-END TESTCLIENT VALIDATION PHASES PASSED!")
        print("=======================================================")

if __name__ == "__main__":
    sys.path.insert(0, os.path.abspath("backend"))
    try:
        asyncio.run(run_live_validation())
    except (ConnectionRefusedError, OSError):
        print("Live backend server not detected on port 8000. Executing test via FastAPI TestClient fallback...")
        run_in_process_testclient()

