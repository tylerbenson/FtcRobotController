#!/usr/bin/env python3
"""Record Limelight 3A REST results to portable JSONL sessions (Python stdlib only)."""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import time
import urllib.error
import urllib.parse
import urllib.request


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def results_object(payload):
    """Older firmware may wrap the result; keep the original response on disk."""
    if not isinstance(payload, dict):
        raise ValueError("Expected a JSON object from /results")
    result = payload.get("Results", payload)
    if not isinstance(result, dict):
        raise ValueError("Results must be a JSON object")
    return result


def base_url(host):
    parsed = urllib.parse.urlsplit(host if "://" in host else "http://" + host)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        raise ValueError("Use a hostname, IP address, or http(s)://host:port")
    if parsed.path not in ("", "/") or parsed.query or parsed.fragment or parsed.username:
        raise ValueError("Supply the camera host, not an endpoint path or credentials")
    hostname = parsed.hostname
    if ":" in hostname:
        hostname = "[" + hostname + "]"
    return f"{parsed.scheme}://{hostname}:{parsed.port or 5807}"


class Client:
    def __init__(self, host, timeout):
        self.base = base_url(host)
        self.timeout = timeout
        # USB camera traffic should not go through a computer's HTTP proxy.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get(self, endpoint):
        request = urllib.request.Request(self.base + endpoint, headers={"Cache-Control": "no-cache"})
        with self.opener.open(request, timeout=self.timeout) as response:
            data = response.read(8 * 1024 * 1024 + 1)
        if len(data) > 8 * 1024 * 1024:
            raise ValueError("JSON response exceeds 8 MB")
        return json.loads(data)


class FrameClock:
    """Track repeated camera frames separately from repeated successful HTTP requests."""
    def __init__(self):
        self.previous = None
        self.first_seen = 0

    def observe(self, result, received_ms):
        identity = tuple((key, result[key]) for key in ("ts_us", "ts", "fidx", "pID")
                         if key in result and isinstance(result[key], (int, float)))
        if not any(key in result for key in ("ts_us", "ts", "fidx")) or not identity:
            return None, None
        duplicate = identity == self.previous
        if not duplicate:
            self.previous, self.first_seen = identity, received_ms
        return duplicate, received_ms - self.first_seen


def tag_summary(result):
    tags = result.get("Fiducial", [])
    if not isinstance(tags, list):
        return [], 0
    ids, with_corners = [], 0
    for tag in tags:
        if not isinstance(tag, dict):
            continue
        ids.append(tag.get("fID"))
        corners = tag.get("pts", [])
        if isinstance(corners, list) and len(corners) == 4 and all(
                isinstance(p, list) and len(p) == 2 for p in corners):
            with_corners += 1
    return ids, with_corners


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def snapshot(client, endpoint):
    try:
        return {"captured_utc": utc_now(), "response": client.get(endpoint)}
    except (OSError, ValueError, urllib.error.URLError) as error:
        return {"captured_utc": utc_now(), "error": str(error)}


def record(args):
    client = Client(args.host, args.timeout)
    calibration = json.loads(args.calibration.read_text()) if args.calibration else None
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "-", args.scenario).strip("-") or "session"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ")
    directory = args.output / f"{stamp}-{slug}"
    directory.mkdir(parents=True, exist_ok=False)
    metadata = {
        "schema_version": 1, "device_model": "Limelight 3A", "started_utc": utc_now(),
        "scenario": args.scenario, "alliance": args.alliance, "side": args.side,
        "expected_state": args.expected_state, "expected_distance_m": args.distance,
        "mount": args.mount, "notes": args.notes, "base_url": client.base,
        "requested_poll_hz": args.hz, "calibration": calibration,
        "complete": False, "snapshots": {},
    }
    write_json(directory / "session.json", metadata)
    print(f"Session: {directory}\nReading camera configuration…", flush=True)
    try:
        # Optional endpoints vary by firmware; failure is recorded, not fatal.
        for endpoint in ("/status", "/hwreport", "/cal-default", "/cal-file", "/cal-eeprom"):
            metadata["snapshots"][endpoint] = snapshot(client, endpoint)
        if args.pipeline_index is not None:
            endpoint = f"/pipeline-atindex?index={args.pipeline_index}"
            metadata["snapshots"][endpoint] = snapshot(client, endpoint)
        write_json(directory / "session.json", metadata)
        if not args.no_wait and args.duration is None:
            input("Position camera, then press Enter to record. Ctrl+C stops and saves. ")
        run_capture(client, directory, args.hz, args.duration)
        metadata["complete"] = True
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
        metadata["complete"] = True
    finally:
        metadata["ended_utc"] = utc_now()
        write_json(directory / "session.json", metadata)
    if (directory / "frames.jsonl").exists():
        summary = inspect_session(directory)
        print(json.dumps(summary, indent=2))
    print(f"Saved: {directory}")
    return directory


def run_capture(client, directory, hz, duration):
    start = time.monotonic()
    clock = FrameClock()
    seq = 0
    next_display = 0
    with (directory / "frames.jsonl").open("x", encoding="utf-8", buffering=1) as output:
        while duration is None or time.monotonic() - start < duration:
            sent = time.monotonic()
            row = {"seq": seq, "request_ms": (sent - start) * 1000, "received_utc": None}
            try:
                payload = client.get("/results")
                received = time.monotonic()
                result = results_object(payload)
                duplicate, age = clock.observe(result, (received - start) * 1000)
                ids, corners = tag_summary(result)
                row.update(response=payload, duplicate=duplicate, observed_staleness_ms=age,
                           tag_ids=ids, tags_with_corners=corners)
            except (OSError, ValueError, urllib.error.URLError) as error:
                received = time.monotonic()
                row["error"] = str(error)
            row.update(received_utc=utc_now(), elapsed_ms=(received - start) * 1000,
                       request_duration_ms=(received - sent) * 1000)
            output.write(json.dumps(row, allow_nan=False) + "\n")
            if received - start >= next_display:
                if "error" in row:
                    print(f"{received-start:6.1f}s  connection/read error: {row['error']}", flush=True)
                else:
                    print(f"{received-start:6.1f}s  poll {seq}  tags={row['tag_ids']}  "
                          f"corners={corners}/{len(ids)}  duplicate={duplicate}", flush=True)
                next_display = received - start + 1
            seq += 1
            pause = max(0, 1 / hz - (time.monotonic() - sent))
            if duration is not None:
                pause = min(pause, max(0, duration - (time.monotonic() - start)))
            time.sleep(pause)


def rows(path):
    """Read complete JSONL rows; tolerate only an interrupted, unterminated final row."""
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                if not line.endswith("\n"):
                    print(f"Ignoring interrupted final row {number}")
                    return
                raise ValueError(f"Invalid JSON in {path}, line {number}")


def inspect_session(directory):
    counts = Counter()
    ids = Counter()
    last_ms = 0
    fields = ["seq", "elapsed_ms", "request_duration_ms", "duplicate", "observed_staleness_ms",
              "valid", "pipeline", "tag_ids", "tags_with_corners", "tx", "ty", "ta",
              "capture_latency_ms", "processing_latency_ms", "error"]
    with (directory / "frames.csv").open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        for row in rows(directory / "frames.jsonl"):
            counts["polls"] += 1
            last_ms = row["elapsed_ms"]
            view = {key: row.get(key) for key in fields}
            if "error" in row:
                counts["errors"] += 1
            else:
                result = results_object(row["response"])
                tag_ids, corners = tag_summary(result)
                counts["successful_polls"] += 1
                counts["duplicate_polls"] += row.get("duplicate") is True
                counts["observed_new_frames"] += row.get("duplicate") is False
                counts["unidentified_frame_polls"] += row.get("duplicate") is None
                counts["valid_polls"] += result.get("v") == 1
                counts["polls_with_tags"] += bool(tag_ids)
                counts["polls_with_missing_corners"] += corners < len(tag_ids)
                ids.update(str(tag) for tag in tag_ids)
                view.update(valid=result.get("v"), pipeline=result.get("pID"),
                            tag_ids=";".join(str(tag) for tag in tag_ids), tags_with_corners=corners,
                            tx=result.get("tx"), ty=result.get("ty"), ta=result.get("ta"),
                            capture_latency_ms=result.get("cl"), processing_latency_ms=result.get("tl"))
            writer.writerow(view)
    summary = {**counts, "duration_seconds": last_ms / 1000,
               "tag_poll_counts": dict(ids)}
    write_json(directory / "summary.json", summary)
    return summary


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Must be a finite positive number")
    return number


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("record", help="Capture a labeled session; Ctrl+C saves")
    capture.add_argument("--host", default="limelight.local", help="Camera host/IP (REST port defaults to 5807)")
    capture.add_argument("--scenario", required=True, help="e.g. red-front-2m or blue-moving-left")
    capture.add_argument("--alliance", choices=["red", "blue"], required=True)
    capture.add_argument("--side", choices=["audience", "scoring-table", "either"], default="either")
    capture.add_argument("--expected-state", choices=["upright", "inverted", "moving", "none", "unknown"], default="unknown")
    capture.add_argument("--distance", type=positive, help="Measured camera-to-opening slant distance in meters (stationary only)")
    capture.add_argument("--mount", choices=["handheld", "fixed", "turret"], default="handheld")
    capture.add_argument("--notes", default="")
    capture.add_argument("--duration", type=positive, help="Seconds to record; otherwise Ctrl+C to stop")
    capture.add_argument("--hz", type=positive, default=30, help="Maximum HTTP polling rate (default 30)")
    capture.add_argument("--timeout", type=positive, default=1, help="Per-request timeout in seconds")
    capture.add_argument("--pipeline-index", type=int, choices=range(10), help="Read/save this pipeline's config; does not switch it")
    capture.add_argument("--calibration", type=Path, help="Optional replay calibration JSON to embed")
    capture.add_argument("--output", type=Path, default=Path("recordings/limelight"))
    capture.add_argument("--no-wait", action="store_true", help="Start without the positioning prompt")
    inspect = commands.add_parser("inspect", help="Generate CSV and summary from an existing recording")
    inspect.add_argument("session", type=Path)
    return root


def main():
    args = parser().parse_args()
    try:
        if args.command == "record":
            record(args)
        else:
            print(json.dumps(inspect_session(args.session), indent=2))
    except (OSError, ValueError) as error:
        raise SystemExit(str(error))


if __name__ == "__main__":
    main()
