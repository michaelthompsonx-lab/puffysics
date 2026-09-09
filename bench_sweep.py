#!/usr/bin/env python3
"""Run reproducible Puffysics benchmark matrices; preserve failures and raw output."""
import argparse
import csv
import datetime
import io
import itertools
import json
import pathlib
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("binary", type=pathlib.Path)
    ap.add_argument("--output", required=True, type=pathlib.Path)
    ap.add_argument("--scenes", nargs="+", default=["flight", "stack", "grains", "chain"])
    ap.add_argument("--worlds", nargs="+", type=int, default=[128, 2048])
    ap.add_argument("--bodies", nargs="+", type=int, default=[8, 32])
    ap.add_argument("--substeps", nargs="+", type=int, default=[1, 4, 8])
    ap.add_argument("--phase", choices=["step", "find"], default="step")
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--threads", type=int, default=64)
    ap.add_argument("--cpu", action="store_true")
    args = ap.parse_args()
    records = []
    rows = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for scene, worlds, bodies, subs in itertools.product(args.scenes, args.worlds, args.bodies, args.substeps):
        cmd = [str(args.binary.resolve()), "--scene", scene, "--worlds", str(worlds),
               "--bodies", str(bodies), "--substeps", str(subs), "--phase", args.phase,
               "--steps", str(args.steps), "--repeats", str(args.repeats), "--threads", str(args.threads)]
        if args.cpu:
            cmd.append("--cpu")
        result = subprocess.run(cmd, capture_output=True, text=True)
        records.append(dict(command=cmd, returncode=result.returncode,
                            stdout=result.stdout, stderr=result.stderr))
        parsed = list(csv.DictReader(io.StringIO(result.stdout)))
        if parsed:
            row = parsed[-1]
            row["returncode"] = result.returncode
            rows.append(row)
        print(f"{scene} worlds={worlds} bodies={bodies} substeps={subs}: exit={result.returncode}", flush=True)
        # Save after every case so interrupted sweeps remain usable.
        args.output.with_suffix(".json").write_text(json.dumps(dict(
            timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            records=records), indent=2) + "\n")
        if rows:
            with args.output.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    return int(any(r["returncode"] for r in records))


if __name__ == "__main__":
    raise SystemExit(main())
