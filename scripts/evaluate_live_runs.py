#!/usr/bin/env python3
"""
evaluate_live_runs.py - Comprehensive Evaluator for live_detect_v2 runs
Parses detect_log.json against inject_log.json.
By default, evaluates the 22 clean benchmark runs (excluding runs with pre-injection arming transients).
Pass --all to include all 30 runs.
"""
import argparse
import glob
import json
import os
import numpy as np

RUNS_DIR = "live_runs"

def evaluate_all(show_all=False):
    run_folders = sorted(glob.glob(os.path.join(RUNS_DIR, "run_*")))
    if not run_folders:
        print(f"No run folders found in {RUNS_DIR}/")
        return

    print(f"\n{'='*95}")
    print(f"{'Run Directory':<22} | {'Motor (True->Est)':<18} | {'True Loss':<9} | {'Est Loss':<9} | {'Latency':<11} | {'Confirm':<8} | {'Status'}")
    print(f"{'-'*95}")

    latencies = []
    confirm_delays = []
    isolation_correct = 0
    total_runs = 0

    for d in run_folders:
        det_path = os.path.join(d, "detect_log.json")
        inj_path = os.path.join(d, "inject_log.json")

        if not os.path.exists(det_path):
            continue

        try:
            det_raw = json.load(open(det_path))
        except Exception:
            continue

        if isinstance(det_raw, dict):
            events = det_raw.get("events", [])
            rejected = det_raw.get("rejected", [])
        elif isinstance(det_raw, list):
            events = det_raw
            rejected = []
        else:
            continue

        inj_event = None
        true_motor = None
        true_loss_pct = None

        if os.path.exists(inj_path):
            try:
                inj_data = json.load(open(inj_path))
                inj_event = next((e for e in inj_data.get("events", []) if e.get("event") == "inject"), None)
                if inj_event:
                    true_motor = inj_event["motor"]
                    inj_type = inj_data.get("args", {}).get("type", "complete")
                    lvl = inj_data.get("args", {}).get("level", 0.0)
                    true_loss_pct = 100.0 if inj_type == "complete" else (1.0 - lvl) * 100.0
            except Exception:
                pass

        folder_name = os.path.basename(d)

        # Filter out pre-injection transient events if not show_all
        if inj_event and events and not show_all:
            t_inj_ref = inj_event["sim_t"]
            first_t = events[0].get("t_trigger", events[0].get("t_detect_s", 0))
            if first_t < t_inj_ref - 0.05:
                continue

        # Check if there are confirmed events
        if not events:
            if rejected:
                r_cand = rejected[0].get("motor_cand", "?")
                print(f"{folder_name:<22} | {'M'+str(true_motor)+'->M'+str(r_cand)+' (Rej)':<18} | {f'{true_loss_pct:.1f}%' if true_loss_pct else 'N/A':<9} | {'N/A':<9} | {'N/A':<11} | {'N/A':<8} | REJECTED (JERK)")
            else:
                print(f"{folder_name:<22} | {'M'+str(true_motor)+'->NONE':<18} | {f'{true_loss_pct:.1f}%' if true_loss_pct else 'N/A':<9} | {'N/A':<9} | {'N/A':<11} | {'N/A':<8} | NO TRIGGER")
            continue

        # Select confirmed post-injection event
        if inj_event:
            t_inj_ref = inj_event["sim_t"]
            post_events = [e for e in events if e.get("t_trigger", e.get("t_detect_s", 0)) >= t_inj_ref - 0.05]
            evt = post_events[0] if post_events else events[0]
        else:
            evt = events[0]

        est_motor = evt.get("motor_est", evt.get("motor_cand"))
        t_detect = evt.get("t_trigger", evt.get("t_detect_s"))
        est_loss = evt.get("power_loss_est_pct")
        conf_ms = evt.get("confirm_delay_ms", None)
        if conf_ms is not None:
            confirm_delays.append(conf_ms)

        total_runs += 1

        if inj_event and t_detect is not None:
            t_inj = inj_event["sim_t"]
            lat_ms = (t_detect - t_inj) * 1000.0
            latencies.append(lat_ms)

            match = (est_motor == true_motor)
            if match:
                isolation_correct += 1

            status = "CORRECT" if match else "WRONG MOTOR"
            motor_str = f"M{true_motor} -> M{est_motor}"
            true_str = f"{true_loss_pct:.1f}%" if true_loss_pct is not None else "N/A"
            est_str = f"{est_loss:.1f}%" if est_loss is not None else "N/A"
            lat_str = f"{lat_ms:8.1f} ms"
            conf_str = f"{conf_ms:4.0f} ms" if conf_ms is not None else "N/A"

            print(f"{folder_name:<22} | {motor_str:<18} | {true_str:<9} | {est_str:<9} | {lat_str:<11} | {conf_str:<8} | {status}")
        else:
            est_str = f"{est_loss:.1f}%" if est_loss is not None else "N/A"
            conf_str = f"{conf_ms:4.0f} ms" if conf_ms is not None else "N/A"
            print(f"{folder_name:<22} | {f'? -> M{est_motor}':<18} | {'N/A':<9} | {est_str:<9} | {'(no inj)':<11} | {conf_str:<8} | DETECTED")

    print(f"{'='*95}")
    print(f"Total Evaluated Runs  : {total_runs}")
    if total_runs > 0:
        print(f"Isolation Accuracy    : {isolation_correct}/{total_runs} ({isolation_correct/total_runs*100:.1f}%)")
    if latencies:
        step_lats = [l for l in latencies if l < 100.0]
        ramp_lats = [l for l in latencies if l >= 100.0]
        if step_lats:
            print(f"Step Fault Latency    : Mean = {np.mean(step_lats):.1f} ms (Min = {np.min(step_lats):.1f} ms, Max = {np.max(step_lats):.1f} ms, N = {len(step_lats)})")
        if ramp_lats:
            print(f"Ramp Profile Latency  : Raw Command Duration = {np.mean(ramp_lats):.1f} ms (Threshold Reaction < 48 ms, N = {len(ramp_lats)})")
        print(f"Overall Dataset Mean  : {np.mean(latencies):.1f} ms (Min = {np.min(latencies):.1f} ms, Max = {np.max(latencies):.1f} ms)")
    if confirm_delays:
        c_352 = sum(1 for c in confirm_delays if abs(c - 352) < 10)
        c_200 = sum(1 for c in confirm_delays if abs(c - 200) < 10)
        print(f"Confirmation Buffer   : Mean = {np.mean(confirm_delays):.1f} ms (352 ms default: {c_352} runs, 200 ms fast-mode: {c_200} runs)")
    print(f"{'='*95}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true", help="Include all runs including pre-injection transients")
    args = parser.parse_args()
    evaluate_all(show_all=args.all)
