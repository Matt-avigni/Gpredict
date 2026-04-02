#!/usr/bin/env python3
import argparse
import sys

from rigctld_diag import (
    MAIN_VFO_TOKENS,
    SUB_VFO_TOKENS,
    RigctldConnection,
    format_result,
    probe_plain_set_once,
    probe_token_once,
)


def choose_token(host, port, timeout_ms, tokens, freq_hz):
    for token in tokens:
        select_result, freq_result = probe_token_once(
            host, port, timeout_ms, token, freq_hz
        )
        if select_result["ok"] and freq_result is not None and freq_result["ok"]:
            return token
    return None


def run_plain_repeat(host, port, timeout_ms, freq_hz, cycles):
    steps = []
    with RigctldConnection(host, port, timeout_ms) as conn:
        for index in range(cycles):
            result = conn.request_rprt("F %d" % (freq_hz + index), timeout_ms)
            steps.append(result)
            if not result["ok"]:
                break
    return steps


def run_select_repeat(host, port, timeout_ms, token, freq_hz, cycles):
    steps = []
    with RigctldConnection(host, port, timeout_ms) as conn:
        for index in range(cycles):
            select_result = conn.request_rprt("V %s" % token, timeout_ms)
            set_result = conn.request_rprt("F %d" % (freq_hz + index), timeout_ms)
            steps.append({"select": select_result, "set": set_result})
            if not select_result["ok"] or not set_result["ok"]:
                break
    return steps


def run_plain_after_select(host, port, timeout_ms, token, freq_hz, cycles):
    steps = []
    with RigctldConnection(host, port, timeout_ms) as conn:
        prime_select = conn.request_rprt("V %s" % token, timeout_ms)
        prime_set = conn.request_rprt("F %d" % freq_hz, timeout_ms)
        steps.append({"select": prime_select, "set": prime_set, "prime": True})
        if not prime_select["ok"] or not prime_set["ok"]:
            return steps

        for index in range(1, cycles + 1):
            set_result = conn.request_rprt("F %d" % (freq_hz + index), timeout_ms)
            steps.append({"set": set_result, "prime": False})
            if not set_result["ok"]:
                break
    return steps


def print_plain_case(label, steps):
    print("")
    print("== %s ==" % label)
    for index, step in enumerate(steps, start=1):
        print(
            "iter=%02d result=%-10s elapsed_ms=%6.1f"
            % (index, format_result(step), step["elapsed_ms"])
        )
    success = sum(1 for step in steps if step["ok"])
    print("summary=%d/%d" % (success, len(steps)))
    return success == len(steps)


def print_select_case(label, steps):
    print("")
    print("== %s ==" % label)
    success = 0
    total = 0
    for index, step in enumerate(steps, start=1):
        total += 1
        if step.get("prime"):
            print(
                "prime select=%-10s set=%-10s"
                % (format_result(step["select"]), format_result(step["set"]))
            )
            if step["select"]["ok"] and step["set"]["ok"]:
                success += 1
            continue
        if "select" in step:
            print(
                "iter=%02d select=%-10s set=%-10s"
                % (
                    index,
                    format_result(step["select"]),
                    format_result(step["set"]),
                )
            )
            if step["select"]["ok"] and step["set"]["ok"]:
                success += 1
        else:
            print(
                "iter=%02d set=%-10s elapsed_ms=%6.1f"
                % (index - 1, format_result(step["set"]), step["set"]["elapsed_ms"])
            )
            if step["set"]["ok"]:
                success += 1
    print("summary=%d/%d" % (success, total))
    return success == total


def main():
    parser = argparse.ArgumentParser(
        description="Stress rigctld Main/Sub handshake stability with raw commands."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4532)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--main-freq", type=int, default=437800000)
    parser.add_argument("--sub-freq", type=int, default=145990000)
    args = parser.parse_args()

    main_token = choose_token(
        args.host, args.port, args.timeout_ms, MAIN_VFO_TOKENS, args.main_freq
    )
    sub_token = choose_token(
        args.host, args.port, args.timeout_ms, SUB_VFO_TOKENS, args.sub_freq
    )

    print("chosen_main_token=%s" % (main_token or "(none)"))
    print("chosen_sub_token=%s" % (sub_token or "(none)"))

    overall_ok = True

    main_plain_steps = run_plain_repeat(
        args.host, args.port, args.timeout_ms, args.main_freq, args.cycles
    )
    overall_ok &= print_plain_case("main_plain_repeat", main_plain_steps)

    if main_token is not None:
        main_select_steps = run_select_repeat(
            args.host,
            args.port,
            args.timeout_ms,
            main_token,
            args.main_freq,
            args.cycles,
        )
        overall_ok &= print_select_case("main_select_repeat", main_select_steps)

    if sub_token is not None:
        sub_select_steps = run_select_repeat(
            args.host,
            args.port,
            args.timeout_ms,
            sub_token,
            args.sub_freq,
            args.cycles,
        )
        overall_ok &= print_select_case("sub_select_repeat", sub_select_steps)

        sub_plain_steps = run_plain_after_select(
            args.host,
            args.port,
            args.timeout_ms,
            sub_token,
            args.sub_freq,
            args.cycles,
        )
        overall_ok &= print_select_case(
            "sub_plain_after_select_repeat", sub_plain_steps
        )
    else:
        overall_ok = False

    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
