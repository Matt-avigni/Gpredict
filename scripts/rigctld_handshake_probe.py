#!/usr/bin/env python3
import argparse
import sys

from rigctld_diag import (
    MAIN_VFO_TOKENS,
    SUB_VFO_TOKENS,
    RigctldConnection,
    format_result,
    parse_vfo_candidates,
    probe_plain_set_once,
    probe_token_once,
)


def print_dump_state(host, port, timeout_ms):
    with RigctldConnection(host, port, timeout_ms) as conn:
        result = conn.request_blank_terminated("\\dump_state", timeout_ms)

    print("== dump_state ==")
    if not result["ok"]:
        print("status: timeout")
        return None

    for line in result["lines"]:
        print(line)
    print("elapsed_ms=%.1f" % result["elapsed_ms"])
    return result["lines"]


def probe_role(host, port, timeout_ms, label, tokens, freq_hz):
    results = []
    print("")
    print("== %s token matrix ==" % label)
    for token in tokens:
        select_result, freq_result = probe_token_once(
            host, port, timeout_ms, token, freq_hz
        )
        freq_status = "-"
        if freq_result is not None:
            freq_status = format_result(freq_result)
        print(
            "token=%-9s select=%-10s set=%-10s select_ms=%6.1f%s"
            % (
                token,
                format_result(select_result),
                freq_status,
                select_result["elapsed_ms"],
                "" if freq_result is None else " set_ms=%6.1f" % freq_result["elapsed_ms"],
            )
        )
        results.append(
            {
                "token": token,
                "select": select_result,
                "set": freq_result,
            }
        )
    return results


def first_working_token(results):
    for result in results:
        if result["select"]["ok"] and result["set"] is not None and result["set"]["ok"]:
            return result["token"]
    for result in results:
        if result["select"]["ok"]:
            return result["token"]
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Raw rigctld handshake probe for Main/Sub VFO selection."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4532)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--main-freq", type=int, default=437800000)
    parser.add_argument("--sub-freq", type=int, default=145990000)
    args = parser.parse_args()

    dump_state_lines = print_dump_state(args.host, args.port, args.timeout_ms)
    if dump_state_lines is None:
        return 2

    candidates = parse_vfo_candidates(dump_state_lines)
    if candidates:
        print("")
        print("vfo_candidates=%s" % ", ".join(candidates))

    main_plain = probe_plain_set_once(
        args.host, args.port, args.timeout_ms, args.main_freq
    )
    print("")
    print("== plain F probe ==")
    print(
        "plain_main_set=%s elapsed_ms=%.1f"
        % (format_result(main_plain), main_plain["elapsed_ms"])
    )

    main_results = probe_role(
        args.host, args.port, args.timeout_ms, "MAIN", MAIN_VFO_TOKENS, args.main_freq
    )
    sub_results = probe_role(
        args.host, args.port, args.timeout_ms, "SUB", SUB_VFO_TOKENS, args.sub_freq
    )

    main_token = first_working_token(main_results)
    sub_token = first_working_token(sub_results)
    explicit_main_ok = any(
        result["select"]["ok"] and result["set"] is not None and result["set"]["ok"]
        for result in main_results
    )
    explicit_sub_ok = any(
        result["select"]["ok"] and result["set"] is not None and result["set"]["ok"]
        for result in sub_results
    )

    print("")
    print("== summary ==")
    print("main_plain_set=%s" % format_result(main_plain))
    print("main_token=%s" % (main_token or "(none)"))
    print("sub_token=%s" % (sub_token or "(none)"))
    print("explicit_main_ok=%d" % (1 if explicit_main_ok else 0))
    print("explicit_sub_ok=%d" % (1 if explicit_sub_ok else 0))

    if explicit_main_ok and explicit_sub_ok:
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
