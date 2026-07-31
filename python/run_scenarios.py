"""Run all ABT scenarios and write plots + an event log to results/.

Usage:  python3 python/run_scenarios.py
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from abt_sim import (
    APU, BATT, MAIN, NONE, SOURCE_NAMES,
    Controller, NaiveMBBController, Params, Result,
    bus_outage_stats, simulate,
)

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")

# ---------------------------------------------------------------------------
# Plot style: validated categorical palette (CVD-checked), recessive chrome.
# Sub-3:1 slots (APU aqua, BATT yellow) are relieved by direct labels.
# ---------------------------------------------------------------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
C_SRC = {MAIN: "#2a78d6", APU: "#1baf7a", BATT: "#eda100"}
C_CRIT = "#d03b3b"  # status: critical (used for the back-feed annotation only)

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 9,
    "text.color": INK,
    "axes.edgecolor": BASELINE,
    "axes.labelcolor": INK2,
    "axes.titlesize": 10,
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "legend.frameon": False,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "savefig.dpi": 150,
})


def _dead_bus_spans(res: Result):
    """Intervals where no contactor is closed (the transfer dead time)."""
    none_closed = ~res.closed.any(axis=1)
    spans, start = [], None
    for k in range(len(none_closed)):
        if none_closed[k] and start is None:
            start = res.t[k]
        elif not none_closed[k] and start is not None:
            spans.append((start, res.t[k]))
            start = None
    if start is not None:
        spans.append((start, res.t[-1]))
    return spans


def plot_scenario(res: Result, p: Params, title: str, fname: str,
                  i_ylim=(-5, 45), notes=()):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7),
                             gridspec_kw={"height_ratios": [3, 1.4, 2.2]})
    ax_v, ax_k, ax_i = axes
    fig.suptitle(title, x=0.07, ha="left", fontweight="bold", fontsize=12,
                 color=INK)

    # shade dead-time (no source connected) spans, skipping initial energization
    spans = [s for s in _dead_bus_spans(res) if s[0] > 0]
    for ax in axes:
        for (a, b) in spans:
            ax.axvspan(a, b, color=GRID, alpha=0.55, lw=0, zorder=0)

    # --- Panel 1: voltages -------------------------------------------------
    for i in (MAIN, APU, BATT):
        ax_v.plot(res.t, res.v_term[:, i], color=C_SRC[i], lw=1.4,
                  label=f"{SOURCE_NAMES[i]} terminal", zorder=3)
    ax_v.plot(res.t, res.v_bus, color=INK, lw=2.0, label="Bus", zorder=4)
    ax_v.axhline(p.v_fail[0], color=MUTED, lw=0.9, ls=(0, (4, 3)), zorder=2)
    ax_v.text(res.t[-1], p.v_fail[0] + 0.4, "undervoltage threshold (18 V)",
              ha="right", va="bottom", fontsize=7.5, color=MUTED)
    ax_v.set_ylabel("Voltage [V]")
    ax_v.set_ylim(-1.5, 32)
    ax_v.legend(loc="right", ncols=2, fontsize=8)
    if spans:
        a, b = spans[0]
        ax_v.annotate("dead time", xy=((a + b) / 2, 9), ha="center",
                      fontsize=7.5, color=INK2)

    # --- Panel 2: contactor states (one lane per contactor) ---------------
    for lane, i in enumerate((BATT, APU, MAIN)):
        y0 = lane * 1.3
        ax_k.fill_between(res.t, y0, y0 + res.closed[:, i],
                          step="post", color=C_SRC[i], lw=0, alpha=0.85)
        ax_k.text(-0.01 * res.t[-1], y0 + 0.5, f"K_{SOURCE_NAMES[i]}",
                  ha="right", va="center", fontsize=8, color=INK2)
    ax_k.set_ylim(-0.25, 3.9)
    ax_k.set_yticks([])
    ax_k.set_ylabel("Contactors\n(closed)", fontsize=8)
    ax_k.grid(False)

    # --- Panel 3: source currents ------------------------------------------
    for i in (MAIN, APU, BATT):
        ax_i.plot(res.t, res.i_src[:, i], color=C_SRC[i], lw=1.4,
                  label=f"{SOURCE_NAMES[i]}")
    ax_i.axhline(0, color=BASELINE, lw=0.9)
    ax_i.set_ylim(*i_ylim)
    ax_i.set_ylabel("Source current [A]")
    ax_i.set_xlabel("Time [s]")
    ax_i.legend(loc="upper right", ncols=3, fontsize=8)
    # annotate clipped recharge inrush peaks (closing onto a discharged bus)
    for i in (MAIN, APU, BATT):
        pk = res.i_src[:, i].max()
        if pk > i_ylim[1]:
            kpk = int(np.argmax(res.i_src[:, i]))
            ax_i.annotate(f"recharge inrush,\npeak ≈ {pk:.0f} A (clipped)",
                          xy=(res.t[kpk], i_ylim[1] - 1),
                          xytext=(res.t[kpk] + 0.04 * res.t[-1], i_ylim[1] - 14),
                          fontsize=7.5, color=INK2,
                          arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))

    for k, note in enumerate(notes):
        fig.text(0.07, 0.005 + 0.018 * (len(notes) - 1 - k), note,
                 fontsize=7.5, color=INK2)

    fig.align_ylabels(axes)
    fig.tight_layout(rect=(0, 0.02 * len(notes), 1, 0.97))
    path = os.path.join(RESULTS_DIR, fname)
    fig.savefig(path)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# Scenarios.  emf_target(t) returns the commanded EMF for [MAIN, APU, BATT];
# a generator failure is a collapse of its target to 0 (the plant applies the
# field time constant).
# ---------------------------------------------------------------------------

def sc1_single_failure(p: Params):
    """MAIN generator fails at t=1.0 s -> transfer to APU."""
    E = p.emf_set

    def emf(t):
        return [0.0 if t >= 1.0 else E[MAIN], E[APU], E[BATT]]

    return simulate(p, 3.0, emf)


def sc2_cascading(p: Params):
    """MAIN fails at t=1.0 s, APU fails at t=3.0 s -> ends on battery."""
    E = p.emf_set

    def emf(t):
        return [0.0 if t >= 1.0 else E[MAIN],
                0.0 if t >= 3.0 else E[APU],
                E[BATT]]

    return simulate(p, 5.0, emf)


def sc3_recovery(p: Params):
    """MAIN fails at 1.0 s, recovers at 3.0 s -> qualified retransfer back."""
    E = p.emf_set

    def emf(t):
        return [0.0 if 1.0 <= t < 3.0 else E[MAIN], E[APU], E[BATT]]

    return simulate(p, 6.0, emf)


def sc4_ride_through(p: Params):
    """A 30 ms sag on MAIN at t=1.0 s (shorter than the 50 ms qualification
    time) plus a +14 A load step at t=2.0 s: neither may cause a transfer."""
    E = p.emf_set

    def emf(t):
        return [0.0 if 1.0 <= t < 1.030 else E[MAIN], E[APU], E[BATT]]

    return simulate(p, 3.0, emf, step_load_fn=lambda t: t >= 2.0)


def sc5_mbb_demo(p: Params):
    """Why break-before-make: on battery (APU dead from start, MAIN fails at
    0.5 s), MAIN recovers at 2.0 s.  Run the retransfer with the proper BBM
    controller and with a naive make-before-break controller that parallels
    MAIN with the battery for 30 ms."""
    E = p.emf_set

    def emf(t):
        return [0.0 if 0.5 <= t < 2.0 else E[MAIN], 0.0, E[BATT]]

    bbm = simulate(p, 4.5, emf)
    mbb = simulate(p, 4.5, emf, controller=NaiveMBBController(p, overlap=0.030))
    return bbm, mbb


def plot_sc5(bbm: Result, mbb: Result, p: Params, fname: str):
    fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(9, 5.6))
    fig.suptitle("S5 — Why break-before-make: retransfer battery → MAIN",
                 x=0.07, ha="left", fontweight="bold", fontsize=12, color=INK)

    ax_v.plot(bbm.t, bbm.v_bus, color=INK, lw=2.0, label="Bus, break-before-make")
    ax_v.plot(mbb.t, mbb.v_bus, color=C_CRIT, lw=1.4, ls=(0, (4, 2)),
              label="Bus, naive make-before-break")
    ax_v.set_ylabel("Bus voltage [V]")
    ax_v.set_ylim(-1.5, 32)
    ax_v.legend(loc="lower right", fontsize=8)

    ax_i.plot(bbm.t, bbm.i_src[:, BATT], color=C_SRC[BATT], lw=2.0,
              label="Battery current, BBM")
    ax_i.plot(mbb.t, mbb.i_src[:, BATT], color=C_CRIT, lw=1.4, ls=(0, (4, 2)),
              label="Battery current, naive MBB")
    ax_i.axhline(0, color=BASELINE, lw=0.9)
    kmin = int(np.argmin(mbb.i_src[:, BATT]))
    ipk = mbb.i_src[kmin, BATT]
    ax_i.annotate(
        f"back-feed: {abs(ipk):.0f} A driven INTO the battery\n"
        "while paralleled with the recovered generator",
        xy=(mbb.t[kmin], max(ipk, -55)), xytext=(mbb.t[kmin] + 0.25, -45),
        fontsize=8, color=C_CRIT,
        arrowprops=dict(arrowstyle="->", color=C_CRIT, lw=1.0))
    ax_i.set_ylim(-60, 45)
    ax_i.set_ylabel("Battery current [A]")
    ax_i.set_xlabel("Time [s]")
    ax_i.legend(loc="lower left", fontsize=8)

    fig.align_ylabels([ax_v, ax_i])
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = os.path.join(RESULTS_DIR, fname)
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    p = Params()
    lines = ["# Scenario event log (Python reference simulation)", ""]

    runs = [
        ("s1_single_failure.png", "S1 — MAIN generator failure → transfer to APU",
         sc1_single_failure(p)),
        ("s2_cascading_failure.png", "S2 — Cascading failure: MAIN → APU → battery",
         sc2_cascading(p)),
        ("s3_recovery_retransfer.png", "S3 — MAIN recovery → qualified retransfer",
         sc3_recovery(p)),
        ("s4_transient_ride_through.png",
         "S4 — 30 ms sag + load step: no nuisance transfer", sc4_ride_through(p)),
    ]

    for fname, title, res in runs:
        outages = bus_outage_stats(res)
        notes = [
            "Bus outages (<18 V): " + (", ".join(
                f"{a:.3f}–{b:.3f} s ({(b - a) * 1e3:.0f} ms)" for a, b in outages)
                if outages else "none"),
        ]
        path = plot_scenario(res, p, title, fname, notes=notes)
        print(f"wrote {path}")

        lines.append(f"## {title}")
        lines.append("")
        lines.append("| t [s] | event |")
        lines.append("|-------|-------|")
        for t, ev in res.events:
            lines.append(f"| {t:.3f} | {ev} |")
        for a, b in outages:
            lines.append(f"| {a:.3f} | bus < 18 V until {b:.3f} s "
                         f"({(b - a) * 1e3:.0f} ms outage) |")
        lines.append("")

    bbm, mbb = sc5_mbb_demo(p)
    path = plot_sc5(bbm, mbb, p, "s5_make_before_break_backfeed.png")
    print(f"wrote {path}")
    ipk = mbb.i_src[:, BATT].min()
    lines.append("## S5 — Why break-before-make")
    lines.append("")
    lines.append(f"Naive 30 ms make-before-break overlap during retransfer "
                 f"drives a peak of **{abs(ipk):.0f} A into the battery** "
                 f"(negative source current = back-feed). The BBM controller "
                 f"never draws reverse current: min battery current "
                 f"{bbm.i_src[:, BATT].min():.2f} A.")
    lines.append("")

    log_path = os.path.join(RESULTS_DIR, "event_log.md")
    with open(log_path, "w") as f:
        f.write("\n".join(lines))
    print(f"wrote {log_path}")


if __name__ == "__main__":
    main()
