"""Automatic bus transfer (ABT) controller -- reference simulation.

Independent Python implementation of the same plant and controller logic that
the Simulink/Simscape model (matlab/) implements.  Because this file has no
toolbox dependency it is the executable "ground truth" for the repo: the
committed plots in results/ are produced from it.

System modelled
---------------
A single 28 VDC bus fed by three sources through contactors:

    MAIN gen (28.5 V, 25 mOhm) --K_MAIN--+
    APU  gen (28.2 V, 35 mOhm) --K_APU---+--- BUS --- C_bus --- loads
    BATTERY  (25.6 V, 40 mOhm) --K_BATT--+

Sources are Thevenin equivalents (EMF behind a series resistance).  Generator
EMF responds to on/off commands through a first-order lag (field / regulator
time constant), so a "failure" is a collapse, not a step.  Contactors are
ideal switches with a fixed actuation delay.  The bus carries a hold-up
capacitance and a resistive load (plus an optional step load).

Controller: priority selection MAIN > APU > BATT, per-source undervoltage
detection with hysteresis and qualification time, break-before-make transfer
with an enforced dead time.  See README.md for the rationale behind every
number.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Source indices / identifiers
MAIN, APU, BATT = 0, 1, 2
NONE = -1
SOURCE_NAMES = {MAIN: "MAIN", APU: "APU", BATT: "BATT", NONE: "NONE"}

# Controller phases
CONNECTED, DEAD_TIME, OVERLAP = 0, 1, 2


@dataclass
class Params:
    """All plant and controller parameters (single source of truth for Python;
    matlab/abt_params.m mirrors these values)."""

    # --- Sources: EMF setpoints [V] and series (source) resistance [ohm] ---
    # MAIN and APU are regulated to slightly different setpoints, as two real
    # regulators would be; the battery is a Thevenin equivalent of a charged
    # 24 V nominal aircraft battery (constant EMF is valid on the seconds
    # timescale of a transfer study -- see README).
    emf_set: tuple = (28.5, 28.2, 25.6)
    r_src: tuple = (0.025, 0.035, 0.040)
    tau_emf: float = 0.050        # generator field/regulator collapse & recovery lag [s]

    # --- Bus & loads ---
    c_bus: float = 0.020          # bus hold-up capacitance [F]
    r_load: float = 1.4           # base load: ~20 A at 28 V [ohm]
    r_step: float = 2.0           # optional step load: ~+14 A [ohm]

    # --- Contactors ---
    t_contactor: float = 0.020    # actuation delay, open and close [s]

    # --- Controller ---
    ts: float = 1.0e-3            # controller sample time [s]
    v_fail: tuple = (18.0, 18.0, 18.0)   # undervoltage (fail) threshold per source [V]
    v_ok: tuple = (25.0, 25.0, 22.0)     # recovery (healthy) threshold per source [V]
    t_fail_qual: float = 0.050    # fault must persist this long before declaring [s]
    t_recov_qual: float = 1.000   # recovery must persist this long before retransfer [s]
    t_dead: float = 0.050         # enforced break-before-make dead time [s]

    # --- Simulation ---
    dt: float = 1.0e-4            # plant integration step (backward Euler) [s]


class Contactor:
    """Ideal switch with a fixed actuation delay.

    The physical state follows the commanded state `t_delay` seconds after the
    command last changed.  If the command flips back within the delay the
    motion is abandoned (adequate for an electromechanical contactor at this
    level of fidelity)."""

    def __init__(self, t_delay: float, closed: bool = False):
        self.t_delay = t_delay
        self.closed = closed
        self._cmd = closed
        self._t_cmd = -1e9

    def step(self, cmd: bool, t: float) -> bool:
        if cmd != self._cmd:
            self._cmd = cmd
            self._t_cmd = t
        if self._cmd != self.closed and (t - self._t_cmd) >= self.t_delay:
            self.closed = self._cmd
        return self.closed


class Plant:
    """Electrical network: three Thevenin sources, contactors, capacitive bus,
    resistive load.  Bus voltage integrated with backward Euler, which is
    unconditionally stable for this linear RC network."""

    def __init__(self, p: Params):
        self.p = p
        self.emf = list(p.emf_set)            # lagged EMF states
        self.v_bus = 0.0
        self.contactors = [Contactor(p.t_contactor) for _ in range(3)]

    def step(self, t: float, emf_target: list, cmd: list, step_load_on: bool):
        p = self.p
        # Generator field / regulator lag (battery uses the same lag; its
        # target never changes so it stays at its EMF)
        for i in range(3):
            self.emf[i] += (emf_target[i] - self.emf[i]) * (p.dt / p.tau_emf)

        closed = [self.contactors[i].step(cmd[i], t) for i in range(3)]

        # Backward-Euler node equation on the bus capacitance
        g_load = 1.0 / p.r_load + (1.0 / p.r_step if step_load_on else 0.0)
        g_src = sum(1.0 / p.r_src[i] for i in range(3) if closed[i])
        i_src_sum = sum(self.emf[i] / p.r_src[i] for i in range(3) if closed[i])
        self.v_bus = (p.c_bus / p.dt * self.v_bus + i_src_sum) / (
            p.c_bus / p.dt + g_src + g_load
        )

        # Source currents (positive = source delivering to the bus; negative
        # would mean back-feed INTO the source)
        i_src = [
            (self.emf[i] - self.v_bus) / p.r_src[i] if closed[i] else 0.0
            for i in range(3)
        ]
        # Source terminal voltage, sensed between the machine and its
        # contactor: equals the bus when connected, open-circuit EMF when not.
        v_term = [self.v_bus if closed[i] else self.emf[i] for i in range(3)]
        return v_term, i_src, closed


class Controller:
    """Priority-based automatic bus transfer controller (break-before-make).

    Runs at a fixed sample time.  Per-source health is decided from the
    source-side (upstream of contactor) voltage with hysteresis and
    qualification timers; the bus is then connected to the highest-priority
    healthy source, with an enforced dead time between opening one contactor
    and closing the next."""

    def __init__(self, p: Params):
        self.p = p
        self.healthy = [True, True, True]     # assume pre-start checks passed
        self.t_fail = [0.0, 0.0, 0.0]
        self.t_recov = [0.0, 0.0, 0.0]
        self.connected = NONE
        self.phase = DEAD_TIME
        self.t_dead = p.t_dead                # pre-elapsed: close first source at t=0
        self.cmd = [False, False, False]

    def _update_health(self, v_term):
        p = self.p
        for i in range(3):
            if self.healthy[i]:
                if v_term[i] < p.v_fail[i]:
                    self.t_fail[i] += p.ts
                    if self.t_fail[i] >= p.t_fail_qual:
                        self.healthy[i] = False
                        self.t_recov[i] = 0.0
                else:
                    self.t_fail[i] = 0.0
            else:
                if v_term[i] > p.v_ok[i]:
                    self.t_recov[i] += p.ts
                    if self.t_recov[i] >= p.t_recov_qual:
                        self.healthy[i] = True
                        self.t_fail[i] = 0.0
                else:
                    self.t_recov[i] = 0.0

    def _best(self) -> int:
        for i in (MAIN, APU, BATT):           # fixed priority order
            if self.healthy[i]:
                return i
        return NONE

    def step(self, v_term):
        p = self.p
        self._update_health(v_term)
        best = self._best()

        if self.phase == CONNECTED:
            if best != self.connected:
                # Either the connected source failed, or a higher-priority
                # source re-qualified.  Break first: open everything and
                # start the dead time.
                self.cmd = [False, False, False]
                self.connected = NONE
                self.phase = DEAD_TIME
                self.t_dead = 0.0
        else:  # DEAD_TIME
            self.cmd = [False, False, False]
            self.t_dead += p.ts
            # The target is re-evaluated every sample, so a source that fails
            # during the dead time (cascading fault) is never connected.
            if self.t_dead >= p.t_dead and best != NONE:
                self.cmd = [i == best for i in range(3)]
                self.connected = best
                self.phase = CONNECTED

        return list(self.cmd), self.connected, self.phase


class NaiveMBBController(Controller):
    """Deliberately WRONG variant used only by the make-before-break demo
    scenario: it closes the incoming contactor while the outgoing one is still
    closed (paralleling the sources for `overlap` seconds).  Exists to show
    the back-feed current this causes -- see results/s5 and the README."""

    def __init__(self, p: Params, overlap: float = 0.030):
        super().__init__(p)
        self.overlap = overlap
        self._t_ovl = 0.0
        self._old = NONE
        self._target = NONE

    def step(self, v_term):
        p = self.p
        self._update_health(v_term)
        best = self._best()

        if self.phase == CONNECTED:
            if best != self.connected:
                if best == NONE or self.connected == NONE:
                    # nothing to parallel with -- fall back to plain dead-time
                    self.cmd = [False, False, False]
                    self.connected = NONE
                    self.phase = DEAD_TIME
                    self.t_dead = 0.0
                else:
                    # make before break: close the target on top of the old one
                    self._old, self._target = self.connected, best
                    self.cmd = [i in (self._old, self._target) for i in range(3)]
                    self.phase = OVERLAP
                    self._t_ovl = 0.0
        elif self.phase == OVERLAP:
            self._t_ovl += p.ts
            if self._t_ovl >= self.overlap:
                self.cmd = [i == self._target for i in range(3)]
                self.connected = self._target
                self.phase = CONNECTED
        else:  # DEAD_TIME
            self.cmd = [False, False, False]
            self.t_dead += p.ts
            if self.t_dead >= p.t_dead and best != NONE:
                self.cmd = [i == best for i in range(3)]
                self.connected = best
                self.phase = CONNECTED

        return list(self.cmd), self.connected, self.phase


@dataclass
class Result:
    t: np.ndarray
    v_bus: np.ndarray
    v_term: np.ndarray        # (n, 3)
    i_src: np.ndarray         # (n, 3)
    closed: np.ndarray        # (n, 3) physical contactor state
    connected: np.ndarray     # controller's connected-source id
    phase: np.ndarray
    events: list = field(default_factory=list)   # (t, "K_MAIN CLOSE") ...


def simulate(p: Params, t_end: float, emf_target_fn, step_load_fn=None,
             controller: Controller | None = None) -> Result:
    """Run the closed-loop simulation.

    emf_target_fn(t) -> [target EMF for MAIN, APU, BATT] lets scenarios script
    failures (collapse to 0), sags, and recoveries.
    step_load_fn(t) -> bool switches the step load."""

    plant = Plant(p)
    ctrl = controller if controller is not None else Controller(p)
    step_load_fn = step_load_fn or (lambda t: False)

    n = int(round(t_end / p.dt))
    ctrl_every = max(1, int(round(p.ts / p.dt)))

    t_arr = np.empty(n)
    v_bus = np.empty(n)
    v_term = np.empty((n, 3))
    i_src = np.empty((n, 3))
    closed_a = np.empty((n, 3), dtype=bool)
    conn_a = np.empty(n, dtype=int)
    phase_a = np.empty(n, dtype=int)
    events = []

    cmd = [False, False, False]
    prev_closed = [c.closed for c in plant.contactors]
    vt = [p.emf_set[i] for i in range(3)]

    for k in range(n):
        t = k * p.dt
        if k % ctrl_every == 0:
            cmd, connected, phase = ctrl.step(vt)
        vt, isrc, closed = plant.step(t, emf_target_fn(t), cmd, step_load_fn(t))

        for i in range(3):
            if closed[i] != prev_closed[i]:
                events.append((t, f"K_{SOURCE_NAMES[i]} {'CLOSE' if closed[i] else 'OPEN'}"))
        prev_closed = list(closed)

        t_arr[k] = t
        v_bus[k] = plant.v_bus
        v_term[k] = vt
        i_src[k] = isrc
        closed_a[k] = closed
        conn_a[k] = ctrl.connected
        phase_a[k] = ctrl.phase

    return Result(t_arr, v_bus, v_term, i_src, closed_a, conn_a, phase_a, events)


def bus_outage_stats(res: Result, v_min: float = 18.0):
    """Contiguous intervals where the bus is below v_min (after initial
    energization) -- used to report transfer outage durations."""
    below = res.v_bus < v_min
    # ignore the initial energization interval
    first_up = np.argmax(~below) if below[0] else 0
    below[:first_up] = False
    intervals = []
    start = None
    for k in range(len(below)):
        if below[k] and start is None:
            start = res.t[k]
        elif not below[k] and start is not None:
            intervals.append((start, res.t[k]))
            start = None
    if start is not None:
        intervals.append((start, res.t[-1]))
    return intervals
