#!/usr/bin/env python3
"""The data NIC's interrupt rate per core during a run: every <period> s for
<total> s, the mlx5 lines of /proc/interrupts summed per CPU, written as rates
(interrupts/s) for the chosen cores and the total. Stdlib only; run it on the
host next to the sounder, and kill it by the pid recorded at launch.
usage: irq_sampler.py <out> <period_s> <total_s> <cpu>[,<cpu>...]   e.g. 10,11,15"""
import sys, time


def mlx5_per_cpu():
    with open("/proc/interrupts") as f:
        ncpu = len(f.readline().split())
        counts = [0] * ncpu
        for line in f:
            if "mlx5" not in line:
                continue
            fields = line.split()
            for i in range(ncpu):
                if i + 1 < len(fields) and fields[i + 1].isdigit():
                    counts[i] += int(fields[i + 1])
    return counts


def main():
    out, period, total = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    cpus = [int(c) for c in sys.argv[4].split(",")]
    prev, t_prev = mlx5_per_cpu(), time.monotonic()
    with open(out, "a") as f:
        f.write("# UTC, mlx5 interrupts/s on cpu %s, then all cpus\n" % ",".join(map(str, cpus)))
    end = t_prev + total
    while time.monotonic() < end:
        time.sleep(period)
        cur, t = mlx5_per_cpu(), time.monotonic()
        dt = max(t - t_prev, 1e-9)
        rates = [(cur[i] - prev[i]) / dt for i in range(len(cur))]
        with open(out, "a") as f:
            f.write("%s %s all %.0f\n" % (time.strftime("%H:%M:%S", time.gmtime()),
                                          " ".join("cpu%d %.0f" % (c, rates[c]) for c in cpus), sum(rates)))
        prev, t_prev = cur, t


if __name__ == "__main__":
    main()
