#!/bin/bash
# Sample the rig host through the hourly esm-cache / PackageKit firing at
# hh:26:39: per-CPU load and per-process CPU (with the core each ran on) at
# 1 s from <start> for <secs>, and the processes behind the firing (gdbus,
# apt, PackageKit, esm) with their parents, every second for 40 s from 23:26:35.
# usage: w6_mon.sh <start 'YYYY-mm-dd HH:MM:SS' UTC> <secs> <outdir>
set -u
start=$1 secs=$2 out=$3
mkdir -p "$out"
sleep $(( $(date -u -d "$start" +%s) - $(date +%s) ))
date -u +"mon start %T" > "$out/mon.txt"
LC_ALL=C mpstat -P ALL 1 "$secs" > "$out/mpstat.txt" 2>&1 &
LC_ALL=C pidstat -u 1 "$secs" > "$out/pidstat.txt" 2>&1 &
sleep $(( $(date -u -d "2026-10-19 23:26:35" +%s) - $(date +%s) ))
for i in $(seq 40); do
  echo "## $(date -u +%T.%N | cut -c1-12)"
  ps -eo pid,ppid,user,psr,pcpu,lstart,args --sort=start_time | grep -iE "[g]dbus|[a]pt|[p]ackagekit|[e]sm|[u]buntu-advantage|[u]a_|[p]ro " | cut -c1-220
  sleep 1
done > "$out/ps.txt" 2>&1
wait
date -u +"mon end %T" >> "$out/mon.txt"
