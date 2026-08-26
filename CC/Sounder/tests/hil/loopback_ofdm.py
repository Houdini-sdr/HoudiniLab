#!/usr/bin/env python3
"""
loopback_ofdm.py -- self-contained OFDM closure test over one board's DAC_B -> ADC_D
loopback cable (default .22).  Reproduces the whole sounder-style signal chain in one
script: build an app-rate frame in replay RAM, loop it, receive it, and run a full
receiver (GOLD beacon sync -> fine CFO from the two identical pilots -> LTS channel
estimate -> zero-forcing equalize -> QPSK constellation + EVM).

Frame (app rate 122.88, in replay RAM, looped):
  [128 GOLD beacon][160 pilot = 2x80 LTS syms][NDATA x 80 QPSK data syms][zero pad]

A --selftest mode runs the SAME receiver on a simulated channel (numpy only, no radio)
to prove the receiver itself is correct before trusting any hardware verdict.

Run on the DGX (after: source ~/houdini_test/bin/activate ; export the lib/plugin paths):
    python3 loopback_ofdm.py                 # hardware, app-rate replay (strong beacon)
    python3 loopback_ofdm.py --rate max      # 8x-upsampled DAC-rate replay
    python3 loopback_ofdm.py --selftest      # offline receiver self-test (no radio)
    python3 loopback_ofdm.py --board 168.6.244.21   # (bare .21 RX may yield 0 samples)

Interpreting the result:
  * clean chain  -> tight QPSK: low data-aided EVM, channel adjacent-phase autocorr high,
                    small |H| spread.
  * broken chain -> ring: data-aided EVM ~140%, channel adjacent-phase autocorr ~0
                    (per-subcarrier RANDOM), deep |H| comb.  The beacon still syncs
                    (wideband) while the narrowband OFDM subcarriers are scrambled -- the
                    RFDC/RF-path issue that blocks OFDM equalization even single-board.
Free the boards first (pkill -9 -f build/sounder).  DAC_B=TX ch0, ADC_D=RX ch0 on .22.
"""
import argparse
import os
import sys

import numpy as np

RATE = 122.88e6
N, CP, SYM, RAM = 64, 16, 80, 4096

# ---- 802.11 Long Training Symbol, DC-centered; 52 active subcarriers ----
_LTS_NEG = [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1]
_LTS_POS = [1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1]
LTS = np.zeros(N, complex); LTS[6:32] = _LTS_NEG; LTS[33:59] = _LTS_POS
ACT = np.where(np.abs(LTS) > 0)[0]                 # 52 active bins (DC-centered idx)
PIL = np.array([25, 11, 39, 53])                   # pilot subcarriers -7,-21,+7,+21
DAT = np.array([k for k in ACT if k not in PIL])   # 48 data subcarriers


def ofdm(Xdc):
    body = np.fft.ifft(np.fft.ifftshift(Xdc))
    return np.concatenate([body[-CP:], body])      # 80-sample symbol (CP + body)


def demod(sym):
    return np.fft.fftshift(np.fft.fft(sym[CP:CP + N]))   # DC-centered spectrum of the body


def gold128():
    """128-sample GOLD beacon from a degree-7 preferred pair (x^7+x^3+1, x^7+x^3+x^2+x+1)."""
    def lfsr(taps):
        reg = [1] * 7; out = []
        for _ in range(127):
            out.append(reg[6])
            fb = 0
            for t in taps:
                fb ^= reg[t - 1]
            reg = [fb] + reg[:6]
        return np.array(out)
    g = (lfsr([7, 3]) ^ lfsr([7, 3, 2, 1])).astype(float) * (-2) + 1   # {0,1}->{+1,-1}
    return np.concatenate([g, g[:1]]).astype(complex)                  # 128 samples


def build_frame(ndata, seed=0xBEEF):
    """[beacon][2x LTS pilot][ndata x QPSK data]; returns (frame, beacon, tx_data_syms)."""
    rng = np.random.default_rng(seed)
    beacon = gold128()
    pil = ofdm(LTS); pilot = np.concatenate([pil, pil])       # two IDENTICAL LTS symbols
    tx, data = [], []
    for _ in range(ndata):
        Xdc = np.zeros(N, complex)
        q = ((rng.integers(0, 2, len(DAT)) * 2 - 1) +
             1j * (rng.integers(0, 2, len(DAT)) * 2 - 1)) / np.sqrt(2)
        Xdc[DAT] = q; Xdc[PIL] = 1.0                          # QPSK data + BPSK pilots
        tx.append(q); data.append(ofdm(Xdc))
    frame = np.concatenate([beacon, pilot, np.concatenate(data)])
    return frame / np.max(np.abs(frame)) * 0.5, beacon, np.array(tx)


def upsample_periodic(sig, factor):
    """Exact 8x interpolation of a LOOPED (periodic) frame -> no boundary ringing."""
    n = len(sig); X = np.fft.fft(sig); Xu = np.zeros(n * factor, complex)
    Xu[:n // 2] = X[:n // 2]; Xu[-(n - n // 2):] = X[n // 2:]
    return np.fft.ifft(Xu) * factor


def _xcorr(sig, tmpl):
    n = 1 << int(np.ceil(np.log2(len(sig))))
    c = np.fft.ifft(np.fft.fft(sig, n) * np.conj(np.fft.fft(tmpl, n)))
    return c[:len(sig) - len(tmpl) + 1]


def receive(x, beacon, tx_syms, ndata, period):
    """Full receiver: beacon sync -> CFO -> channel est -> ZF equalize.  Prints metrics."""
    x = x / (np.sqrt(np.mean(np.abs(x) ** 2)) + 1e-12)
    siglen = 128 + 160 + ndata * SYM
    bnorm = beacon / np.linalg.norm(beacon)

    # coarse frequency search for the beacon (app-rate replay shifts the band a few MHz)
    xs = x[:1 << 18]; nn = np.arange(len(xs)); best = None
    for f in np.arange(-16e6, 16.01e6, 0.25e6):
        c = np.abs(_xcorr(xs * np.exp(-2j * np.pi * f * nn / RATE), bnorm))
        pk = c.max() / (np.median(c) + 1e-12)
        if best is None or pk > best[0]:
            best = (pk, f)
    pk, f0 = best
    print("beacon: coarse freq offset %.2f MHz, peak/median %.0f" % (f0 / 1e6, pk))
    xd = x * np.exp(-2j * np.pi * f0 * np.arange(len(x)) / RATE)

    # find every frame start
    c = np.abs(_xcorr(xd, bnorm)); thr = 0.4 * c.max(); peaks = []; i = 0
    while i < len(c):
        if c[i] > thr:
            j = i + int(np.argmax(c[i:i + 200])); peaks.append(j); i = j + period - 200
        else:
            i += 1
    print("found %d beacon frames" % len(peaks))

    allX, evms, cfos = [], [], []; H = np.zeros(N, complex)
    for s in peaks:
        if s + siglen > len(xd):
            break
        fr = xd[s:s + siglen]
        # fine CFO from the two identical pilot symbols (bodies SYM apart)
        b1 = fr[128 + CP:128 + CP + N]; b2 = fr[128 + SYM + CP:128 + SYM + CP + N]
        cfo = np.angle(np.vdot(b1, b2)) / SYM
        cfos.append(cfo * RATE / (2 * np.pi))
        fr = fr * np.exp(-1j * cfo * np.arange(len(fr)))
        # channel estimate from the two pilot LTS symbols
        H = np.zeros(N, complex)
        for p in (128, 128 + SYM):
            H[ACT] += demod(fr[p:p + SYM])[ACT] * np.conj(LTS[ACT])
        H[ACT] /= 2
        # equalize each data symbol
        for k in range(ndata):
            Y = demod(fr[288 + k * SYM:288 + (k + 1) * SYM])
            Xe = np.zeros(N, complex); Xe[ACT] = Y[ACT] / H[ACT]
            Xe *= np.exp(-1j * np.angle(np.sum(Xe[PIL])))       # common-phase from known pilots
            r = tx_syms[k]
            sc = np.sqrt(np.mean(np.abs(r) ** 2) / (np.mean(np.abs(Xe[DAT]) ** 2) + 1e-12))
            allX.extend((Xe[DAT] * sc).tolist())
            evms.append(np.sqrt(np.mean(np.abs(Xe[DAT] * sc - r) ** 2) / np.mean(np.abs(r) ** 2)))

    if not evms:
        print("no frames decoded"); return None
    p = np.array(allX); Ha = H[ACT]
    zz = Ha / np.abs(Ha); dphi = np.angle(zz[1:] * np.conj(zz[:-1]))
    ac = np.corrcoef(dphi[1:], dphi[:-1])[0, 1] if len(dphi) > 2 else 0.0
    evm = 100 * np.mean(evms)
    pn = p / np.sqrt(np.mean(np.abs(p) ** 2))
    ideal = (np.sign(pn.real) + 1j * np.sign(pn.imag)) / np.sqrt(2)
    blind = 100 * np.sqrt(np.mean(np.abs(pn - ideal) ** 2))
    hspread = 20 * np.log10(np.max(np.abs(Ha)) / np.min(np.abs(Ha)))
    print("frames %d, data symbols %d" % (len(peaks), len(evms)))
    print("CFO estimate: mean %.0f Hz (std %.0f)" % (np.mean(cfos), np.std(cfos)))
    print("channel |H| spread: %.1f dB" % hspread)
    print("channel adjacent-phase autocorr: %.2f  (>0.5 smooth/clean, ~0 random/scrambled)" % ac)
    print("DATA-AIDED EVM vs known TX: %.1f%%" % evm)
    print("blind nearest-QPSK EVM: %.1f%%" % blind)
    return {"evm": evm, "blind_evm": blind, "autocorr": ac, "hspread": hspread,
            "beacon_pk": pk, "cfo": float(np.mean(cfos)),
            "sc": (ACT - N // 2).tolist(),
            "mag_db": (20 * np.log10(np.abs(Ha) / np.max(np.abs(Ha)))).tolist(),
            "phase_deg": np.degrees(np.angle(Ha)).tolist(),
            "constellation": p}


HTML_BODY = """<style>
 :root{--bg:#080b0c;--screen:#0d1315;--bezel:#1b2528;--grid:#132020;--axis:#33474b;
  --ink:#c9d4d1;--muted:#6d7c78;--teal:#3fdca6;--amber:#f0a94e;--coral:#ff5d6c;
  --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,Consolas,monospace;
  --sans:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);-webkit-font-smoothing:antialiased}
 .wrap{max-width:1060px;margin:0 auto;padding:28px 20px 44px}
 .eyebrow{font-family:var(--mono);font-size:11px;letter-spacing:.18em;text-transform:uppercase;color:var(--teal);margin:0 0 8px}
 h1{font-size:22px;font-weight:650;letter-spacing:-.01em;margin:0 0 5px;text-wrap:balance}
 .sub{font-family:var(--mono);color:var(--muted);font-size:12px;margin:0 0 20px;word-break:break-word}
 .verdict{display:flex;gap:11px;align-items:flex-start;padding:12px 15px;border-radius:6px;font-size:13.5px;line-height:1.45;margin:0 0 18px;border:1px solid}
 .verdict .dot{width:9px;height:9px;border-radius:50%;flex:0 0 auto;margin-top:5px}
 .bad{background:#180e10;border-color:#45202a;color:#ff9aa4}.bad .dot{background:var(--coral);box-shadow:0 0 9px var(--coral)}
 .good{background:#0b1d16;border-color:#1c4a38;color:#7fe6bd}.good .dot{background:var(--teal);box-shadow:0 0 9px var(--teal)}
 .readout{display:grid;grid-template-columns:repeat(6,1fr);gap:1px;background:var(--bezel);border:1px solid var(--bezel);border-radius:8px;overflow:hidden;margin:0 0 20px}
 .cell{background:var(--screen);padding:11px 13px}
 .cell .k{font-family:var(--mono);font-size:9.5px;letter-spacing:.09em;text-transform:uppercase;color:var(--muted)}
 .cell .val{font-family:var(--mono);font-size:19px;font-variant-numeric:tabular-nums;margin-top:4px}
 .panels{display:grid;grid-template-columns:1fr 1fr;gap:14px}
 .scope{background:var(--screen);border:1px solid var(--bezel);border-radius:10px;padding:12px 12px 8px}
 .scope h2{font-family:var(--mono);font-size:11px;font-weight:600;letter-spacing:.07em;text-transform:uppercase;color:var(--muted);margin:0 0 6px}
 .scope.full{grid-column:1/-1;max-width:440px;margin:0 auto}
 canvas{width:100%;height:auto;display:block}
 @media(max-width:700px){.panels{grid-template-columns:1fr}.readout{grid-template-columns:repeat(3,1fr)}}
</style>
<div class="wrap">
 <p class="eyebrow">Hardware-in-the-loop &middot; single-board DAC_B &rarr; ADC_D</p>
 <h1>OFDM loopback &mdash; channel &amp; constellation</h1>
 <p class="sub" id="sub"></p>
 <div class="verdict" id="verdict"><span class="dot"></span><span id="vtext"></span></div>
 <div class="readout" id="readout"></div>
 <div class="panels">
  <div class="scope"><h2>Channel |H| (dB) &middot; 52 subcarriers</h2><canvas id="mag" width="500" height="250"></canvas></div>
  <div class="scope"><h2>Channel phase (deg)</h2><canvas id="phase" width="500" height="250"></canvas></div>
  <div class="scope full"><h2>Equalized constellation</h2><canvas id="const" width="440" height="440"></canvas></div>
 </div>
<script>
const D=__DATA__,M=D.meta;
const C=k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim();
document.getElementById('sub').textContent=D.title;
document.getElementById('readout').innerHTML=[['EVM data-aided',M.evm+'%'],['EVM blind',M.blind+'%'],
 ['phase autocorr',M.ac],['|H| spread',M.hspread+' dB'],['beacon pk/med',M.beacon],['CFO',M.cfo+' Hz']]
 .map(c=>`<div class="cell"><div class="k">${c[0]}</div><div class="val">${c[1]}</div></div>`).join('');
const clean=M.evm<20&&M.ac>0.4;
document.getElementById('verdict').className='verdict '+(clean?'good':'bad');
document.getElementById('vtext').textContent=clean
 ?'Chain closes \\u2014 tight QPSK clusters on a smooth channel.'
 :'Scrambled \\u2014 ring constellation and a per-subcarrier-random channel (deep |H| comb), while the wideband beacon still syncs. The corruption is in the RF/RFDC path, not the receiver.';
function trace(id,xs,ys,yr,yt,col,dots){
 const c=document.getElementById(id),x=c.getContext('2d'),W=c.width,H=c.height,mL=42,mB=22,mT=8,mR=10,xr=[-30,30];
 const PX=v=>mL+(v-xr[0])/(xr[1]-xr[0])*(W-mL-mR),PY=v=>H-mB-(v-yr[0])/(yr[1]-yr[0])*(H-mB-mT);
 x.clearRect(0,0,W,H);x.font='9px '+C('--mono');x.lineWidth=1;
 yt.forEach(t=>{const y=PY(t);x.strokeStyle=C('--grid');x.beginPath();x.moveTo(mL,y);x.lineTo(W-mR,y);x.stroke();x.fillStyle=C('--muted');x.textAlign='right';x.fillText(t,mL-5,y+3);});
 [-26,-13,0,13,26].forEach(t=>{const p=PX(t);x.strokeStyle=C('--grid');x.beginPath();x.moveTo(p,mT);x.lineTo(p,H-mB);x.stroke();x.fillStyle=C('--muted');x.textAlign='center';x.fillText(t,p,H-mB+13);});
 x.strokeStyle=C('--axis');x.beginPath();x.moveTo(mL,mT);x.lineTo(mL,H-mB);x.lineTo(W-mR,H-mB);x.stroke();
 x.shadowColor=col;x.shadowBlur=6;
 if(!dots){x.strokeStyle=col;x.lineWidth=1.5;x.beginPath();xs.forEach((v,i)=>{const p=PX(v),q=PY(ys[i]);i?x.lineTo(p,q):x.moveTo(p,q);});x.stroke();}
 x.fillStyle=col;xs.forEach((v,i)=>{x.beginPath();x.arc(PX(v),PY(ys[i]),dots?2:1.5,0,7);x.fill();});x.shadowBlur=0;}
const mn=Math.min(-45,Math.min.apply(null,D.mag)-2);
trace('mag',D.sc,D.mag,[mn,3],[0,-10,-20,-30,-40],C('--teal'),false);
trace('phase',D.sc,D.phase,[-180,180],[180,90,0,-90,-180],C('--amber'),true);
(function(){const c=document.getElementById('const'),x=c.getContext('2d'),W=c.width,H=c.height,R=1.9;
 const PX=v=>W/2+v/R*(W/2-18),PY=v=>H/2-v/R*(H/2-18);x.clearRect(0,0,W,H);
 x.strokeStyle=C('--grid');x.lineWidth=1;x.beginPath();x.moveTo(PX(0),10);x.lineTo(PX(0),H-10);x.moveTo(10,PY(0));x.lineTo(W-10,PY(0));x.stroke();
 x.beginPath();x.arc(PX(0),PY(0),(1/R)*(W/2-18),0,7);x.stroke();
 x.shadowColor=C('--teal');x.shadowBlur=5;x.fillStyle='rgba(63,220,166,.42)';
 D.pts.forEach(p=>{x.beginPath();x.arc(PX(p[0]),PY(p[1]),1.5,0,7);x.fill();});x.shadowBlur=0;
 const q=0.7071;x.strokeStyle=C('--coral');x.lineWidth=2;x.shadowColor=C('--coral');x.shadowBlur=8;
 [[q,q],[-q,q],[q,-q],[-q,-q]].forEach(p=>{x.beginPath();x.arc(PX(p[0]),PY(p[1]),7,0,7);x.stroke();});x.shadowBlur=0;
 x.fillStyle=C('--coral');x.font='10px '+C('--mono');x.textAlign='left';x.fillText('rx',14,20);
 x.fillStyle=C('--teal');x.fillText('\\u25cf ideal QPSK',14,H-14);})();
</script>
</div>"""

HTML_TEMPLATE = ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
                 '<meta name="viewport" content="width=device-width,initial-scale=1">'
                 '<title>loopback_ofdm</title></head><body>' + HTML_BODY + "</body></html>")


def write_html(path, r, title):
    import json
    p = r["constellation"]
    pn = p / (np.sqrt(np.mean(np.abs(p) ** 2)) + 1e-12)
    if len(pn) > 3000:
        pn = pn[np.linspace(0, len(pn) - 1, 3000).astype(int)]
    data = {"title": title, "sc": r["sc"],
            "mag": [round(v, 2) for v in r["mag_db"]],
            "phase": [round(v, 1) for v in r["phase_deg"]],
            "pts": [[round(float(z.real), 4), round(float(z.imag), 4)] for z in pn],
            "meta": {"evm": round(r["evm"], 1), "blind": round(r["blind_evm"], 1),
                     "ac": round(r["autocorr"], 2), "hspread": round(r["hspread"], 1),
                     "beacon": int(r["beacon_pk"]), "cfo": int(r["cfo"])}}
    with open(path, "w") as f:
        f.write(HTML_TEMPLATE.replace("__DATA__", json.dumps(data)))
    print("wrote visualization -> %s  (open in a browser)" % path)


def run_selftest(html=None):
    """Run the receiver on a simulated channel (no radio).  EVM must be ~0 on an ideal
    channel and small on mild in-CP multipath, proving the receiver is correct."""
    ndata = 10
    frame, beacon, tx_syms = build_frame(ndata)
    ram = np.zeros(RAM, complex); ram[:len(frame)] = frame
    cap = np.tile(ram, 20)
    print("=== SELFTEST 1/2: ideal channel (EVM must be ~0) ===")
    r1 = receive(cap.copy(), beacon, tx_syms, ndata, RAM)
    print("\n=== SELFTEST 2/2: mild in-CP multipath + CFO + noise ===")
    rng = np.random.default_rng(1)
    h = np.zeros(8, complex); h[0] = 1.0; h[3] = 0.3 * np.exp(1j * 0.7); h[6] = 0.15 * np.exp(-1j * 1.1)
    cap2 = np.convolve(cap, h)[:len(cap)]
    cap2 = cap2 * np.exp(2j * np.pi * 1234.0 * np.arange(len(cap2)) / RATE)
    cap2 = cap2 + 0.005 * (rng.standard_normal(len(cap2)) + 1j * rng.standard_normal(len(cap2)))
    r2 = receive(cap2, beacon, tx_syms, ndata, RAM)
    ok = r1 and r2 and r1["evm"] < 1.0 and r2["evm"] < 40.0
    print("\nSELFTEST %s (ideal %.2f%% < 1%%, multipath %.1f%% < 40%%)" %
          ("PASS" if ok else "FAIL", r1["evm"] if r1 else 99, r2["evm"] if r2 else 99))
    if html and r2:
        write_html(html, r2, "SELFTEST — simulated mild-multipath channel (receiver reference)")
    return 0 if ok else 1


def run_hardware(a):
    _EX = os.environ.get("HOUDINI_EXAMPLES", os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
    if _EX not in sys.path:
        sys.path.insert(0, _EX)
    import SoapySDR  # noqa: E402
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX  # noqa: E402
    import houdini_setup as hs  # noqa: E402

    ndata = a.ndata if a.ndata is not None else (10 if a.rate == "app" else 2)
    frame, beacon, tx_syms = build_frame(ndata)
    if a.rate == "app":
        ram = np.zeros(RAM, complex); ram[:len(frame)] = frame; period = RAM
    else:
        fr = np.zeros(RAM // 8, complex); fr[:len(frame)] = frame
        ram = upsample_periodic(fr, 8); period = len(fr)

    ram = ram * a.tx_scale
    pk = np.max(np.abs(np.concatenate([ram.real, ram.imag])))
    print("TX drive scale %.3f -> peak %.3f of full scale%s"
          % (a.tx_scale, pk, "  [CLIPS int16!]" if pk > 1.0 else ""))
    iq = np.empty(2 * len(ram), np.int16)
    iq[0::2] = np.round(np.clip(ram.real, -1, 1) * 32767)
    iq[1::2] = np.round(np.clip(ram.imag, -1, 1) * 32767)
    cs16 = np.ascontiguousarray(iq).view(np.int32)

    txc = hs.open_device(node=a.board, ch=a.tx_ch, verbose=False)
    rxc = hs.open_device(node=a.board, ch=a.rx_ch, verbose=False)
    txd, rxd = txc["sdr"], rxc["sdr"]
    native, dtype = rxc["native_fmt"], rxc["dtype"]
    tx_rate = max(txd.listSampleRates(SOAPY_SDR_TX, a.tx_ch)) if a.rate == "max" else RATE
    txd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, tx_rate)
    txd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco * 1e6)
    print("rate=%s ndata=%d TX replay %.2f MSPS, DAC_B ch%d -> ADC_D ch%d @ NCO %.0f MHz" %
          (a.rate, ndata, float(txd.getSampleRate(SOAPY_SDR_TX, a.tx_ch)) / 1e6,
           a.tx_ch, a.rx_ch, a.nco))
    tx = txd.setupStream(SOAPY_SDR_TX, "CS16", [a.tx_ch], {"tx_mode": "replay"})
    txd.writeStream(tx, [cs16], len(ram), 0, 0); txd.activateStream(tx)
    try:
        rxd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, RATE)
        rxd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
        buf, _ = hs.capture_rx(rxd, a.rx_ch, native, dtype, duration_sec=a.secs,
                               capture_bytes=16 * 1024 * 1024, live_print=False)
    finally:
        txd.deactivateStream(tx); txd.closeStream(tx)
    # production RX decode: native CS16 is [I0,Q0,I1,Q1,...] -> even + j*odd
    # (the earlier hand-rolled L0+j*L2 mis-paired lanes and fabricated the "ring").
    # conj(): the RX DDC returns a spectrally-INVERTED baseband (a +f tone comes back
    # at -f, verified via analyze_rx_interleave); conjugate to un-mirror the subcarriers.
    x = np.conj(hs.iq_from_cs16(buf)).astype(np.complex128)
    if len(x) < (1 << 17):
        print("captured only %d samples -- bare RX on this board may not egress (try .22)" % len(x))
        return 1
    r = receive(x, beacon, tx_syms, ndata, period)
    if r is not None:
        np.save("/tmp/loopback_constellation.npy", r["constellation"])
        print("saved constellation -> /tmp/loopback_constellation.npy")
        if a.html:
            write_html(a.html, r, "%s replay, DAC_B ch%d -> ADC_D ch%d @ %.0f MHz on %s" %
                       (a.rate, a.tx_ch, a.rx_ch, a.nco, a.board))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="offline receiver validation (no radio)")
    ap.add_argument("--board", default="168.6.244.22", help="board IP (has the DAC_B->ADC_D cable)")
    ap.add_argument("--tx-ch", type=int, default=0, help="DAC_B = TX ch0")
    ap.add_argument("--rx-ch", type=int, default=0, help="ADC_D = RX ch0 on .22")
    ap.add_argument("--nco", type=float, default=500.0, help="TX=RX NCO in MHz (Zone 1)")
    ap.add_argument("--rate", default="app", choices=["app", "max"],
                    help="app: 122.88 replay (strong beacon); max: 8x-upsampled DAC-rate replay")
    ap.add_argument("--ndata", type=int, default=None, help="number of QPSK data symbols")
    ap.add_argument("--secs", type=float, default=0.4, help="RX capture duration")
    ap.add_argument("--tx-scale", type=float, default=1.0,
                    help="extra TX drive scale (1.0 = frame's default 0.5 peak); "
                         "sweep down to test DAC/ADC overdrive/clipping")
    ap.add_argument("--html", nargs="?", const="/tmp/loopback_ofdm.html", default=None,
                    help="write a self-contained HTML visualization (channel + constellation); "
                         "optional path, default /tmp/loopback_ofdm.html")
    a = ap.parse_args()
    return run_selftest(a.html) if a.selftest else run_hardware(a)


if __name__ == "__main__":
    sys.exit(main())
