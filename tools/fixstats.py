#!/usr/bin/env python3
# Statistics of a console capture: the `d` stream's fix rows (rest jitter,
# axis wander, the R floor from second differences) and the EVENTS the
# stream does not carry, read from the `:load` and `l` reports bracketing a
# capture (cold starts by cause, presence toggles, frames missed, drops,
# restarts) as rates per minute. `python3 tools/fixstats.py rest.txt
# rest_end.txt` reads the files in order as one capture; `--selftest` is the
# one runnable check. The bench protocol is docs/knobs.md section 8.
import sys, statistics as st, math, re

cols="fix,t_ms,present,valid,x,y,z,tip_x,tip_y,tip_z,axis_x,axis_y,axis_z,tilt_deg,strength,residual_mT,fit_us,misfit,seen_by,sigma_x,sigma_y,sigma_z,faint_by,raw_x,raw_y,raw_z,track_state,track_x,track_y,track_z,track_sx,track_sy,track_sz,cursor_x,cursor_y,cursor_z,gate,dropped".split(',')
ix={c:i for i,c in enumerate(cols)}
GAP_MS=60          # a hole in the stream longer than this is an absent / too-few-noticing spell
ROUGH_MM=5.0       # MAGLOC_ROUGH_ABOVE_MM: a bar wider than this is a rough fix

LOAD_RE=[
    ('cold_starts', r'(\d+) cold starts since boot'),
    ('absent', r'(\d+) times nothing present'), ('few', r'(\d+) too few noticing'), ('coasted', r'(\d+) the track coasted out'),
    ('rejected', r'(\d+) (?:far )?cold starts rejected'),
    ('presence_toggles', r'presence toggled (\d+) times'), ('frames_missed', r'(\d+) frames missed while tracking'),
    ('max_peak_level', r'the strongest reading since the last look ([0-9.]+)'),
]
TRACK_RE=r'(\d+) fixes taken, (\d+) dropped, (\d+) restarts'
PREFIX_RE=re.compile(r'^\d+\.\d+\s+(.*)$')

def parse(lines):
    """The fix rows (floats, `fix` column dropped), the :load blocks (dicts) and the track counters, in file order."""
    rows=[]; loads=[]; tracks=[]; block=None
    for line in lines:
        line=line.rstrip('\r\n').lstrip()
        m=PREFIX_RE.match(line)   # tools/readport.py puts the host time in front of every line
        if m: line=m.group(1)
        if line.startswith('fix,'):
            p=line.split(',')
            if len(p)<len(cols): continue
            try: rows.append([0]+[float(v) for v in p[1:len(cols)]])
            except ValueError: continue
        elif line.startswith('load{'):
            block={}
        elif block is not None:
            for key,rx in LOAD_RE:
                m=re.search(rx,line)
                if m: block[key]=float(m.group(1))
            if line.startswith('}'):
                loads.append(block); block=None
        m=re.search(TRACK_RE,line)
        if m and line.startswith('track:'):
            tracks.append(tuple(int(v) for v in m.groups()))
    return rows,loads,tracks

def _mad(vals):
    if len(vals)<2: return 0.0
    d=[abs(b-a) for a,b in zip(vals,vals[1:])]
    return st.median(d)

def analyze(lines, skip_s=10):
    rows,loads,tracks=parse(lines)
    r={'rows':len(rows),'loads':len(loads),'tracks':len(tracks)}
    valid=[x for x in rows if x[ix['valid']]==1]
    if rows:
        t0=rows[0][ix['t_ms']]
        rows=[x for x in rows if x[ix['t_ms']]-t0>=skip_s*1000]   # the port opening, the cold start
        valid=[x for x in rows if x[ix['valid']]==1]
    minutes=(rows[-1][ix['t_ms']]-rows[0][ix['t_ms']])/60000.0 if len(rows)>1 else 0.0
    r['minutes']=minutes
    def col(c,src): return [x[ix[c]] for x in src]
    # ---- events ----
    e={}
    if len(loads)>=2:
        a,b=loads[0],loads[-1]
        d=lambda k: b.get(k,0)-a.get(k,0)
        e['cold_starts']=int(d('cold_starts'))
        e['why']={k:int(d(k)) for k in ('absent','few','coasted','rejected')}
        e['presence_toggles']=int(d('presence_toggles')); e['frames_missed']=int(d('frames_missed'))
        e['max_peak_level']=max(x.get('max_peak_level',0.0) for x in loads[1:])   # the maximum SINCE the first :load reset it
    if len(tracks)>=2:
        e['track_taken']=tracks[-1][0]-tracks[0][0]; e['track_dropped']=tracks[-1][1]-tracks[0][1]; e['track_restarts']=tracks[-1][2]-tracks[0][2]
    ts=col('t_ms',rows)
    gaps=[b-a for a,b in zip(ts,ts[1:]) if b-a>GAP_MS]
    e['gaps']=len(gaps); e['gap_ms']=sum(gaps)
    states=[int(x[ix['track_state']]) for x in rows]
    e['states']={k:states.count(k) for k in range(4)}
    e['transitions']=sum(1 for a,b in zip(states,states[1:]) if a!=b)
    e['rough_while_valid']=(sum(1 for x in valid if int(x[ix['track_state']])==1)/len(valid)) if valid else 0.0
    if rows: e['dropped_delta']=int(rows[-1][ix['dropped']]-rows[0][ix['dropped']])
    r['events']=e
    # ---- floors and jitter (valid fixes) ----
    f={}
    if len(valid)>=3:
        sig={}
        for a in 'xyz':
            v=col('raw_'+a,valid); dd=[v[k]-2*v[k-1]+v[k-2] for k in range(2,len(v))]
            sig[a]=st.pstdev(dd)/math.sqrt(6)
        f['sigma_r']=sig
        err=[math.sqrt(x[ix['sigma_x']]**2+x[ix['sigma_y']]**2+x[ix['sigma_z']]**2) for x in valid]
        f['error_over_5mm']=sum(1 for v in err if v>ROUGH_MM)/len(err)
        f['seen_min']=int(min(col('seen_by',valid))); f['seen_mean']=st.mean(col('seen_by',valid)); f['faint_mean']=st.mean(col('faint_by',valid))
        g=col('gate',valid); f['gate_mean']=st.mean(g); f['gate_over_4']=sum(1 for v in g if v>4)/len(g)
        f['raw_sd']={a:st.pstdev(col('raw_'+a,valid)) for a in 'xyz'}
        f['raw_mad']={a:_mad(col('raw_'+a,valid)) for a in 'xyz'}
        tr=[x for x in valid if x[ix['track_state']]>=2]
        f['track_mad']={a:_mad(col('track_'+a,tr)) for a in 'xyz'} if len(tr)>2 else None
        f['cursor_mad']={a:_mad(col('cursor_'+a,tr)) for a in 'xy'} if len(tr)>2 else None
        f['tilt_sd']=st.pstdev(col('tilt_deg',valid))
    r['floors']=f
    r['valid']=len(valid); r['rows_after_skip']=len(rows)
    return r

def legacy_report(name, lines, skip_s=10):
    """What this tool printed before 2026-09-23 (kept so old notes compare)."""
    rows,_,_=parse(lines)
    valid=[x for x in rows if x[ix['valid']]==1]
    if len(valid)<10: print(name, 'too few valid', len(valid)); return
    t0=valid[0][ix['t_ms']]; valid=[x for x in valid if x[ix['t_ms']]-t0>skip_s*1000]
    if len(valid)<2: print(name, 'too few valid after the skip'); return
    dur=(valid[-1][ix['t_ms']]-valid[0][ix['t_ms']])/1000
    def col(c): return [x[ix[c]] for x in valid]
    def sd(c): return st.pstdev(col(c))
    ax=[(x[ix['axis_x']],x[ix['axis_y']],x[ix['axis_z']]) for x in valid]
    mx=tuple(st.mean(v) for v in zip(*ax)); n=math.sqrt(sum(v*v for v in mx)) or 1.0; mx=tuple(v/n for v in mx)
    ang=[math.degrees(math.acos(max(-1,min(1,sum(u*v for u,v in zip(a,mx)))))) for a in ax]
    tr=[x for x in valid if x[ix['track_state']]>=2]
    print(f"{name}: {len(valid)} fixes in {dur:.0f} s ({len(valid)/max(dur,1e-9):.1f}/s), {len(rows)-len([x for x in rows if x[ix['valid']]==1])} not valid")
    print(f"   raw fix   mean x {st.mean(col('x')):.2f} y {st.mean(col('y')):.2f} z {st.mean(col('z')):.2f}   sd {sd('x'):.3f} {sd('y'):.3f} {sd('z'):.3f} mm   tilt {st.mean(col('tilt_deg')):.1f} deg, axis wander {st.mean(ang):.2f} deg mean / {st.pstdev(ang):.2f} sd   strength {st.mean(col('strength')):.0f} sd {sd('strength'):.0f}   misfit {st.mean(col('misfit'))*100:.1f} %   fit {st.mean(col('fit_us')):.0f} us")
    if tr:
        def tcol(c): return [x[ix[c]] for x in tr]
        print(f"   tracked   sd {st.pstdev(tcol('track_x')):.3f} {st.pstdev(tcol('track_y')):.3f} {st.pstdev(tcol('track_z')):.3f} mm   cursor sd {st.pstdev(tcol('cursor_x')):.3f} {st.pstdev(tcol('cursor_y')):.3f} mm   ({len(tr)} tracked)")

def report(name, r, minutes_arg=None):
    e=r['events']; f=r['floors']
    m=minutes_arg if minutes_arg else r['minutes']
    per=lambda n: f"{n/m:.2f}/min" if m>0 else "n/a per min"
    print(f"   events    " + (f"cold starts {e['cold_starts']} ({per(e['cold_starts'])}: absent {e['why']['absent']}, few {e['why']['few']}, coasted out {e['why']['coasted']}, rejected {e['why']['rejected']}); presence toggles {e['presence_toggles']} ({per(e['presence_toggles'])}); frames missed {e['frames_missed']}; strongest reading since the first :load {e['max_peak_level']:.4f}" if 'cold_starts' in e else "no :load pair (bracket the capture with :load)"))
    print(f"             " + (f"track: +{e['track_taken']} fixes, +{e['track_dropped']} dropped ({100.0*e['track_dropped']/max(e['track_taken'],1):.1f} %), +{e['track_restarts']} restarts;" if 'track_taken' in e else "no track: pair (send l at both ends);") + f" stream gaps >{GAP_MS} ms: {e['gaps']} ({e['gap_ms']/1000:.1f} s); states none {e['states'][0]} rough {e['states'][1]} coast {e['states'][2]} track {e['states'][3]}, transitions {e['transitions']}; rough-while-valid {100*e['rough_while_valid']:.1f} %" + (f"; dropped +{e['dropped_delta']}" if 'dropped_delta' in e else ""))
    if f:
        print(f"   floors    sigma_R x {f['sigma_r']['x']:.3f} y {f['sigma_r']['y']:.3f} z {f['sigma_r']['z']:.3f} mm (the R floor: std of second differences / sqrt 6); bar >{ROUGH_MM:.0f} mm: {100*f['error_over_5mm']:.1f} %; seen by min {f['seen_min']} mean {f['seen_mean']:.1f} (+{f['faint_mean']:.1f} faint); gate mean {f['gate_mean']:.2f} sigma, >4: {100*f['gate_over_4']:.1f} %")
        line=f"   jitter    raw sd x {f['raw_sd']['x']:.3f} y {f['raw_sd']['y']:.3f} z {f['raw_sd']['z']:.3f}, frame-to-frame MAD x {f['raw_mad']['x']:.3f} y {f['raw_mad']['y']:.3f} z {f['raw_mad']['z']:.3f} mm; tilt sd {f['tilt_sd']:.2f} deg"
        if f['track_mad']: line+=f"; track MAD x {f['track_mad']['x']:.3f} y {f['track_mad']['y']:.3f} z {f['track_mad']['z']:.3f}; cursor MAD x {f['cursor_mad']['x']:.3f} y {f['cursor_mad']['y']:.3f} mm"
        print(line)
    else:
        print("   floors    too few valid fixes for the floors")

def main(argv):
    minutes=None; files=[]
    for a in argv:
        if a.startswith('--minutes='): minutes=float(a.split('=',1)[1])   # the absent run: no fix rows to time it by
        else: files.append(a)
    if not files:
        print("usage: fixstats.py [--minutes=N] capture.txt [capture_end.txt ...] | --selftest"); return 2
    lines=[]
    for fn in files: lines+=open(fn, errors='replace').readlines()
    name=' + '.join(files)
    legacy_report(name, lines)
    r=analyze(lines)
    report(name, r, minutes)
    return 0

# ---- self-check: `python3 tools/fixstats.py --selftest` (the one runnable check; fails if the event math breaks) ----
def _synthetic():
    import random
    random.seed(1)
    lines=[]
    lines.append("load{")
    lines.append("fit: running, steady load, the last 800 us; 3 cold starts since boot, the last 900 us in all, its longest slice 400 us - the V5F's heaviest work")
    lines.append("  the track ended before them: 1 times nothing present, 0 too few noticing, 0 the track coasted out; 0 cold starts rejected (chi over 2.0, a bar over 30 mm, or beyond the array)")
    lines.append("  presence toggled 1 times; 5 frames missed while tracking; the strongest reading since the last look 0.0210 (TMAG terms; the presence level is 0.040)")
    lines.append("}")
    lines.append("track: tracking  x 1 y 2 z 3 +/-0.1 0.1 0.1 mm  velocity 0 mm/s (moving 0, smoothing alpha 1.00)  tilt 5 deg  cursor aim x 1 y 2 (+/-0.1 mm, reach 0.0)  10 fixes taken, 1 dropped, 0 restarts")
    t=100000
    for k in range(200):
        if k==100: t+=500   # one gap of ~550 ms
        x=10+random.gauss(0,0.1); y=20+random.gauss(0,0.1); z=15+random.gauss(0,0.05)
        state=3 if k>=10 else 1
        gate=abs(random.gauss(0,1))
        row=["fix",t,1,1,x,y,z,x,y,z,0,0,1,5.0,1839,0.005,800,0.04,5,0.2,0.2,0.1,2,x,y,z,state,x,y,z,0.2,0.2,0.1,x,y,z,gate,1 if k<150 else 3]
        lines.append(",".join(str(v) for v in row))
        t+=50
    lines.append("load{")
    lines.append("fit: running, steady load, the last 800 us; 8 cold starts since boot, the last 900 us in all, its longest slice 400 us - the V5F's heaviest work")
    lines.append("  the track ended before them: 2 times nothing present, 1 too few noticing, 0 the track coasted out; 1 cold starts rejected (chi over 2.0, a bar over 30 mm, or beyond the array)")
    lines.append("  presence toggled 4 times; 12 frames missed while tracking; the strongest reading since the last look 0.0310 (TMAG terms; the presence level is 0.040)")
    lines.append("}")
    lines.append("track: tracking  x 1 y 2 z 3 +/-0.1 0.1 0.1 mm  velocity 0 mm/s (moving 0, smoothing alpha 1.00)  tilt 5 deg  cursor aim x 1 y 2 (+/-0.1 mm, reach 0.0)  210 fixes taken, 3 dropped, 1 restarts")
    return lines

def selftest():
    r=analyze(_synthetic(), skip_s=0)
    e=r['events']
    assert e['cold_starts']==5 and e['why']=={'absent':1,'few':1,'coasted':0,'rejected':1}, e
    assert e['presence_toggles']==3 and e['frames_missed']==7, e
    assert e['track_dropped']==2 and e['track_restarts']==1 and e['track_taken']==200, e
    assert e['gaps']==1 and 500<=e['gap_ms']<=600, e
    assert e['states']=={0:0,1:10,2:0,3:190} and e['transitions']==1, e
    assert abs(e['rough_while_valid']-0.05)<1e-9, e
    f=r['floors']
    assert all(0.05<f['sigma_r'][a]<0.16 for a in 'xy') and f['sigma_r']['z']<0.1, f
    assert f['error_over_5mm']==0.0 and f['seen_min']==5, f
    assert 0.5<f['gate_mean']<1.2, f
    assert abs(r["minutes"]-(199*50+500)/60000.0)<1e-6, r["minutes"]   # 200 rows span 199 intervals
    # tools/readport.py writes every line with the host time in front ("   12.3 fix,..."): the same capture must parse the same
    r2=analyze(["%7.1f %s" % (0.1*k, l) for k,l in enumerate(_synthetic())], skip_s=0)
    assert r2['rows']==r['rows']==200 and r2['loads']==2 and r2['tracks']==2 and r2['events']==r['events'], (r2['rows'], r2['loads'], r2['tracks'])
    print("fixstats selftest ok")


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--selftest':
        selftest()
    else:
        sys.exit(main(sys.argv[1:]))
