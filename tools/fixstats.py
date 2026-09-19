import sys, statistics as st, math
cols="fix,t_ms,present,valid,x,y,z,tip_x,tip_y,tip_z,axis_x,axis_y,axis_z,tilt_deg,strength,residual_mT,fit_us,misfit,seen_by,sigma_x,sigma_y,sigma_z,faint_by,raw_x,raw_y,raw_z,track_state,track_x,track_y,track_z,track_sx,track_sy,track_sz,cursor_x,cursor_y,cursor_z,gate,dropped".split(',')
ix={c:i for i,c in enumerate(cols)}
for f in sys.argv[1:]:
    rows=[]
    for line in open(f, errors='replace'):
        if not line.startswith('fix,'): continue
        p=line.strip().split(',')
        if len(p)<len(cols): continue
        try: rows.append([float(v) for v in p[1:]])
        except: continue
    rows=[[0]+r for r in rows]
    valid=[r for r in rows if r[ix['valid']]==1]
    if len(valid)<10: print(f, 'too few valid', len(valid)); continue
    # skip the first 10 s (cold start / the port opening)
    t0=valid[0][ix['t_ms']]; valid=[r for r in valid if r[ix['t_ms']]-t0>10000]
    dur=(valid[-1][ix['t_ms']]-valid[0][ix['t_ms']])/1000
    def col(c): return [r[ix[c]] for r in valid]
    def sd(c): return st.pstdev(col(c))
    ang=[]
    ax=[(r[ix['axis_x']],r[ix['axis_y']],r[ix['axis_z']]) for r in valid]
    mx=tuple(st.mean(v) for v in zip(*ax)); n=math.sqrt(sum(v*v for v in mx)); mx=tuple(v/n for v in mx)
    for a in ax:
        d=max(-1,min(1,sum(u*v for u,v in zip(a,mx)))); ang.append(math.degrees(math.acos(d)))
    tr=[r for r in valid if r[ix['track_state']]>=2]
    print(f"{f}: {len(valid)} fixes in {dur:.0f} s ({len(valid)/dur:.1f}/s), {len(rows)-len([r for r in rows if r[ix['valid']]==1])} not valid")
    print(f"   raw fix   mean x {st.mean(col('x')):.2f} y {st.mean(col('y')):.2f} z {st.mean(col('z')):.2f}   sd {sd('x'):.3f} {sd('y'):.3f} {sd('z'):.3f} mm   tilt {st.mean(col('tilt_deg')):.1f} deg, axis wander {st.mean(ang):.2f} deg mean / {st.pstdev(ang):.2f} sd   strength {st.mean(col('strength')):.0f} sd {sd('strength'):.0f}   misfit {st.mean(col('misfit'))*100:.1f} %   fit {st.mean(col('fit_us')):.0f} us")
    if tr:
        def tcol(c): return [r[ix[c]] for r in tr]
        print(f"   tracked   sd {st.pstdev(tcol('track_x')):.3f} {st.pstdev(tcol('track_y')):.3f} {st.pstdev(tcol('track_z')):.3f} mm   cursor sd {st.pstdev(tcol('cursor_x')):.3f} {st.pstdev(tcol('cursor_y')):.3f} mm   ({len(tr)} tracked)")
