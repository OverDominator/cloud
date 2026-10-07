"""Read-only source audit and reproducible paired analysis. Run using WSL Python."""
from pathlib import Path
import csv
import json
import hashlib
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
BATCH = ROOT / 'formal_batches/20261007_135525'
OUT = ROOT / 'analysis_paired_20261007'
OUT.mkdir(exist_ok=True)
records = json.loads((BATCH / 'completed.json').read_text(encoding='utf-8-sig'))
assert len(records) == 100
hashes = {}

def read_run(entry, original=False):
    run = entry['run']
    folder = ROOT / 'runs' / run
    if original and entry['pair_id'] == 34 and entry['variant'] == 'no_call':
        run = '20261007_193332_no_call'
        folder = BATCH / 'excluded_user_requested' / run
    result_file = folder / 'result.json'
    result = json.loads(result_file.read_text(encoding='utf-8-sig'))
    files = list(folder.rglob('trials.csv'))
    assert len(files) == 1, (run, files)
    rows = list(csv.DictReader(files[0].open(encoding='utf-8-sig', newline='')))
    assert len(rows) == 1, (run, len(rows))
    row = rows[0]
    for key in ('pair_id', 'seed', 'variant'):
        assert result[key] == entry[key], (run, key)
    for key in ('data_integrity_pass', 'readiness_pass', 'finished', 'fire_selection_verified'):
        assert result[key] is True, (run, key)
    assert result['pilot_only'] is False
    assert result['call_guidance_param'] == (entry['variant'] == 'full')
    assert int(row['total_victims']) == result['total'] == 4
    assert int(row['rescued_victims']) == result['rescued']
    assert int(row['collision_count']) == result['collisions']
    success = row['mission_success'].lower() == 'true'
    assert success == (result['rescued'] == 4)
    duration = float(row['duration_s'])
    assert 0 < duration < 610
    if not success:
        assert duration >= 599, (run, duration)
    for file in [result_file, files[0]]:
        hashes[str(file.relative_to(ROOT))] = hashlib.sha256(file.read_bytes()).hexdigest()
    return dict(pair_id=entry['pair_id'], seed=entry['seed'], variant=entry['variant'],
                run=run, success=int(success), duration_s=duration,
                path_length_m=float(row['path_length_m']), rescued=result['rescued'],
                collisions=result['collisions'], source_csv=str(files[0].relative_to(ROOT)),
                original_pair34=int(original and entry['pair_id']==34 and entry['variant']=='no_call'))

main = [read_run(e, True) for e in records]
retry = [read_run(e) for e in records]

def export_csv(name, rows):
    with (OUT/name).open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def describe(values):
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                std=float(a.std(ddof=1)), min=float(a.min()), max=float(a.max()))

def analyze(rows):
    lookup = {(r['pair_id'], r['variant']): r for r in rows}
    assert len(lookup) == 100
    pairs = []
    for i in range(1,51):
        f, n = lookup[i,'full'], lookup[i,'no_call']
        assert f['seed'] == n['seed'] == 27000+i
        p = dict(pair_id=i, seed=f['seed'])
        for key in ['run','success','duration_s','path_length_m','rescued','collisions']:
            p['full_'+key], p['no_call_'+key] = f[key], n[key]
        p['both_success'] = int(f['success'] and n['success'])
        pairs.append(p)
    both = [p for p in pairs if p['both_success']]
    b = sum(p['full_success'] and not p['no_call_success'] for p in pairs)
    c = sum(p['no_call_success'] and not p['full_success'] for p in pairs)
    exact_p = min(1.,2*sum(math.comb(b+c,k) for k in range(min(b,c)+1))/2**(b+c)) if b+c else 1.
    summary = {'pair_count':50, 'both_success_count':len(both),
               'full_only_success':b, 'no_call_only_success':c,
               'mcnemar_exact_two_sided_p':exact_p, 'variants':{}, 'paired_both_success':{}}
    for v in ['full','no_call']:
        rs = [r for r in rows if r['variant']==v]
        summary['variants'][v] = dict(n=50,success=sum(r['success'] for r in rs),
            timeout=sum(1-r['success'] for r in rs),
            rescued_total=sum(r['rescued'] for r in rs),collision_events=sum(r['collisions'] for r in rs),
            trials_with_collision=sum(r['collisions']>0 for r in rs))
    rng = np.random.RandomState(20261007)
    for metric in ['duration_s','path_length_m']:
        f = np.array([p['full_'+metric] for p in both])
        n = np.array([p['no_call_'+metric] for p in both])
        d = f-n
        boot = d[rng.randint(0,len(d),size=(20000,len(d)))].mean(axis=1)
        summary['paired_both_success'][metric] = dict(full=describe(f),no_call=describe(n),
            difference_full_minus_no_call=describe(d),
            mean_difference_bootstrap_percentile_ci95=[float(x) for x in np.percentile(boot,[2.5,97.5])],
            full_lower_count=int((d<0).sum()),full_higher_count=int((d>0).sum()))
    return summary,pairs

summary,pairs = analyze(main)
sensitivity,_ = analyze(retry)
export_csv('main_trials.csv',main)
export_csv('main_pairs.csv',pairs)
export_csv('main_timeouts.csv',[r for r in main if not r['success']])
export_csv('retry_sensitivity_trials.csv',retry)
report = dict(main=summary,retry_sensitivity=sensitivity,
    methods={'primary':'50 pairs, original pair 34 no_call retained; successful retry is supplementary only',
             'continuous_metrics':'Only pairs in which both methods completed all 4 victims; conditional, not all-trial efficiency',
             'difference':'full minus no_call; negative means lower time or distance',
             'bootstrap':'20000 paired resamples, numpy RandomState seed 20261007, percentile 95% CI',
             'success_test':'Exact two-sided McNemar, binomial discordant pairs, no continuity correction',
             'limitations':'Exploratory post-hoc analysis. Same seed is verified; identical realized trajectories and all environmental states are not assumed. Simulated call cues and pseudo-thermal class rendering, not real acoustic or radiometric sensing.',
             'exclusions':'Startup-invalid 20261007_184809_full not included. Qualification pilots and video demo excluded. Original valid timeout 20261007_193332_no_call retained.'})
(OUT/'statistics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'source_sha256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')

plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(figsize=(6,4.5))
vals=[summary['variants'][v]['success'] for v in ['full','no_call']]
ax.bar([0,1],np.array(vals)*2,color=['#147D92','#C77734'],width=.55)
for x,y in enumerate(vals):
    ax.text(x,y*2+2,f'{y}/50 ({y*2}%)',ha='center')
ax.set(xticks=[0,1],xticklabels=['Full method','Without call guidance'],ylim=(0,112),
       ylabel='Task success (%)',title='All 50 paired conditions')
ax.set_yticks([0,20,40,60,80,100])
fig.tight_layout()
fig.savefig(OUT/'success_comparison.png',dpi=220)
plt.close(fig)
both=[p for p in pairs if p['both_success']]
fig,axes=plt.subplots(1,2,figsize=(10,4.6))
for ax,key,label in zip(axes,['duration_s','path_length_m'],['Completion time (s)','Travel distance (m)']):
    for p in both:
        ax.plot([0,1],[p['full_'+key],p['no_call_'+key]],color='#a9b2bb',alpha=.5,lw=.8)
    for x,v,col in [(0,'full','#147D92'),(1,'no_call','#C77734')]:
        values=[p[v+'_'+key] for p in both]
        ax.scatter([x]*len(values),values,color=col,s=16,zorder=3)
        ax.plot([x-.12,x+.12],[np.median(values)]*2,color='black',lw=2,zorder=4)
    ax.set(xticks=[0,1],xticklabels=['Full','No call'],ylabel=label,xlim=(-.4,1.4))
fig.suptitle(f'Both-success pairs only (n = {len(both)}); black bars show medians')
fig.tight_layout(rect=[0,0,1,.94])
fig.savefig(OUT/'paired_time_distance.png',dpi=220)
plt.close(fig)
print(json.dumps(report,ensure_ascii=False,indent=2))
