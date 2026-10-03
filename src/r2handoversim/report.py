"""Portable result report with no JavaScript or remote assets."""
import html
from pathlib import Path
import re


def write_report(results, output):
    output = Path(output)
    metrics = ("stability", "plan", "reach", "affordance", "safe")
    rows = []
    for r in results:
        name = str(r["trial_id"])
        links = []
        if re.fullmatch(r"[a-zA-Z0-9_-]+", name):
            for suffix, label in [(".mp4", "Video"), (".png", "Screenshot"), (".usda", "Scene"), ("_animation.usda", "Animation"), ("_trial.json", "Resolved trial"), ("_trajectory.npz", "Trajectory")]:
                filename = name + suffix
                if filename in r.get("artifacts", {}).values() and (output/filename).exists():
                    links.append(f'<a href="{filename}">{label}</a>')
        flags = []
        for metric in metrics:
            v = r["metrics"][metric]
            text, cls = ("—", "na") if v is None else (("Pass", "pass") if v else ("Fail", "fail"))
            flags.append(f'<td class="{cls}">{text}</td>')
        contact = r.get('grasp_contact', {})
        distances = contact.get('bilateral_distance_m')
        contact_text = (' / '.join(f'{d*1000:.3f}' for d in distances) + ' mm') if distances else contact.get('status', '—')
        reference = r.get('replay_reference') or {}
        reference_text = str(reference.get('assigned_outcome', '—'))
        rows.append(f'<tr><td>{html.escape(name)}</td><td>{html.escape(r["split"])}</td>'
                    + ''.join(flags) + f'<td>{html.escape(reference_text)}</td><td>{html.escape(r["first_failure"] or "success")}</td><td>{html.escape(contact_text)}</td><td>{" · ".join(links)}</td></tr>')
    count = sum(r["success"] for r in results)
    document = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>R2HandoverSim results</title><style>
body{background:#111b2d;color:#e9f0ff;font:15px system-ui;margin:40px auto;max-width:1320px;padding:0 24px}h1{font-size:30px}p{color:#b8c8df;line-height:1.7}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:12px;border-bottom:1px solid #35445b;white-space:nowrap}.pass{color:#68dfb5}.fail{color:#ff9292}.na{color:#91a0b8}a{color:#84b8ff}
</style><h1>R2HandoverSim / results</h1>'''
    document += f'<p>{len(results)} trials · {count} successful · {len(results)-count} failed</p>'
    if any(r.get("replay_reference") for r in results):
        document += '<p>Reconstructed paper replay. Metrics below are evaluated from these scenes; assigned reference outcomes remain in results.json.</p>'
    document += '<p>Safety source: ' + html.escape('; '.join(sorted({r.get("safe_source", "unspecified") for r in results}))) + '</p>'
    document += '<div class="scroll"><table><tr><th>Trial</th><th>Split</th>' + ''.join(f'<th>{m.title()}</th>' for m in metrics) + '<th>Reference outcome</th><th>Evaluated outcome</th><th>Pad distances R / L</th><th>Artifacts</th></tr>' + ''.join(rows) + '</table></div>'
    document += '<p>Evaluated replay results. Plan checks the recorded path; Safe uses frame-sampled overlap. S0 omits Affordance. First failure: Stability → Plan → Reach → Affordance → Safe.</p><p><a href="results.json">JSON</a> · <a href="results.csv">CSV</a> · <a href="summary.json">Summary</a></p></html>'
    document = document.replace('</html>', '<p>Table I aggregation (per object, then per split): <a href="paper_table.csv">CSV</a> · <a href="paper_table.json">JSON and conventions</a></p></html>')
    (output/'report.html').write_text(document)
