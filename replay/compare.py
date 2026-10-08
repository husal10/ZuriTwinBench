"""Field-vs-simulated detector comparison (1 s resolution)."""
import numpy as np, pandas as pd


def _rises(x):
    x = np.asarray(x) > 0
    return np.where(x[1:] & ~x[:-1])[0] + 1 if len(x) else np.array([], int)


def match_events(a, b, tol):
    """Greedy one-to-one matching of event times within +-tol; returns #matched."""
    i = j = m = 0
    while i < len(a) and j < len(b):
        if abs(a[i] - b[j]) <= tol:
            m += 1; i += 1; j += 1
        elif a[i] < b[j]:
            i += 1
        else:
            j += 1
    return m


def compare(real, sim, dets, tol=2):
    rows = {}
    for d in dets:
        r, s = (real[d].to_numpy() > 0), (sim[d].to_numpy() > 0)
        er, es = _rises(r), _rises(s)
        m = match_events(er, es, tol)
        inter, union = (r & s).sum(), (r | s).sum()
        rows[d] = {
            "events_real": len(er), "events_sim": len(es),
            "event_recall": m / len(er) if len(er) else np.nan,
            "event_precision": m / len(es) if len(es) else np.nan,
            "occ_s_real": int(r.sum()), "occ_s_sim": int(s.sum()),
            "second_accuracy": float((r == s).mean()),
            "jaccard": float(inter / union) if union else 1.0,
        }
    return pd.DataFrame(rows).T


def format_report(rep):
    out = rep.round(3).to_string()
    for k, v in rep.attrs.items():
        out += f"\n{k}: {v:.2f}" if isinstance(v, float) else f"\n{k}: {v}"
    return out
