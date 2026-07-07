# Release & DOI checklist

Repo: https://github.com/mulhamfetna/wifi-radar-slam · numeric id `1292636094`

The DOI badge in the README is already wired to this repo id; it stays "pending" until the first
release is archived by Zenodo. **Order matters** — do these steps in sequence.

## Zenodo connection (one-time, only you can do this — needs your login)

1. Go to **https://zenodo.org** → *Log in with GitHub* → **Authorize** (one-time).
2. In your Zenodo profile, connect your **ORCID** (0009-0006-4432-798X) so the DOI is linked to you.
3. Go to **https://zenodo.org/account/settings/github/** → find `mulhamfetna/wifi-radar-slam` →
   flip the toggle **ON** (click *Sync* if it's not listed yet).

> ⚠️ The repo MUST be enabled in Zenodo **before** you publish the release below. Publishing the
> release first is the #1 way this silently fails to archive.

## Cut the release (mints the DOI)

After the toggle is ON, publish a GitHub Release. You (or I, on your go-ahead) run:

```bash
gh release create v0.1.0 \
  --title "v0.1.0 — Feasibility simulation (sub-7 GHz WiFi-radar SLAM)" \
  --notes "First release: verified two-round literature survey + physics-based (Sionna RT) \
feasibility simulation pipeline (scene → channel → sensing → SLAM → eval), Phase-A/Phase-B \
experiments. Sub-7 GHz path; 60 GHz mmWave planned as future work."
```

Zenodo fires within ~a minute and mints two DOIs: a **concept DOI** (all versions) and a
**version DOI** (v0.1.0).

## After the DOI exists

Read the DOIs back and record them:

```bash
# find the record id from your Zenodo dashboard, then:
curl -s https://zenodo.org/api/records/<record_id> | python -c "import sys,json;d=json.load(sys.stdin);print('version:',d['doi']);print('concept:',d.get('conceptdoi'))"
```

Then add to `CITATION.cff` (`doi:` = concept DOI, plus an `identifiers:` list with both) and the
README BibTeX, commit, and push. Future versions: just `gh release create v0.2.0 …` after bumping
`version`/`date-released` in `CITATION.cff`.
