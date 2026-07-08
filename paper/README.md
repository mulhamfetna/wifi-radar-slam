# Paper draft — Ambient WiFi as a Radar Replacement for Automotive SLAM

Q1-targeted manuscript (IEEEtran journal class). **Status: skeleton.** The
sub-7 GHz localization, realistic-sensing and mapping sections are written from the
released results (`../docs/results-v1.md`, v0.4.0). The **60 GHz extension**
(Sec. VII) and any real-CSI validation are pending — marked `%% TODO` in `main.tex`.
Numbers to re-verify against the artifacts before submission are marked `%% CHECK`.

## Build

```bash
latexmk -pdf main.tex        # or: pdflatex main && bibtex main && pdflatex main x2
```

Or upload the `paper/` folder to Overleaf (it pulls figures from `../docs/assets/`
via `\graphicspath`; on Overleaf, copy the needed PNGs alongside `main.tex` or
adjust the path).

## Files
- `main.tex` — manuscript.
- `refs.bib` — bibliography (copied from `../literature/references.bib`, 25 entries).
- Figures are referenced from `../docs/assets/` (scene renders + map figures).

## Before submission
- Fill Sec. VII (60 GHz) with results.
- Re-read every `%% CHECK` number from `../docs/results-v1.md` / the result JSONs.
- Choose the target venue (e.g. IEEE IoT-J, IEEE Sensors J., IEEE T-ITS) and switch
  to its template if needed.
- Consider adding a real-CSI proof-of-concept to strengthen the Q1 case.
