# Paper draft — Ambient WiFi as a Radar Replacement for Automotive SLAM

Q1-targeted manuscript (IEEEtran journal class). **Status: skeleton.** The
sub-7 GHz localization, realistic-sensing and mapping sections are written from the
released results (`../docs/results-v1.md`, v0.4.0). The **60 GHz extension**
(Sec. VII) and any real-CSI validation are pending — marked `%% TODO` in `main.tex`.
Numbers to re-verify against the artifacts before submission are marked `%% CHECK`.

## Build

`main.pdf` is the committed rendered version (current release). To rebuild:

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

`IEEEtran.cls`/`IEEEtran.bst` are vendored so the paper builds without a TeX
distribution that ships them, and the unit macros are defined inline (no
`siunitx` dependency). Figures come from `../docs/assets/` via `\graphicspath`;
on Overleaf, upload `paper/` and either copy the referenced PNGs next to
`main.tex` or keep the relative path.

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
