# Paper — Ambient WiFi as a Radar Replacement for Automotive SLAM

**Target venue: IEEE Internet of Things Journal (IoT-J)** — IEEEtran journal class.
**Status: complete draft, editorially passed.** All sections (localization, joint
MUSIC, oracle-vs-realistic mapping, the 60 GHz/aperture path-discrimination finding,
the learned discriminator, real-CSI proof-of-concept) are written from the released
result artifacts (`../docs/results-v1.md`); all numbers are reproduced from those.
`refs.bib` has been cleaned of internal working notes for submission.

Before submitting: create the IoT-J submission on ScholarOne with this source, and
consider adding an outdoor real-CSI capture (the one item flagged in Future Work).

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
