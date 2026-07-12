# Paper — Ambient WiFi as a Radar Replacement for Automotive SLAM

> ## ⚠️ ERRATUM (2026-07-12) — please read before reproducing
>
> A re-verification of this artifact found that **one value reported in the submitted
> manuscript does not reproduce**. It was **self-reported to the IEEE IoT-J editor on
> 2026-07-12**, and `main.tex` / `main.pdf` in this directory are the **corrected**
> version.
>
> **The error.** The submitted manuscript reported that joint 2-D MUSIC lifts realistic
> ATE to **0.027 m**, "matching" the 0.045 m oracle. Re-running this repository's own code
> with its own config (`configs/controlled_music_joint.yaml`) gives **0.098 ± 0.028 m**
> (mean ± std over seeds 42, 1, 2, 3, 4, 5 → 0.143, 0.108, 0.056, 0.089, 0.092, 0.097).
> The reported value lies below the minimum of every seed, so this is not run-to-run
> variance. It appears to have been recorded from an earlier state of the code and not
> re-verified before submission.
>
> **The corrected claim.** Joint 2-D MUSIC still improves realistic ATE ≈**7×** over
> sorted 1-D pairing (0.73 m → 0.098 m), so the qualitative finding stands — but it
> **approaches rather than matches** the oracle (0.049 ± 0.027 m). The "oracle-quality
> realistic localization" claim has been removed. WiFi results are now reported as
> **mean ± std over five seeds**, not a single run.
>
> **Scope — the rest of the paper reproduces cleanly** (5-seed audit,
> `experiments/regen_wifi_results.py`): the controlled-scene oracle map
> (0.049±0.027 / 0.248±0.003 / IoU 0.791 vs reported 0.045 / 0.25 / 0.79), the
> street-scene oracle map, and the 60 GHz + 16-antenna null result all hold.
>
> **Two further corrections**, also in the corrected manuscript:
> 1. The learned discriminator's **F1 ≈ 0.9 used an arrival-elevation feature**, which the
>    single-ULA delay–azimuth front-end described in the paper does **not** estimate. It is
>    an upper bound, not a commodity-CSI result (observable-feature F1: 0.00–0.45).
> 2. Follow-up work indicates the realistic-mapping floor is dominated by **phantom
>    detections** (≈89 % of MUSIC detections match no real propagation path) and an
>    estimator **range bias**, rather than by path discrimination. The *empirical* mapping
>    results are unchanged; the interpretation is softened.
>
> Full record: `DOSSIER.md` → *ERRATUM*. Disclosure letter: `erratum-to-editor.md`.
> The exact state that was submitted is preserved at tag `v0.7.1` / branch
> `paper1-submitted`.

**Target venue: IEEE Internet of Things Journal (IoT-J)** — IEEEtran journal class.
**Status: complete draft, editorially passed.** All sections (localization, joint
MUSIC, oracle-vs-realistic mapping, the 60 GHz/aperture path-discrimination finding,
the learned discriminator, real-CSI proof-of-concept) are written from the released
result artifacts (`../../docs/results-v1.md`); all numbers are reproduced from those.
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
`siunitx` dependency). Figures come from `../../docs/assets/` via `\graphicspath`;
on Overleaf, upload `paper/` and either copy the referenced PNGs next to
`main.tex` or keep the relative path.

## Files
- `main.tex` — manuscript.
- `refs.bib` — bibliography (copied from `../literature/references.bib`, 25 entries).
- Figures are referenced from `../../docs/assets/` (scene renders + map figures).

## Before submission
- Fill Sec. VII (60 GHz) with results.
- Re-read every `%% CHECK` number from `../../docs/results-v1.md` / the result JSONs.
- Choose the target venue (e.g. IEEE IoT-J, IEEE Sensors J., IEEE T-ITS) and switch
  to its template if needed.
- Consider adding a real-CSI proof-of-concept to strengthen the Q1 case.
