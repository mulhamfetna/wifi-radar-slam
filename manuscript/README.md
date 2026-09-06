# Consolidated manuscript

The single journal paper folding papers 1–4 (see `DOSSIER.md` for the arc and the IoT-J
reviewer-response ledger).

- **Target venue:** IEEE Access (open access).
- **Build:** `pdflatex main; bibtex main; pdflatex main; pdflatex main`
- **State:** submission draft — 13 sections, 7 figures, 4 tables, 9 pp; all references resolve.

## Before submitting — two things need your input

1. **Fill the author biography placeholders** (end of `main.tex`):
   `[DEGREE] [FIELD] [INSTITUTION] [CITY, COUNTRY] [YEAR] [ROLE] [AFFILIATION]`.
   To add a photo: drop a headshot at `figures/author-photo.jpg`, then comment out the
   `\IEEEbiographynophoto` block and uncomment the `\IEEEbiography` block above it.
2. **Port to the IEEE Access class** — 4 mechanical steps documented at the top of `main.tex`.
   IEEE ships `ieeeaccess.cls` only in the Author Center template zip (it is not on CTAN), so it
   must be downloaded once; the body, figures, tables, bibliography and biography carry over
   unchanged. The Access front matter (`\history`, `\doi`, `\address`, `\corresp`) is already
   written out, commented, ready to uncomment — fill `[DEPARTMENT]/[INSTITUTION]/[CITY]/[COUNTRY]`.

## Figures
`figures/` — `paper2_fig2..6.pdf` regenerate via `experiments/make_paper2_figures.py`;
`ap_sensitivity.png` / `blockage_robustness.png` via `experiments/make_sensitivity_figures.py`.
