# Consolidated manuscript

The single journal paper folding papers 1–4 (see `DOSSIER.md` for the arc and the IoT-J
reviewer-response ledger).

- **Target venue:** IEEE Access (open access).
- **Build:** `pdflatex main; bibtex main; pdflatex main; pdflatex main`
- **State:** **ready to submit** — 13 sections, 7 figures, 4 tables, 9 pp; all references
  resolve; author biography and photo in place; acronyms audited against IEEE Access
  checklist item 11.

See **`SUBMISSION.md`** for the submission package, the portal field values, the
compliance table against `../IEEE-Access-Submission-Checklist.pdf`, and what is still
open off-repo (APC, waiver eligibility, revision policy).

## Figures
`figures/` — `paper2_fig2..6.pdf` regenerate via `experiments/make_paper2_figures.py`;
`ap_sensitivity.png` / `blockage_robustness.png` via `experiments/make_sensitivity_figures.py`.
