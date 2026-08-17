# Consolidated manuscript

The single journal paper folding papers 1–4 (see `DOSSIER.md` for the arc and the IoT-J
reviewer-response ledger).

- **Target venue:** IEEE Access (open access, fast). Drafted in `IEEEtran` (builds offline);
  **port to `ieeeaccess.cls`** for submission.
- **Build:** `pdflatex main; bibtex main; pdflatex main; pdflatex main`
  (the `\bibliography` lines are commented until sections carry `\cite{...}`).
- **State:** SKELETON — full structure, written abstract, real result tables/figures, section
  stubs with the key numbers and `% TODO` markers; reviewer points mapped in the header comments.
- **⬚ Hardware slot** (Sec. VI): the phantom rate on a real reflector — fill once a ≥15 m
  corridor is found and Rung 1 runs.
- Figures in `figures/`; regenerate the robustness ones via
  `experiments/make_sensitivity_figures.py`.
