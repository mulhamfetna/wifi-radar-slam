#!/usr/bin/env bash
# Builds manuscript/submission/ (gitignored): every file to upload to Editorial Manager.
#   bash manuscript/make-submission.sh
set -euo pipefail
cd "$(dirname "$0")"
pdflatex -interaction=nonstopmode main >/dev/null && bibtex main >/dev/null \
  && pdflatex -interaction=nonstopmode main >/dev/null && pdflatex -interaction=nonstopmode main >/dev/null
grep -qE '^! |undefined' main.log && { echo "LaTeX errors - see main.log"; exit 1; }
rm -rf submission && mkdir -p submission/latex-source/figures
cp main.tex refs.bib main.bbl elsarticle.cls elsarticle-num.bst submission/latex-source/
cp figures/*.pdf submission/latex-source/figures/
(cd submission/latex-source && zip -qr ../01-latex-source.zip .)
cp main.pdf submission/02-manuscript.pdf
cp highlights.txt submission/03-highlights.txt
sed "s/^/- /" highlights.txt | pandoc -f markdown -t docx -o submission/03-highlights.docx
pandoc cover-letter.md -o submission/04-cover-letter.pdf --pdf-engine=xelatex \
  -V geometry:margin=1in -V fontsize=11pt -V mainfont="DejaVu Serif"
pandoc cover-letter.md -o submission/04-cover-letter.docx
i=1; for f in localization frontend_decomposition maps_controlled robustness; do
  cp figures/$f.pdf "submission/05-Figure_${i}.pdf"; i=$((i+1)); done
# compile check of the source zip in a clean directory
t=$(mktemp -d); (cd "$t" && unzip -q "$OLDPWD/submission/01-latex-source.zip" \
  && pdflatex -interaction=nonstopmode main >/dev/null && pdflatex -interaction=nonstopmode main >/dev/null \
  && echo "clean-directory build: $(pdfinfo main.pdf | grep Pages)"); rm -rf "$t"
ls -1 submission
