#!/usr/bin/env bash
# Render the experiment setup sheet to self-contained local PDF + PNG, LIGHT and DARK.
# No network, no artifacts. Usage: bash docs/render-setup-sheet.sh
set -e
cd "$(dirname "$0")/.."
SRC=docs/paper4-experiment-setup.html
CHROME=$(command -v google-chrome || command -v chromium || command -v chromium-browser)

render() {  # $1 = theme (light|dark), $2 = suffix
  local theme="$1" suf="$2"
  local standalone="docs/.setup.$theme.html"
  python3 - "$SRC" "$standalone" "$theme" <<'PY'
import sys
src=open(sys.argv[1]).read()
open(sys.argv[2],"w").write(
 f'<!doctype html>\n<html lang="en" data-theme="{sys.argv[3]}">\n<head>\n'
 '<meta charset="utf-8">\n<meta name="viewport" content="width=device-width,initial-scale=1">\n'
 +src+'\n</body>\n</html>\n')
PY
  "$CHROME" --headless=new --disable-gpu --no-sandbox --no-pdf-header-footer \
    --print-to-pdf="docs/paper4-experiment-setup$suf.pdf" "file://$PWD/$standalone" 2>/dev/null
  "$CHROME" --headless=new --disable-gpu --no-sandbox --hide-scrollbars \
    --window-size=1180,4300 --screenshot="docs/paper4-experiment-setup$suf.png" "file://$PWD/$standalone" 2>/dev/null
  rm -f "$standalone"
  echo "rendered $theme -> docs/paper4-experiment-setup$suf.{pdf,png}"
}

render light ""
render dark ".dark"
