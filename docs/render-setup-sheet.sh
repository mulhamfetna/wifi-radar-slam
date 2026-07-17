#!/usr/bin/env bash
# Render the experiment setup sheet to a self-contained local PDF + PNG. No network/artifacts.
# Usage: bash docs/render-setup-sheet.sh
set -e
cd "$(dirname "$0")/.."
SRC=docs/paper4-experiment-setup.html
STANDALONE=docs/paper4-experiment-setup.standalone.html
python3 - "$SRC" "$STANDALONE" <<'PY'
import sys
src=open(sys.argv[1]).read()
open(sys.argv[2],"w").write(
 '<!doctype html>\n<html lang="en" data-theme="light">\n<head>\n'
 '<meta charset="utf-8">\n<meta name="viewport" content="width=device-width,initial-scale=1">\n'
 +src+'\n</body>\n</html>\n')
PY
CHROME=$(command -v google-chrome || command -v chromium || command -v chromium-browser)
"$CHROME" --headless=new --disable-gpu --no-sandbox --no-pdf-header-footer \
  --print-to-pdf=docs/paper4-experiment-setup.pdf "file://$PWD/$STANDALONE" 2>/dev/null
"$CHROME" --headless=new --disable-gpu --no-sandbox --hide-scrollbars \
  --window-size=1180,4300 --screenshot=docs/paper4-experiment-setup.png "file://$PWD/$STANDALONE" 2>/dev/null
echo "rendered: docs/paper4-experiment-setup.pdf and .png"
