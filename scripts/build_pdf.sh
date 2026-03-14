#!/usr/bin/env bash
set -euo pipefail

# Build PDF from docs/main.tex using pdflatex
# Usage: ./scripts/build_pdf.sh
# Supports: macOS (Homebrew/MacTeX), Windows (Git Bash + MiKTeX), Linux (TeX Live)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$PROJECT_ROOT/docs"
TEX_FILE="main.tex"
OUTPUT_DIR="$DOCS_DIR/output"

# ── Detect OS ────────────────────────────────────────────────────────────────
OS="$(uname -s)"
case "$OS" in
  Darwin*)  PLATFORM="macos" ;;
  Linux*)   PLATFORM="linux" ;;
  MINGW*|MSYS*|CYGWIN*) PLATFORM="windows" ;;
  *)        PLATFORM="unknown" ;;
esac

# ── Locate pdflatex ───────────────────────────────────────────────────────────
find_pdflatex_windows() {
  local candidates=(
    "/c/Users/$USERNAME/AppData/Local/Programs/MiKTeX/miktex/bin/x64/pdflatex.exe"
    "/c/Program Files/MiKTeX/miktex/bin/x64/pdflatex.exe"
    "/c/Program Files (x86)/MiKTeX/miktex/bin/pdflatex.exe"
  )
  for path in "${candidates[@]}"; do
    if [[ -f "$path" ]]; then
      echo "$path"
      return 0
    fi
  done
  return 1
}

PDFLATEX=""

if command -v pdflatex &>/dev/null; then
  PDFLATEX="pdflatex"
elif [[ "$PLATFORM" == "windows" ]]; then
  if PDFLATEX_PATH="$(find_pdflatex_windows)"; then
    PDFLATEX="$PDFLATEX_PATH"
  else
    echo "ERROR: pdflatex not found."
    echo ""
    echo "Install MiKTeX for Windows (no admin required):"
    echo "  https://miktex.org/download"
    echo ""
    echo "After install, re-open Git Bash and retry."
    exit 1
  fi
elif [[ "$PLATFORM" == "macos" ]]; then
  echo "==> pdflatex not found. Installing MacTeX via Homebrew..."
  if ! command -v brew &>/dev/null; then
    echo "ERROR: Homebrew is not installed. Install it from https://brew.sh and retry."
    exit 1
  fi
  brew install --cask mactex-no-gui
  eval "$(/usr/libexec/path_helper)"
  PDFLATEX="pdflatex"
  echo "==> MacTeX installed."
elif [[ "$PLATFORM" == "linux" ]]; then
  echo "ERROR: pdflatex not found. Install TeX Live:"
  echo "  sudo apt install texlive-full   # Debian/Ubuntu"
  echo "  sudo dnf install texlive-scheme-full  # Fedora"
  exit 1
else
  echo "ERROR: pdflatex not found and platform '$OS' is not recognized."
  exit 1
fi

# ── Build ─────────────────────────────────────────────────────────────────────
mkdir -p "$OUTPUT_DIR"

echo "==> Using: $PDFLATEX"
echo "==> Compiling $TEX_FILE..."

cd "$DOCS_DIR"

# Run twice to resolve references and TOC
"$PDFLATEX" -interaction=nonstopmode -output-directory="$OUTPUT_DIR" "$TEX_FILE"
"$PDFLATEX" -interaction=nonstopmode -output-directory="$OUTPUT_DIR" "$TEX_FILE"

echo "==> PDF generated at: $OUTPUT_DIR/main.pdf"
