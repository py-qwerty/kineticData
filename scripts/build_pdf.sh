#!/usr/bin/env bash
set -euo pipefail

# Build PDF from docs/main.tex using pdflatex
# Usage: ./scripts/build_pdf.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$PROJECT_ROOT/docs"
TEX_FILE="main.tex"
OUTPUT_DIR="$DOCS_DIR/output"

# Check if pdflatex is installed
if ! command -v pdflatex &>/dev/null; then
  echo "==> pdflatex not found. Installing MacTeX via Homebrew..."
  if ! command -v brew &>/dev/null; then
    echo "Error: Homebrew is not installed. Install it from https://brew.sh and retry."
    exit 1
  fi
  brew install --cask mactex-no-gui
  eval "$(/usr/libexec/path_helper)"
  echo "==> MacTeX installed successfully."
fi

mkdir -p "$OUTPUT_DIR"

echo "==> Compiling $TEX_FILE with pdflatex..."

cd "$DOCS_DIR"

# Run pdflatex twice to resolve references and TOC
pdflatex -interaction=nonstopmode -output-directory="$OUTPUT_DIR" "$TEX_FILE"
pdflatex -interaction=nonstopmode -output-directory="$OUTPUT_DIR" "$TEX_FILE"

echo "==> PDF generated at: $OUTPUT_DIR/main.pdf"