#!/usr/bin/env bash
set -euo pipefail

# Build PDFs from docs/paper and docs/ble_guide
# Usage: ./scripts/build_pdf.sh [paper|ble_guide|all]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCS_DIR="$PROJECT_ROOT/docs"

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

build_doc() {
  local doc_dir="$1"
  local tex_file="$2"
  local output_dir="$doc_dir/output"

  mkdir -p "$output_dir"
  echo "==> Compiling $tex_file..."
  cd "$doc_dir"
  pdflatex -interaction=nonstopmode -output-directory="$output_dir" "$tex_file"
  pdflatex -interaction=nonstopmode -output-directory="$output_dir" "$tex_file"
  echo "==> PDF generated at: $output_dir/${tex_file%.tex}.pdf"
}

TARGET="${1:-all}"

case "$TARGET" in
  paper)
    build_doc "$DOCS_DIR/paper" "main.tex"
    ;;
  ble_guide)
    build_doc "$DOCS_DIR/ble_guide" "ble_imu_guide.tex"
    ;;
  all)
    build_doc "$DOCS_DIR/paper" "main.tex"
    build_doc "$DOCS_DIR/ble_guide" "ble_imu_guide.tex"
    ;;
  *)
    echo "Usage: $0 [paper|ble_guide|all]"
    exit 1
    ;;
esac
