# latex-pdf-builder

Compile the LaTeX project in this repository and produce a PDF.

## Steps

Follow these steps precisely, in order:

### 1. Read the build script

Read `scripts/build_pdf.sh`. Identify:
- Which LaTeX engine is used (pdflatex / xelatex / lualatex)
- Which file is the root `.tex` file
- The output directory
- How many compilation passes are run
- Whether bibtex or biber is invoked

### 2. Inspect the LaTeX project structure

Read `docs/main.tex`. Identify:
- All `\input{}` and `\include{}` directives (subdocuments in `docs/chapters/`)
- `\bibliography{}` or `\addbibresource{}` entries (`.bib` files)
- `\graphicspath{}` setting
- Any non-standard packages that might be missing

### 3. Detect the available LaTeX engine

Run the following in order and use the first one found:
```bash
where pdflatex 2>/dev/null || \
  ls "/c/Users/$USERNAME/AppData/Local/Programs/MiKTeX/miktex/bin/x64/pdflatex.exe" 2>/dev/null || \
  echo "NOT_FOUND"
```

If no engine is found, stop and tell the user:
> "pdflatex no está instalado. Instalá MiKTeX desde https://miktex.org/download (sin necesidad de admin). Después reiniciá Git Bash y volvé a correr el skill."

### 4. Ensure the output directory exists

```bash
mkdir -p docs/output
```

### 5. Run the compilation

Execute from the `docs/` directory. Replicate exactly what `scripts/build_pdf.sh` does.

First check `docs/main.tex` for an active (non-commented) `\bibliography{}` or `\addbibresource{}` line.

**If no active bibliography** (current state of this project — `\bibliography{references}` is commented out):
```bash
cd docs
pdflatex -interaction=nonstopmode -output-directory=output main.tex
pdflatex -interaction=nonstopmode -output-directory=output main.tex
```

**If an active `\bibliography{}` or `\addbibresource{}` is present**, run the full bibtex sequence:
```bash
cd docs
pdflatex -interaction=nonstopmode -output-directory=output main.tex
bibtex output/main           # or: biber output/main (if \usepackage{biblatex})
pdflatex -interaction=nonstopmode -output-directory=output main.tex
pdflatex -interaction=nonstopmode -output-directory=output main.tex
```

### 6. Check the result

After compilation:
- Check whether `docs/output/main.pdf` was created (or updated).
- If it was created → report success and the path.
- If it was NOT created → go to step 7.

### 7. Handle errors (only if compilation failed)

Read the log file `docs/output/main.log`. Extract lines starting with `!` (fatal errors) and lines containing `LaTeX Warning:` that are relevant (undefined references, missing files, etc.).

**Common error → action mapping:**

| Error pattern | Action |
|---|---|
| `! LaTeX Error: File '...' not found` | The referenced `.tex` or image file is missing. Report it to the user with the exact filename. |
| `! Undefined control sequence` | Likely a missing package. Check which package defines it and report. |
| `! LaTeX Error: Unknown option` or `! Package ... Error` | Package conflict or wrong option. Report the package name and the exact error line. |
| `LaTeX Warning: Reference '...' undefined` | Normal on first pass — only an issue if it persists after 2 passes. |
| `LaTeX Warning: Citation '...' undefined` | Bibliography not yet compiled. Run bibtex/biber pass and retry. |
| `mktexpk` / `kpathsea` font errors | MiKTeX auto-installs fonts on first run; retry once. |

Show the user only the **relevant error lines** (not the full 1000-line log), grouped and explained in plain language.

### 8. Report final outcome

If success:
```
✓ PDF generado en: docs/output/main.pdf
```

If failure, provide:
- The 3–5 most relevant error lines from the log
- The specific fix the user needs to apply
- Whether retrying will help (e.g. font auto-install) or manual action is needed

---

## Trigger phrases (for reference)

This skill should be used when the user says things like:
- "compilar LaTeX", "generar el PDF", "build LaTeX"
- "ejecutar el script de compilación"
- "el PDF no compila", "hay errores en el .tex"

## Arguments

`$ARGUMENTS` — optional: a specific `.tex` file or subdocument to compile. If provided, use it as the root file instead of `docs/main.tex`. If empty, use `docs/main.tex`.
