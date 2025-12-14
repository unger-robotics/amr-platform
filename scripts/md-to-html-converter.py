#!/usr/bin/env python3

r"""
md-to-html-converter.py — Markdown → HTML mit Mermaid- & MathJax-Support

Zweck
-----
Konvertiert Markdown-Dateien (*.md) in eigenständige HTML-Seiten. Unterstützt
Mermaid-Diagramme und TeX-Formeln via MathJax. Optional wird eine start.html
mit Link-Übersicht erzeugt.

Hauptfunktionen (Kurzüberblick)
-------------------------------
- Pandoc-Lua-Filter (main_enhanced_filter.lua) generieren/aktualisieren:
  • Tabellen: CSS-Klassen 'table', 'table-compact'
  • Inline-Math → <span class="math math-inline">…</span>
  • reine Display-Math-Absätze → <div class="math math-display">…</div>
  • optionaler Container <div class="main-container">…</div> (einmalig)
- CSS ergänzen (main-design.css) um Regeln für Mermaid, Charts, Formeln.
- Header-Snippets:
  • Mermaid-Initialisierung (CDN, startOnLoad)
  • MathJax v3 (CDN, feste Version), Makros: \dd, \degC, \Ohm/\ohm, \degree
- Inhaltsscan:
  • Mermaid-Fences erkennen; rohe 'graph|flowchart (TD/TB/LR/RL/BT)'-Blöcke idempotent kapseln
  • Math-Delimiters außerhalb von Codeblöcken erkennen
- Optionaler Sanitizer (konservativ):
  • '{,}' → ',' (Dezimalkomma)
  • sehr kurze \text{…}-Einheiten → \mathrm{…}
  • vorsichtige $…$-Kapselung typischer TeX-Muster außerhalb von Code

Voraussetzungen
---------------
- Pandoc (≥ 2.10) im PATH
- main-design.css im Projektverzeichnis
- Internetzugang zur Anzeige (CDN für Mermaid/MathJax), nicht für die Konvertierung

Aufruf
------
  python3 md-to-html-converter.py                  # scannt Standardverzeichnisse
  python3 md-to-html-converter.py file1.md …       # konvertiert gezielte Dateien
  python3 md-to-html-converter.py --dirs docs,docs/projekt_doku --recursive
  python3 md-to-html-converter.py --no-start-page

CLI-Optionen
------------
- files (positional): Liste von .md-Dateien (optional). Überschreibt die Verzeichnissuche.
- --dirs (repeatable): Kommaseparierte Verzeichnisse. Ohne Angabe: dynamische docs/*-Topics
  (inkl. 'docs' und 'docs/projekt_doku').
- --recursive: rekursive Suche nach *.md in den angegebenen Verzeichnissen.
- --no-start-page: unterdrückt die Generierung von start.html.
- -v / --verbose: ausführlichere Konsolenmeldungen (reserviert).

Pipeline (vereinfacht)
----------------------
1) check_pandoc() → create_main_enhanced_filter() → enhance_css_file()
2) Verzeichnisse bestimmen (dynamisch oder via --dirs) → Markdown-Dateien sammeln
3) check_and_fix_content(): Mermaid-/Math-Suche; rohe Mermaid-Blöcke zu ```mermaid
4) sanitize_math_for_mathjax() (idempotent) → temporäre *.mathjax_tmp.md falls nötig
5) Pandoc-Aufruf:
   --from markdown+tex_math_dollars+tex_math_single_backslash+raw_tex
   --to html5 --standalone --mathjax --lua-filter=main_enhanced_filter.lua
   + Header-Includes (mermaid-header.html / mathjax-header.html bei Bedarf)
6) CSS neben HTML spiegeln, temporäre Dateien aufräumen
7) generate_start_page() (falls nicht --no-start-page)

Ausgabe/Artefakte
-----------------
- Für jede .md eine gleichnamige .html im selben Verzeichnis
- start.html im Projekt-Wurzelverzeichnis (Option)
- Erzeugt/überschreibt: main_enhanced_filter.lua, mathjax-header.html, mermaid-header.html
- Spiegelt main-design.css in Zielordner (idempotent)

Exit-Codes
----------
- 0   Erfolg
- 1   unerwarteter Fehler
- 130 Abbruch durch SIGINT (Strg+C)

Hinweise & Grenzen
------------------
- MathJax rendert TeX zur Laufzeit im Browser; Pandoc-Warnungen sind tolerierbar,
  wenn das Rendering korrekt ist.
- Der Sanitizer fasst Codeblöcke/Links nicht an und kapselt Math nur außerhalb
  von Fences; siunitx-spezifische Makros werden nicht automatisch unterstützt.
- Mermaid-Fences werden nicht doppelt erzeugt (idempotente Kapselung).

Lizenz/Änderungen
-----------------
- Dieses Skript überschreibt die o. g. Artefakte bei jedem Lauf.
- CSS-Erweiterung ist idempotent und per Marker erkennbar.
"""

# Standardbibliothek
import argparse
import datetime
import glob
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from shutil import which  # nur wenn in check_pandoc() genutzt

# stdlib-Module, die Objektnamen exportieren (klarer als 'import html')
import html  # bleibt ok, da du html.escape(...) nutzt
import shutil



def check_pandoc():
    """Prüft, ob Pandoc installiert ist (schnell + verifiziert)."""
    try:
        from shutil import which
        if not which('pandoc'):
            return False
        subprocess.run(['pandoc', '--version'],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        return True
    except Exception:
        return False


def create_main_enhanced_filter():
    lua_content = r"""-- main_enhanced_filter.lua – kompatibel ohne pandoc.List

-- Attribute/Classes sicherstellen (als einfache Tabellen)
local function ensure_attr(el)
  if not el.attr then el.attr = pandoc.Attr() end
  if not el.attr.classes then el.attr.classes = {} end
end

local function has_class(el, cls)
  ensure_attr(el)
  for _, c in ipairs(el.attr.classes) do
    if c == cls then return true end
  end
  return false
end

local function add_class(el, cls)
  ensure_attr(el)
  if not has_class(el, cls) then
    table.insert(el.attr.classes, cls)
  end
end

-- Tabellen: Klassen ergänzen (duplikatfrei)
function Table(tbl)
  add_class(tbl, 'table')
  add_class(tbl, 'table-compact')
  return tbl
end

-- CodeBlock: Mermaid unverändert lassen
function CodeBlock(cb)
  ensure_attr(cb)
  for _, c in ipairs(cb.attr.classes) do
    if string.lower(c) == 'mermaid' then
      return cb
    end
  end
  return cb
end

-- Inline-Math kapseln; DisplayMath unverändert
function Math(el)
  if el.mathtype == 'InlineMath' then
    local html = '<span class="math math-inline">$' .. el.text .. '$</span>'
    return pandoc.RawInline('html', html)
  end
  return el
end

-- Prüft, ob Block ausschließlich EIN DisplayMath enthält
local function sole_display_math_block(blk)
  if blk.t ~= 'Para' and blk.t ~= 'Plain' then return nil end
  local inlines = blk.content
  if #inlines ~= 1 then return nil end
  local x = inlines[1]
  if x.t == 'Math' and x.mathtype == 'DisplayMath' then
    return x.text
  end
  return nil
end

-- Reine DisplayMath-Absätze → HTML-Block
function Blocks(blocks)
  local out = {}
  for _, blk in ipairs(blocks) do
    local dm = sole_display_math_block(blk)
    if dm then
      table.insert(out, pandoc.RawBlock('html', '<div class="math math-display">$$' .. dm .. '$$</div>'))
    else
      table.insert(out, blk)
    end
  end
  return out
end

-- Hauptcontainer nur einmal einfügen
local main_container_start = '<div class="main-container">'
local main_container_end   = '</div>'

function Pandoc(doc)
  local first = doc.blocks[1]
  local last  = doc.blocks[#doc.blocks]
  local has_start = first and first.t == 'RawBlock' and first.format == 'html'
                    and first.text:match('main%-container')
  local has_end   = last and last.t == 'RawBlock' and last.format == 'html'
                    and last.text:match('^%s*</div>%s*$')
  if not has_start then
    table.insert(doc.blocks, 1, pandoc.RawBlock('html', main_container_start))
  end
  if not has_end then
    table.insert(doc.blocks, pandoc.RawBlock('html', main_container_end))
  end
  return doc
end

return {
  Table    = Table,
  CodeBlock= CodeBlock,
  Math     = Math,
  Blocks   = Blocks,
  Pandoc   = Pandoc
}
"""
    with open('main_enhanced_filter.lua', 'w', encoding='utf-8') as f:
        f.write(lua_content)




def enhance_css_file():
    """Erweitert 'main-design.css' idempotent um Regeln für Mermaid/Charts/Math."""
    css_path = Path("main-design.css")
    if not css_path.exists():
        print("Warnung: CSS-Datei 'main-design.css' nicht gefunden.")
        return

    # Ein konsistenter Block (einheitliche Werte; keine Doppelpfade)
    block_header = "/* === Visualisierungen & Math (auto-added) === */"
    visualization_and_math_css = r"""
/* === Visualisierungen & Math (auto-added) === */
/* Mermaid-Container */
.mermaid {
  display: flex;
  justify-content: center;
  margin: 1.5rem auto;
  padding: 1rem;
  background-color: white;
  border-radius: 5px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  max-width: 100%;
  overflow-x: auto;
}

/* Charts/Grafiken */
.chart-container {
  margin: 1.5rem auto;
  background-color: white;
  border-radius: 5px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  padding: 1rem;
  max-width: 100%;
}

/* Interaktive Visualisierungen */
.interactive-viz {
  width: 100%;
  height: auto;
  min-height: 400px;
  margin: 1.5rem auto;
  border: 1px solid #e0e0e0;
  border-radius: 5px;
}

/* Beschriftungen/Legenden */
.viz-caption {
  font-size: 0.9rem;
  color: #666;
  text-align: center;
  margin-top: 0.5rem;
  margin-bottom: 1.5rem;
}
.viz-legend { display: flex; flex-wrap: wrap; justify-content: center; gap: 1rem; margin-top: 1rem; font-size: 0.85rem; }
.viz-legend-item { display: flex; align-items: center; margin-right: 1rem; }
.viz-legend-color { width: 12px; height: 12px; margin-right: 5px; border-radius: 2px; }

/* Mathe (mit MathJax) */
.math { font-size: 0.95em; overflow-x: auto; max-width: 100%; padding: 0.2rem 0; }
.math-inline { display: inline-block; margin: 0 0.2rem; }
.math-display { display: block; margin: 1rem auto; text-align: center; }

/* Responsiv */
@media (max-width: 768px) {
  .mermaid, .chart-container { padding: 0.5rem; }
  .interactive-viz { min-height: 300px; }
  .math-display { font-size: 0.95em; }
}

/* Print */
@media print {
  .mermaid, .chart-container, .interactive-viz, .math-display {
    page-break-inside: avoid;
    box-shadow: none;
    border: 1px solid #e0e0e0;
  }
}
""".lstrip("\n")

    # Inhalte laden
    content = css_path.read_text(encoding="utf-8")

    # Bereits vorhanden? (erkenne Header ODER Kern-Selektoren via Regex)
    has_block_header = block_header in content
    has_mermaid = re.search(r"\.mermaid\s*\{", content) is not None
    has_math = re.search(r"\.math\s*\{", content) is not None

    if has_block_header or (has_mermaid and has_math):
        print("ℹ CSS enthält bereits Visualisierungs- und Math-Regeln")
        return

    # Atomar anhängen (Tempdatei → ersetzen), saubere Leerzeile davor
    new_content = content
    if not new_content.endswith("\n"):
        new_content += "\n"
    new_content += "\n" + visualization_and_math_css

    with tempfile.NamedTemporaryFile("w", delete=False, encoding="utf-8") as tmp:
        tmp.write(new_content)
        tmp_path = Path(tmp.name)
    shutil.move(str(tmp_path), css_path)
    print("✓ CSS-Datei um Visualisierungs- und Math-Regeln erweitert")



def check_and_fix_content(md_files):
    r"""
    Erkennung:
      - Mermaid: ```mermaid ...``` ODER rohe 'graph|flowchart <dir>'-Blöcke außerhalb von Code
      - Mathe: $...$, $$...$$, \(..\), \[..\] außerhalb von Code
    Korrektur:
      - Rohe Mermaid-Blöcke werden zu ```mermaid```-Fences umgewandelt (idempotent).
    Rückgabe: (files_with_mermaid, files_with_math, files_updated)
    """
    files_with_mermaid, files_with_math, files_updated = [], [], []

    # Regex: fenced code (``` oder ~~~), mit optionaler Sprache (nicht gierig)
    fence_start_re = re.compile(r'^(\s*)(`{3,}|~{3,})([ \t]*)(\w+)?[ \t]*$', re.MULTILINE)
    fence_any_re   = re.compile(r'^(\s*)(`{3,}|~{3,}).*$', re.MULTILINE)

    # Mermaid-Fence sicher erkennen
    mermaid_fence_re = re.compile(r'^(\s*)(`{3,}|~{3,})[ \t]*mermaid[ \t]*$', re.IGNORECASE | re.MULTILINE)

    # Rohe Mermaid-Startzeile (außerhalb von Fences)
    raw_mermaid_start_re = re.compile(
        r'^\s*(graph|flowchart)\s+(TD|TB|LR|RL|BT)\b', re.IGNORECASE | re.MULTILINE
    )

    # Math außerhalb von Code (später mit maskiertem Text prüfen)
    inline_math_re  = re.compile(r'(?<!\$)\$(?!\$).*?(?<!\$)\$(?!\$)', re.DOTALL)     # $...$ (kein $$)
    display_math_re = re.compile(
        r'\$\$.*?\$\$|\\\[(?:.|\n)*?\\\]|\\\(.*?\\\)',  # $$..$$ | \[..\] | \(..\)
        re.DOTALL
    )

    def _mask_code_blocks(text: str) -> str:
        """Ersetzt Inhalte aller Fences durch Platzhalter gleicher Länge, um Suchen außerhalb von Code zu erlauben."""
        out = []
        i = 0
        lines = text.splitlines(keepends=True)
        in_fence = False
        fence_token = ""
        for ln in lines:
            if not in_fence:
                m = fence_any_re.match(ln)
                if m:
                    in_fence = True
                    fence_token = m.group(2)[0]  # backtick oder tilde
                    out.append(ln)  # Startzeile bleibt
                else:
                    out.append(ln)
            else:
                # innerhalb Fence bis passende schließende Linie
                m = fence_any_re.match(ln)
                if m and m.group(2).startswith(fence_token):
                    in_fence = False
                    fence_token = ""
                    out.append(ln)  # Endzeile bleibt
                else:
                    # maskieren, Länge behalten
                    out.append(re.sub(r'.', ' ', ln))
        return ''.join(out)

    def _wrap_raw_mermaid(text: str) -> (str, bool, bool):
        """
        Wandelt rohe Mermaid-Abschnitte in ```mermaid``` um.
        Heuristik: ab Startzeile bis vor nächste Leerzeile oder Dateiende,
        idempotent (existierende Fences werden nicht angetastet).
        """
        if mermaid_fence_re.search(text):
            # Bereits gefenct, nur Flag setzen
            return text, True, False

        masked = _mask_code_blocks(text)
        has_mermaid = False
        modified = False

        # Finde alle Startzeilen außerhalb von Fences
        starts = [m.start() for m in raw_mermaid_start_re.finditer(masked)]
        if not starts:
            return text, has_mermaid, modified

        # Zeilenweise verarbeiten, um sauber zu kapseln
        lines = text.splitlines()
        i = 0
        while i < len(lines):
            line = lines[i]
            # außerhalb von Fences prüfen
            # Wir rekonstruieren Fence-Zustand leichtgewichtig
            if fence_any_re.match(line):
                # Skip bis Fence-Ende
                tok = fence_any_re.match(line).group(2)[0]
                i += 1
                while i < len(lines) and not (fence_any_re.match(lines[i]) and fence_any_re.match(lines[i]).group(2).startswith(tok)):
                    i += 1
                # Endzeile mitnehmen, falls vorhanden
                if i < len(lines):
                    i += 1
                continue

            if raw_mermaid_start_re.match(line):
                has_mermaid = True
                # Wenn schon unmittelbar vorher ein ```mermaid steht, nichts tun
                if i > 0 and mermaid_fence_re.match(lines[i-1] or ""):
                    i += 1
                    continue
                # Finde Blockende: bis zur nächsten komplett leeren Zeile oder Datei-Ende
                j = i + 1
                while j < len(lines) and lines[j].strip() != "":
                    # Stoppen, wenn ein neuer Fence beginnt
                    if fence_any_re.match(lines[j]):
                        break
                    j += 1
                # Kapseln
                block = lines[i:j]
                # Normiere 'graph' -> 'flowchart' (optional)
                if block and block[0].lstrip().lower().startswith('graph '):
                    block[0] = re.sub(r'^\s*graph\s+', lambda m: m.group(0).replace('graph', 'flowchart'), block[0], flags=re.IGNORECASE)
                new_block = ["```mermaid"] + block + ["```"]
                lines[i:j] = new_block
                modified = True
                # Index hinter den neu eingefügten Block setzen
                i += len(new_block)
            else:
                i += 1

        return ("\n".join(lines) + ("\n" if text.endswith("\n") else "")), has_mermaid, modified

    for md_file in md_files:
        try:
            content = Path(md_file).read_text(encoding='utf-8')
        except Exception as e:
            print(f"Fehler beim Lesen von {md_file}: {e}")
            continue

        # Mermaid: ggf. kapseln
        content_new, has_mermaid, modified = _wrap_raw_mermaid(content)

        # Math: außerhalb von Fences detektieren
        masked_for_math = _mask_code_blocks(content_new)
        has_math = bool(display_math_re.search(masked_for_math) or inline_math_re.search(masked_for_math))

        # Schreiben, wenn geändert
        if modified:
            try:
                Path(md_file).write_text(content_new, encoding='utf-8')
                files_updated.append(md_file)
            except Exception as e:
                print(f"Fehler beim Schreiben von {md_file}: {e}")
                # Falls Schreiben fehlschlägt, trotzdem Flags setzen

        if has_mermaid:
            files_with_mermaid.append(md_file)
        if has_math:
            files_with_math.append(md_file)

    # Eindeutig sortiert zurückgeben
    files_with_mermaid = sorted(set(files_with_mermaid))
    files_with_math    = sorted(set(files_with_math))
    files_updated      = sorted(set(files_updated))
    return files_with_mermaid, files_with_math, files_updated





def _natkey(s: str):
    # natürliche Sortierung: "file2" < "file10"
    parts = re.findall(r'\d+|\D+', str(s))
    return [int(t) if t.isdigit() else t.lower() for t in parts]

def _human_size(n):
    """Gibt menschenlesbare Größe aus (Basis 1024), inkl. TB-Fallback."""
    try:
        size = float(n)
    except Exception:
        return "—"
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1024 or unit == "TB":
            return f"{int(size)} B" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024.0

def generate_start_page(html_files_by_dir=None):
    """
    Erstellt eine start.html mit Links zu allen HTMLs (für alle docs/*-Sektionen)
    und den PDFs (Build, Python-Lernheft).
    """
    d = {} if html_files_by_dir is None else dict(html_files_by_dir)

    # PDFs IMMER ergänzen
    d.setdefault("build_pdfs", glob.glob("build/*.pdf") if os.path.isdir("build") else [])
    d.setdefault("python_lernheft_pdfs", glob.glob("python_lernheft/build/*.pdf") if os.path.isdir("python_lernheft/build") else [])

    # Prüfen: gibt es IRGENDWELCHE Artefakte?
    all_files = [p for lst in d.values() for p in lst]
    if not all_files:
        print("Keine Artefakte für die Startseite gefunden.")
        return

    current_time = datetime.datetime.now().strftime("%d.%m.%Y %H:%M")

    html_content = [
        "<!DOCTYPE html>",
        '<html lang="de">',
        "<head>",
        '  <meta charset="UTF-8">',
        '  <meta name="viewport" content="width=device-width, initial-scale=1.0">',
        "  <title>Dokumentation - Startseite</title>",
        '  <link rel="stylesheet" href="main-design.css">',
        "</head>",
        "<body>",
        '  <div class="main-container">',
        "    <h1>Dokumentation</h1>",
    ]

    # 1) Alle docs/*-Sektionen dynamisch (alphabetisch, natürlich sortiert)
    doc_sections = sorted((k for k in d.keys() if str(k).startswith("docs")), key=_natkey)
    for section_key in doc_sections:
        files = sorted(d.get(section_key, []), key=_natkey)
        if not files:
            continue
        title = html.escape(_pretty_title_from_dir(section_key))
        html_content.append(f"        <h3>{title}</h3>")
        html_content.append('        <ul class="documentation-list">')
        for file_path in files:
            # Pfade normalisieren und escapen
            path_norm = str(file_path).replace("\\", "/")
            title_item = Path(path_norm).stem.replace('-', ' ').replace('_', ' ')
            html_content.append(f'          <li><a href="{html.escape(path_norm)}">{html.escape(title_item)}</a></li>')
        html_content.append("        </ul>")

    # 2) PDFs
    for key, section_title in (("build_pdfs", "LaTeX-Build (PDF)"),
                               ("python_lernheft_pdfs", "Python-Lernheft (PDF)")):
        files = sorted(d.get(key, []), key=_natkey)
        if not files:
            continue
        html_content.append(f"        <h3>{html.escape(section_title)}</h3>")
        html_content.append('        <ul class="documentation-list pdf-list">')
        for file_path in files:
            path_norm = str(file_path).replace("\\", "/")
            title = Path(path_norm).stem.replace('-', ' ').replace('_', ' ')
            try:
                size = _human_size(os.path.getsize(path_norm))
                meta = f'<span class="file-meta">{html.escape(size)}</span>'
            except Exception:
                meta = ""
            html_content.append(f'          <li><a href="{html.escape(path_norm)}">📄 {html.escape(title)}</a> {meta}</li>')
        html_content.append("        </ul>")

    html_content += [
        "        <footer>",
        f"          <p>bearbeitet am {html.escape(current_time)}</p>",
        "        </footer>",
        "      </div>",
        "    </body>",
        "    </html>",
    ]

    with open('start.html', 'w', encoding='utf-8') as f:
        f.write("\n".join(html_content))
    print("✓ Startseite 'start.html' erfolgreich erstellt")






# --- Schönere Titel aus Pfaden ---
def _pretty_title_from_dir(d: str) -> str:
    """
    'docs/projekt_doku' -> 'Projekt-Doku''
    'docs/e_auto'       -> 'Docs / E-Auto'
    Sonst: 'Docs / <Titelkette>' bzw. Rohpfad, falls nicht unter docs/.
    """
    p = d.strip("/").replace("\\", "/").split("/")
    if not p:
        return d

    if p[0] != "docs":
        return d  # Fremdpfade unverändert

    if len(p) == 1:
        return "Docs"

    # Mapping für bekannte Slugs
    custom = {
        "konfig": "konfig",
        "robotik-engineering": "robotik-engineering",
    }

    def slug_to_title(seg: str) -> str:
        s = seg.lower()
        if s in custom:
            return custom[s]
        # Allgemein: '_' -> ' ', jedes Wort Capitalize, Ziffern bleiben
        s = seg.replace("_", " ")
        # Unicode-sicheres Capitalize jedes Wort
        return " ".join(w[:1].upper() + w[1:] if w else w for w in s.split(" "))

    tail_titles = [slug_to_title(seg) for seg in p[1:]]
    return "Docs / " + " / ".join(tail_titles)

# --- Verzeichnisse normalisieren ---
def _normalize_dirs(dir_list):
    """
    - Trimmt Leerzeichen, normiert Slashes
    - Entfernt Duplikate bei Erhalt der Reihenfolge
    - Lässt nur existierende Verzeichnisse passieren
    """
    seen = set()
    out = []
    for d in dir_list or []:
        dd = str(d).strip().strip("/").replace("\\", "/")
        if not dd:
            continue
        if dd in seen:
            continue
        if os.path.isdir(dd):
            out.append(dd)
            seen.add(dd)
        else:
            # Optional: hier bewusst leise; bei Bedarf warnen
            # print(f"Info: Verzeichnis nicht gefunden und ignoriert: {dd}")
            pass
    return out

# --- CSS in Zielordner spiegeln ---
def ensure_css_in_dir(target_dir: Path, css_file: str):
    """
    Kopiert CSS in target_dir, falls es dort fehlt oder älter ist.
    Wichtig, weil Pandoc ohne Pfad auf 'main-design.css' verweist.
    """
    try:
        target_dir.mkdir(parents=True, exist_ok=True)
        src = Path(css_file)
        if not src.exists():
            print(f"Warnung: CSS-Quelle '{css_file}' nicht gefunden.")
            return
        dst = target_dir / src.name
        # Kopiere, wenn Ziel fehlt oder Quelle neuer ist
        if (not dst.exists()) or (src.stat().st_mtime > dst.stat().st_mtime + 1e-6):
            shutil.copy2(src, dst)
            # Optional: kurze Meldung nur bei tatsächlicher Aktion
            # print(f"✓ CSS aktualisiert in {target_dir}")
    except Exception as e:
        print(f"Warnung: CSS konnte nicht nach '{target_dir}' gespiegelt werden: {e}")

def ensure_css_for_md_dirs(md_dirs, css_file="main-design.css"):
    """
    Spiegelt CSS in alle Basisverzeichnisse. Beim Konvertieren wird zusätzlich
    pro HTML-Zielordner noch einmal ensure_css_in_dir() aufgerufen.
    """
    src = Path(css_file)
    if not src.exists():
        print(f"Warnung: CSS-Datei '{css_file}' nicht gefunden.")
        return
    for d in md_dirs:
        try:
            ensure_css_in_dir(Path(d), css_file)
        except Exception as e:
            print(f"Warnung: CSS-Kopie nach '{d}' fehlgeschlagen: {e}")



def sanitize_math_for_mathjax(text: str) -> str:
    r"""
    Konservative Auto-Korrekturen für häufige Pandoc/MathJax-Warnungen:
    - '{,}' → ',' (Dezimalkomma)
    - \text{…} bleibt i. d. R. erhalten (MathJax v3+ams kann \text)
    - optionale Umstellung kurzer Einheiten-Tokens auf \mathrm{…}
    - vorsichtige Kapselung von TeX-Mustern in $…$, außer in Code/Links
    """
    import re

    # Early exit: Nichts tun bei leer
    if not text:
        return text

    # --- Hilfsfunktionen/Flags ---
    fenced_re = re.compile(r"^\s*(```|~~~)")
    inline_code_re = re.compile(r"(`+)(.+?)\1")  # `code` oder ``code``
    url_re = re.compile(r"https?://")
    has_math_delim = lambda s: ("$" in s) or (r"\[" in s) or (r"\(" in s)
    tex_marker_re = re.compile(
        r"\\(frac|xrightarrow|dot|ddot|Delta|eta|rho|mathrm|sum|int|approx|sqrt|cdot|overline|underline|vec|bar)\b"
    )

    # Sehr kurze Einheiten-Tokens (optional -> \mathrm)
    unit_token_re = re.compile(r"\\text\{([A-Za-zµΩ]{1,4})\}")

    in_fence = False
    out_lines = []

    for raw_line in text.splitlines():
        line = raw_line

        # Fence-Start/Ende?
        if fenced_re.match(line):
            in_fence = not in_fence
            out_lines.append(line)
            continue

        # 1) Dezimalkomma: {,} → ,
        if not in_fence:
            line = line.replace("{,}", ",")

        # 2) \text{TOKEN} -> \mathrm{TOKEN} (nur sehr kurze Einheiten), sonst \text belassen
        def _units_text_to_rm(m):
            token = m.group(1)
            return r"\mathrm{" + token + "}"
        if not in_fence:
            line = unit_token_re.sub(_units_text_to_rm, line)

        # 3) Vorsichtige Kapselung: nur wenn
        #    - nicht in fenced code
        #    - keine Math-Delims vorhanden
        #    - kein Inline-Code-Segment
        #    - keine URL in der Zeile
        if (not in_fence
            and not has_math_delim(line)
            and not inline_code_re.search(line)
            and not url_re.search(line)
            and tex_marker_re.search(line)):
            line = f"${line}$"

        out_lines.append(line)

    return "\n".join(out_lines)


def create_mathjax_header():
    """Erstellt und gibt den Dateinamen einer temporären MathJax-Headerdatei zurück (v3, mit Makros)."""
    mathjax_header = r"""
<script>
window.MathJax = {
  loader: {
    load: ['[tex]/ams']  // explizit laden
  },
  tex: {
    inlineMath: [['$','$'], ['\\(','\\)']],
    displayMath: [['$$','$$'], ['\\[','\\]']],
    processEscapes: true,
    processEnvironments: true,
    packages: {'[+]': ['ams']},
    macros: {
      dd: '{\\,\\mathrm{d}}',
      degC: '{^{\\circ}\\!\\mathrm{C}}',
      Ohm: '{\\,\\Omega}',      // \Ohm →  \,\Omega
      ohm: '{\\,\\Omega}',      // Alias
      degree: '{^{\\circ}}'     // nur °, z. B. 30\degree
    }
  },
  options: {
    skipHtmlTags: ['script','noscript','style','textarea','pre','code','kbd','samp']
  }
};
</script>
<script defer src="https://cdn.jsdelivr.net/npm/mathjax@3.2.2/es5/tex-mml-chtml.js"></script>
"""
    with open('mathjax-header.html', 'w', encoding='utf-8') as f:
        f.write(mathjax_header)
    return 'mathjax-header.html'




def convert_all_markdown_files(args):
    """Konvertiert alle *.md-Dateien mithilfe von Pandoc und dem Lua-Filter."""
    assert callable(create_mathjax_header), "create_mathjax_header() nicht definiert oder nicht callable"

    if not check_pandoc():
        print("Fehler: Pandoc ist nicht installiert. Bitte installieren oder Pfad korrigieren.")
        return

    create_main_enhanced_filter()  # Lua-Filter erzeugen/aktualisieren
    enhance_css_file()             # CSS ergänzen

    css_file = "main-design.css"
    if not os.path.exists(css_file):
        print(f"Warnung: CSS-Datei '{css_file}' nicht gefunden.")

    # --- NEU: Verzeichnisse bestimmen (dynamisch statt statisch) ---
    import re
    def _slugify(name: str) -> str:
        name = (name.replace("ä","ae").replace("ö","oe").replace("ü","ue")
                    .replace("Ä","Ae").replace("Ö","Oe").replace("Ü","Ue")
                    .replace("ß","ss")).strip()
        name = re.sub(r"[^\w\s\-]", "_", name)  # Sonderzeichen -> _
        name = re.sub(r"\s+", "_", name)        # Leerzeichen -> _
        name = re.sub(r"_+", "_", name)         # Mehrfach-Underscores bündeln
        return name.lower()

    # Quelle: deine Themenliste
    ordner = [
        "konfig", "robotik-engineering"
    ]

    base_dirs   = {"docs"}  # fixe Basismappen
    topic_dirs  = {f"docs/{_slugify(x)}" for x in ordner}
    default_dirs = sorted(base_dirs | topic_dirs)

    # args.dirs (Komma-getrennt) überschreibt optional
    md_dirs = default_dirs[:]
    if args.dirs:
        user_dirs = []
        for spec in args.dirs:
            user_dirs.extend([p.strip() for p in spec.split(",") if p.strip()])
        md_dirs = user_dirs or default_dirs

    md_dirs = _normalize_dirs(md_dirs)
    if not md_dirs:
        print("Keine gültigen Markdown-Verzeichnisse gefunden.")
        return

    # Sammle alle Markdown-Dateien
    if args.files:
        markdown_files = [f for f in args.files if f.endswith('.md') and os.path.isfile(f)]
    else:
        markdown_files = []
        pattern = "**/*.md" if args.recursive else "*.md"
        for md_dir in md_dirs:
            if os.path.isdir(md_dir):
                markdown_files.extend(glob.glob(os.path.join(md_dir, pattern), recursive=args.recursive))

    if not markdown_files:
        print("Keine Markdown-Dateien zum Konvertieren gefunden.")
        return

    print(f"Gefundene Markdown-Dateien: {len(markdown_files)}")

    files_with_mermaid, files_with_math, files_updated = check_and_fix_content(markdown_files)
    if files_with_mermaid:
        print(f"Dateien mit Mermaid-Diagrammen erkannt: {len(files_with_mermaid)}")
    if files_with_math:
        print(f"Dateien mit TeX-Formeln erkannt: {len(files_with_math)}")
    if files_updated:
        print(f"Syntax in {len(files_updated)} Dateien korrigiert")

    # CSS in Basismappen spiegeln
    ensure_css_for_md_dirs(md_dirs, css_file=css_file)

    mermaid_header_file = 'mermaid-header.html'
    mathjax_header_file = create_mathjax_header()

    with open(mermaid_header_file, 'w', encoding='utf-8') as f:
        f.write(r"""
<script src="https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js"></script>
<script>
  document.addEventListener('DOMContentLoaded', function() {
    mermaid.initialize({
      startOnLoad: true,
      theme: 'default',
      flowchart: {
        useMaxWidth: true,
        htmlLabels: true,
        curve: 'basis',
        nodeSpacing: 50,
        rankSpacing: 70
      }
    });
  });
</script>
""")

    # Erfolgreich konvertierte HTML-Dateien nach Verzeichnis ordnen
    html_files_by_dir = {}

    for md_file in markdown_files:
        html_file = Path(md_file).with_suffix('.html')

        # Kategorie = echter Ordnername (z. B. "docs/phase1")
        target_dir = os.path.dirname(md_file).replace("\\", "/")
        html_category = target_dir if target_dir not in ("", ".") else "root"
        html_files_by_dir.setdefault(html_category, [])

        # Optional: Eingabe sanitizen → temp-Datei
        tmp_input = md_file
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                raw = f.read()
            sanitized = sanitize_math_for_mathjax(raw)
            if sanitized != raw:
                tmp_input = str(Path(md_file).with_suffix('.mathjax_tmp.md'))
                with open(tmp_input, 'w', encoding='utf-8') as f:
                    f.write(sanitized)
        except Exception:
            tmp_input = md_file

        cmd = [
            'pandoc',
            tmp_input,
            '-o', str(html_file),
            '--standalone',
            '--to=html5',
            '--from=markdown+tex_math_dollars+tex_math_single_backslash+raw_tex',
            '--mathjax',
            '--css', css_file,
            '--lua-filter=main_enhanced_filter.lua'
        ]
        if md_file in files_with_mermaid:
            cmd.append(f'--include-in-header={mermaid_header_file}')
        if md_file in files_with_math:
            cmd.append(f'--include-in-header={mathjax_header_file}')

        try:
            print(f"Konvertiere {md_file} zu {html_file}...")
            subprocess.run(cmd, check=True)
            print(f"✓ Erfolgreich konvertiert: {html_file}")
            html_files_by_dir[html_category].append(str(html_file))

            # NEU: CSS sicherheitshalber auch NEBEN die HTML spiegeln (für Unterordner)
            ensure_css_in_dir(Path(html_file).parent, css_file)

        except subprocess.CalledProcessError as e:
            print(f"✗ Fehler bei der Konvertierung von {md_file}: {e}")
        finally:
            if tmp_input != md_file and os.path.exists(tmp_input):
                os.remove(tmp_input)

    for temp_file in [mermaid_header_file, mathjax_header_file]:
        if os.path.exists(temp_file):
            os.remove(temp_file)

    if not args.no_start_page:
        # 'root' absichtlich NICHT automatisch listen – Fokus auf docs/*
        html_files_by_dir.pop("root", None)
        generate_start_page(html_files_by_dir)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Konvertiert Markdown-Dateien zu HTML (Mermaid + MathJax).'
    )
    parser.add_argument(
        'files', nargs='*',
        help='Spezifische Markdown-Dateien (optional). Wenn angegeben, überschreibt dies die Verzeichnissuche.'
    )
    parser.add_argument(
        '--no-start-page', action='store_true',
        help='Keine Startseite generieren.'
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Ausführliche Ausgabe.'
    )
    parser.add_argument(
        '--dirs', action='append',
        help=('Komma-separierte Verzeichnisse; kann mehrfach angegeben werden. '
              'Ohne Angabe werden thematische docs/*-Ordner dynamisch bestimmt '
              '(inkl. docs und docs/projekt_doku).')
    )
    parser.add_argument(
        '--recursive', action='store_true',
        help='Rekursiv in Unterordnern nach *.md suchen.'
    )

    args = parser.parse_args()

    try:
        convert_all_markdown_files(args)
        return 0
    except KeyboardInterrupt:
        print("Abgebrochen (Strg+C).")
        return 130  # üblicher Code für SIGINT
    except Exception as e:
        # Fallback: nicht ausufern, aber CI-tauglich einen Fehlercode liefern
        print(f"Unerwarteter Fehler: {e}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())
