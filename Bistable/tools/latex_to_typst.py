#!/usr/bin/env python3
"""Convert the Rolly bistable LaTeX note to a Typst draft.

This is a project-specific converter, not a general LaTeX parser. It handles the
environments and macros used by bistable_math_derivation.tex and emits a Typst
document plus a small BibTeX file.
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEX_PATH = ROOT / "bistable_math_derivation.tex"
TYP_PATH = ROOT / "bistable_math_derivation.typ"
BIB_PATH = ROOT / "bistable_math_derivation.bib"


BIB_ENTRIES = {
    "nayfeh1979": """@book{nayfeh1979,
  author = {Nayfeh, Ali H. and Mook, Dean T.},
  title = {Nonlinear Oscillations},
  publisher = {Wiley-Interscience},
  address = {New York},
  year = {1979}
}
""",
    "denhartog1956": """@book{denhartog1956,
  author = {Den Hartog, J. P.},
  title = {Mechanical Vibrations},
  edition = {4},
  publisher = {McGraw-Hill},
  address = {New York},
  year = {1956}
}
""",
    "qiu2003": """@patent{qiu2003,
  author = {Qiu, J. and Slocum, A. H. and Lang, J. H. and Struempler, R. and Brenner, M. P. and Li, J.},
  title = {Bistable Actuation Techniques, Mechanisms, and Applications},
  number = {US 2003/0029705 A1},
  date = {2003-02}
}
""",
    "qiu2004": """@article{qiu2004,
  author = {Qiu, J. and Lang, J. H. and Slocum, A. H.},
  title = {A Curved-Beam Bistable Mechanism},
  journal = {Journal of Microelectromechanical Systems},
  volume = {13},
  number = {2},
  pages = {137--146},
  year = {2004},
  doi = {10.1109/JMEMS.2004.825308}
}
""",
    "zolfagharian2025": """@article{zolfagharian2025,
  author = {Zolfagharian, A. and Demoly, F. and Lakhi, M. and Rolfe, B. and Bodaghi, M.},
  title = {Bistable Mechanisms 3D Printing for Mechanically Programmable Vibration Control},
  journal = {Advanced Engineering Materials},
  volume = {28},
  number = {9},
  pages = {2402233},
  year = {2026},
  doi = {10.1002/adem.202402233}
}
""",
    "liu2023": """@article{liu2023,
  author = {Liu, Y. and Zhang, J. and Pan, D. and Wu, Z. and Wang, Q.},
  title = {Resonant Actuation Based on Dynamic Characteristics of Bistable Laminates},
  journal = {Machines},
  volume = {11},
  number = {3},
  pages = {318},
  year = {2023},
  doi = {10.3390/machines11030318}
}
""",
    "robstride2026": """@manual{robstride2026,
  author = {{RobStride}},
  title = {RS05 User Manual},
  edition = {Rev. 260428},
  year = {2026}
}
""",
}


LABEL_PREFIX = {
    "sec": "section",
    "ssec": "section",
    "sssec": "section",
    "eq": "equation",
    "fig": "figure",
    "tab": "table",
}


def typ_label(label: str) -> str:
    return label.replace(":", "-").replace("_", "-")


def ref_text(label: str) -> str:
    prefix = label.split(":", 1)[0]
    if prefix in {"sec", "ssec", "sssec"}:
        return "section"
    noun = LABEL_PREFIX.get(prefix, "item")
    return f"{noun} @{' '.join([typ_label(label)]).strip()}"


def convert_inline_math(text: str) -> str:
    def repl(match: re.Match[str]) -> str:
        content = convert_math(match.group(1))
        return f"${content}$"

    return re.sub(r"\$(.+?)\$", repl, text)


def convert_citations(text: str) -> str:
    def repl(match: re.Match[str]) -> str:
        keys = [k.strip() for k in match.group(1).split(",")]
        return "[" + "; ".join(f"@{key}" for key in keys) + "]"

    return re.sub(r"~?\\cite\{([^}]+)\}", repl, text)


def convert_refs(text: str) -> str:
    def repl(match: re.Match[str]) -> str:
        labels = [label.strip() for label in match.group(2).split(",")]
        converted = [ref_text(label) for label in labels]
        return " and ".join(converted) if len(converted) <= 2 else ", ".join(converted)

    text = re.sub(
        r"(Sections|sections)[~ ]\\ref\{([^}]+)\}(?:--|-)\\ref\{([^}]+)\}",
        lambda m: "the relevant sections",
        text,
    )
    text = re.sub(r"(Section|section)~\\ref\{([^}]+)\}", repl, text)
    def generic_repl(match: re.Match[str]) -> str:
        labels = [label.strip() for label in match.group(1).split(",")]
        converted = [ref_text(label) for label in labels]
        return " and ".join(converted) if len(converted) <= 2 else ", ".join(converted)

    text = re.sub(r"\\(?:cref|ref)\{([^}]+)\}", generic_repl, text)
    return text


def convert_text_markup(text: str) -> str:
    text = text.replace("---", "-")
    text = text.replace("--", "-")
    text = text.replace("``", '"').replace("''", '"')
    text = text.replace(r"\%", "%")
    text = text.replace(r"\_", "_")
    text = text.replace(r"\,", " ")
    text = text.replace(r"\;", " ")
    text = text.replace(r"\ ", " ")
    text = text.replace("~", " ")
    text = replace_texorpdfstring(text)
    text = replace_text_commands(text)
    text = re.sub(r"\\textbf\{([^{}]*)\}", r"*\1*", text, flags=re.S)
    text = re.sub(r"\\textit\{([^{}]*)\}", r"_\1_", text, flags=re.S)
    text = re.sub(r"\\emph\{([^{}]*)\}", r"_\1_", text, flags=re.S)
    text = re.sub(r"\\texttt\{([^{}]*)\}", r"`\1`", text, flags=re.S)
    text = re.sub(r"\\url\{([^{}]+)\}", r"#link(\"\1\")", text)
    text = re.sub(r"\\href\{([^{}]+)\}\{([^{}]+)\}", r"#link(\"\1\")[\2]", text)
    text = convert_citations(text)
    text = convert_refs(text)
    text = convert_inline_math(text)
    text = re.sub(r'([A-Za-z])\$_\("([^"]+)"\)\$', r'$\1_("\2")$', text)
    return text


def convert_math(math: str) -> str:
    s = math.strip()
    s = re.sub(
        r"\\underbrace\{(.*?)\}_\{\\text\{(.*?)\}\}",
        lambda m: f"underbrace({convert_math(m.group(1))}, \"{m.group(2).strip()}\")",
        s,
        flags=re.S,
    )
    s = replace_math_commands(s)
    s = re.sub(r"\\ddot\\([A-Za-z]+)", lambda m: f"dot.double({convert_math('\\' + m.group(1))})", s)
    s = re.sub(r"\\dot\\([A-Za-z]+)", lambda m: f"dot({convert_math('\\' + m.group(1))})", s)
    s = s.replace(r"\leftrightarrow", " arrow.l.r ")
    s = s.replace(r"\uparrow", " arrow.t ")
    s = s.replace(r"\downarrow", " arrow.b ")
    s = s.replace(r"\Longleftrightarrow", " <=> ")
    s = s.replace(r"\Longrightarrow", " => ")
    s = s.replace(r"\longrightarrow", " -> ")
    s = s.replace(r"\rightarrow", " -> ")
    s = s.replace(r"\to", " -> ")
    s = s.replace(r"\left", "").replace(r"\right", "")
    for size_cmd in (r"\bigl", r"\bigr", r"\Bigl", r"\Bigr", r"\big", r"\Big"):
        s = s.replace(size_cmd, "")
    s = s.replace(r"\!", "")
    s = s.replace(r"\,", " ")
    s = s.replace(r"\;", " ")
    s = s.replace(r"\qquad", " ")
    s = s.replace(r"\quad", " ")
    s = s.replace(r"\notag", "")
    s = s.replace(r"\mathrel{+}=", "+=")
    s = s.replace(r"\times", " times ")
    s = s.replace(r"\cdot", " dot ")
    s = s.replace(r"\leq", " <= ")
    s = s.replace(r"\geq", " >= ")
    s = s.replace(r"\ll", " << ")
    s = s.replace(r"\gg", " >> ")
    s = s.replace(r"\gtrsim", " >= ")
    s = s.replace(r"\lesssim", " <= ")
    s = s.replace(r"\approx", " approx ")
    s = s.replace(r"\sim", " tilde ")
    s = s.replace(r"\propto", " prop ")
    s = s.replace(r"\infty", " infinity ")
    s = s.replace(r"\equiv", " equiv ")
    s = s.replace(r"\textwidth", "100%")
    s = s.replace(r"\%", "%")
    s = s.replace(r"\&", "&")
    s = s.replace(r"\pm", " plus.minus ")
    s = s.replace(r"\circ", " degree ")
    s = s.replace(r"\top", "^T")
    s = re.sub(r"\\diff(?![A-Za-z])", "dif", s)
    s = re.sub(r"\\int(?![A-Za-z])", "integral", s)
    s = re.sub(r"\\partial(?![A-Za-z])", "partial", s)
    s = re.sub(r"\\sin\b", "sin", s)
    s = re.sub(r"\\cos\b", "cos", s)
    s = re.sub(r"\\pi\b", "pi", s)
    s = re.sub(r"\\Delta\b", "Delta", s)
    s = re.sub(r"\\delta\b", "delta", s)
    s = re.sub(r"\\zeta\b", "zeta", s)
    s = re.sub(r"\\omega\b", "omega", s)
    s = re.sub(r"\\varphi\b", "phi", s)
    s = re.sub(r"\\xi\b", "xi", s)
    s = re.sub(r"\\theta\b", "theta", s)
    s = re.sub(r"\\Theta\b", "Theta", s)
    s = re.sub(r"\\alpha\b", "alpha", s)
    s = re.sub(r"\\tau\b", "tau", s)
    s = re.sub(r"\\mu\b", "mu", s)
    s = re.sub(r"\\begin\{bmatrix\}(.*?)\\end\{bmatrix\}", lambda m: convert_matrix(m.group(1), bracket="["), s, flags=re.S)
    s = re.sub(r"\\begin\{Bmatrix\}(.*?)\\end\{Bmatrix\}", lambda m: convert_matrix(m.group(1), bracket="{"), s, flags=re.S)
    s = re.sub(r"\\deriv\{([^{}]+)\}\{([^{}]+)\}", r"(dif \1)/(dif \2)", s)
    s = re.sub(r"\\pderiv\{([^{}]+)\}\{([^{}]+)\}", r"(partial \1)/(partial \2)", s)
    s = re.sub(r"\\[a-zA-Z]+", lambda m: m.group(0)[1:], s)
    s = normalize_subscripts(s)
    s = normalize_products(s)
    return s.strip()


def find_balanced(text: str, start: int) -> tuple[str, int] | None:
    if start >= len(text) or text[start] != "{":
        return None
    depth = 0
    content_start = start + 1
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[content_start:i], i + 1
    return None


def replace_text_commands(text: str) -> str:
    s = text
    for command, left, right in (
        ("textbf", "*", "*"),
        ("textit", "_", "_"),
        ("emph", "_", "_"),
        ("texttt", "`", "`"),
    ):
        needle = "\\" + command + "{"
        i = 0
        out = []
        while True:
            pos = s.find(needle, i)
            if pos < 0:
                out.append(s[i:])
                break
            out.append(s[i:pos])
            arg = find_balanced(s, pos + len(command) + 1)
            if not arg:
                out.append(s[pos:pos + len(needle)])
                i = pos + len(needle)
                continue
            content, end = arg
            converted = convert_text_markup(content)
            if command == "texttt":
                converted = converted.replace("`", "")
            out.append(f"{left}{converted}{right}")
            i = end
        s = "".join(out)
    return s


def flatten_latex_text_commands(text: str) -> str:
    s = text
    for command in ("textbf", "textit", "emph", "texttt"):
        needle = "\\" + command + "{"
        i = 0
        out = []
        while True:
            pos = s.find(needle, i)
            if pos < 0:
                out.append(s[i:])
                break
            out.append(s[i:pos])
            arg = find_balanced(s, pos + len(command) + 1)
            if not arg:
                out.append(s[pos:pos + len(needle)])
                i = pos + len(needle)
                continue
            content, end = arg
            content = " ".join(content.splitlines())
            out.append(f"\\{command}" + "{" + content + "}")
            i = end
        s = "".join(out)
    return s


def replace_one_arg_command(text: str, command: str, fn) -> str:
    needle = "\\" + command + "{"
    i = 0
    out = []
    while True:
        pos = text.find(needle, i)
        if pos < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:pos])
        arg = find_balanced(text, pos + len(command) + 1)
        if not arg:
            out.append(text[pos:pos + len(needle)])
            i = pos + len(needle)
            continue
        content, end = arg
        out.append(fn(content))
        i = end


def replace_two_arg_command(text: str, command: str, fn) -> str:
    needle = "\\" + command + "{"
    i = 0
    out = []
    while True:
        pos = text.find(needle, i)
        if pos < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:pos])
        first = find_balanced(text, pos + len(command) + 1)
        if not first:
            out.append(text[pos:pos + len(needle)])
            i = pos + len(needle)
            continue
        a, p2 = first
        second = find_balanced(text, p2)
        if not second:
            out.append(text[pos:p2])
            i = p2
            continue
        b, end = second
        out.append(fn(a, b))
        i = end


def replace_math_commands(text: str) -> str:
    prev = None
    s = text
    while s != prev:
        prev = s
        s = replace_two_arg_command(s, "frac", lambda a, b: f"({convert_math(a)})/({convert_math(b)})")
        s = replace_one_arg_command(s, "sqrt", lambda a: f"sqrt({convert_math(a)})")
        s = replace_one_arg_command(s, "mathrm", lambda a: '"' + a.replace('"', r'\"') + '"')
        s = replace_one_arg_command(s, "text", lambda a: '"' + a.replace('"', r'\"') + '"')
        s = replace_one_arg_command(s, "mathbf", lambda a: f"bold({convert_math(a)})")
        s = replace_one_arg_command(s, "bm", lambda a: f"bold({convert_math(a)})")
        s = replace_one_arg_command(s, "dot", lambda a: f"dot({convert_math(a)})")
        s = replace_one_arg_command(s, "ddot", lambda a: f"dot.double({convert_math(a)})")
        s = replace_one_arg_command(s, "hat", lambda a: f"hat({convert_math(a)})")
        s = replace_one_arg_command(s, "bar", lambda a: f"macron({convert_math(a)})")
        s = replace_one_arg_command(s, "boxed", lambda a: convert_math(a))
    return s


def normalize_subscripts(text: str) -> str:
    def braced_repl(match: re.Match[str]) -> str:
        op = match.group(1)
        content = match.group(2).strip()
        if op == "_":
            if len(content) >= 2 and content[0] == '"' and content[-1] == '"':
                content = content[1:-1]
            content = content.replace('"', "")
            return f'_("{content}")'
        return f"^({content})"

    s = re.sub(r"([_^])\{([^{}]+)\}", braced_repl, text)
    s = re.sub(r'_"([^"]+)"', lambda m: f'_("{m.group(1)}")', s)
    s = re.sub(r'\^"([^"]+)"', lambda m: f'^("{m.group(1)}")', s)
    s = re.sub(r"_([A-Za-z])(?=dot\.)", r"_\1 ", s)
    s = re.sub(r"_([A-Za-z])(?=dot\()", r"_\1 ", s)
    s = re.sub(r"_([A-Za-z])(?=(pi|xi|omega|theta|Theta|Delta|delta|zeta|tau|mu|alpha|dif|sin|cos|sqrt|dot|bold|hat|macron))", r"_\1 ", s)
    s = re.sub(r"_([A-Za-z]{2,})", lambda m: f'_("{m.group(1)}")', s)
    return s


def normalize_products(text: str) -> str:
    words = r"(pi|xi|omega|theta|Theta|Delta|delta|zeta|tau|mu|alpha|dif|sin|cos|sqrt|dot|bold|hat|macron)"
    prev = None
    s = text
    while s != prev:
        prev = s
        s = re.sub(rf"(\d)({words})", r"\1 \2", s)
        s = re.sub(rf"\b([A-Za-z])({words})", r"\1 \2", s)
        s = re.sub(rf"\b(sin|cos)({words})", r"\1 \2", s)
        s = re.sub(r"([A-Za-z0-9)\]\"])(integral)", r"\1 \2", s)
        s = re.sub(r"([A-Za-z0-9)\]\"])(dot\.)", r"\1 \2", s)
        s = re.sub(r"([A-Za-z0-9)\]\"])(dot\()", r"\1 \2", s)
        s = re.sub(r"\b([A-Z])([a-z]_)", r"\1 \2", s)
        s = re.sub(r"([A-Z])([A-Z]_)", r"\1 \2", s)
        s = re.sub(r"([a-z]_\([^)]+\))\(", r"\1 (", s)
        s = re.sub(rf"(pi|xi|omega|theta|Theta|Delta|delta|zeta|tau|mu|alpha|dif)({words})", r"\1 \2", s)
        s = re.sub(r"\b([A-Z])([A-Z])\b", r"\1 \2", s)
    return s


def replace_texorpdfstring(text: str) -> str:
    needle = r"\texorpdfstring{"
    i = 0
    out = []
    while True:
        pos = text.find(needle, i)
        if pos < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:pos])
        first = find_balanced(text, pos + len(r"\texorpdfstring"))
        if not first:
            out.append(text[pos:])
            return "".join(out)
        a, p2 = first
        second = find_balanced(text, p2)
        if not second:
            out.append(text[pos:p2])
            i = p2
            continue
        math = a.strip()
        if math.startswith("$") and math.endswith("$"):
            math = math[1:-1]
        out.append(f"${convert_math(math)}$")
        i = second[1]


def convert_matrix(body: str, bracket: str) -> str:
    rows = []
    for row in body.strip().split(r"\\"):
        cells = [convert_math(cell.strip()) for cell in row.split("&")]
        rows.append("(" + ", ".join(cells) + ")")
    delim = "mat" if bracket == "[" else "mat"
    return f"{delim}({', '.join(rows)})"


def collect_environment(lines: list[str], start: int, env: str) -> tuple[list[str], int]:
    out = []
    i = start + 1
    depth = 1
    while i < len(lines):
        line = lines[i]
        if f"\\begin{{{env}}}" in line:
            depth += 1
        if f"\\end{{{env}}}" in line:
            depth -= 1
            if depth == 0:
                return out, i
        out.append(line)
        i += 1
    return out, i


def extract_label_caption(lines: list[str]) -> tuple[str | None, str | None]:
    label = None
    caption = None
    joined = "\n".join(lines)
    lm = re.search(r"\\label\{([^}]+)\}", joined)
    if lm:
        label = lm.group(1)
    pos = joined.find(r"\caption{")
    if pos >= 0:
        found = find_balanced(joined, pos + len(r"\caption"))
        if found:
            caption = " ".join(convert_text_markup(found[0]).split())
    return label, caption


def convert_figure(lines: list[str]) -> list[str]:
    label, caption = extract_label_caption(lines)
    joined = "\n".join(lines)
    image = re.search(r"\\includegraphics(?:\[[^\]]+\])?\{([^}]+)\}", joined)
    if image:
        path = image.group(1)
        if not path.startswith("figures/"):
            path = f"figures/sim_excitation/{path}"
        body = f'image("{path}", width: 100%)'
    elif r"\begin{verbatim}" in joined:
        raw = re.search(r"\\begin\{verbatim\}(.*?)\\end\{verbatim\}", joined, flags=re.S).group(1).strip("\n")
        body = "```text\n" + raw + "\n```"
    elif r"\begin{subfigure}" in joined:
        imgs = re.findall(r"\\includegraphics(?:\[[^\]]+\])?\{([^}]+)\}", joined)
        cells = []
        for img in imgs:
            path = img if img.startswith("figures/") else f"figures/sim_excitation/{img}"
            cells.append(f'[image("{path}", width: 100%)]')
        body = "grid(columns: (1fr, 1fr), gutter: 12pt,\n  " + ",\n  ".join(cells) + "\n)"
    else:
        body = "[Unsupported figure content]"
    label_part = f" <{typ_label(label)}>" if label else ""
    cap = f", caption: [{caption}]" if caption else ""
    return [f"#figure({body}{cap}){label_part}", ""]


def convert_table(lines: list[str]) -> list[str]:
    label, caption = extract_label_caption(lines)
    joined = "\n".join(lines)
    tm = re.search(r"\\begin\{(?:tabular|longtable)\}\{[^}]*\}(.*?)\\end\{(?:tabular|longtable)\}", joined, flags=re.S)
    if not tm:
        return ["#figure([Unsupported table content])", ""]
    body = tm.group(1)
    body = re.sub(r"\\caption\{.*?\}", "", body, flags=re.S)
    body = re.sub(r"\\label\{[^}]+\}", "", body)
    body = re.sub(r"\\(?:toprule|midrule|bottomrule|endfirsthead|endhead|endfoot|endlastfoot)", "", body)
    body = re.sub(r"\\multicolumn\{\d+\}\{[^}]+\}\{(.*?)\}", r"\1", body)
    rows = []
    for raw_row in re.split(r"\\\\", body):
        raw_row = raw_row.strip()
        if not raw_row:
            continue
        if raw_row.startswith(r"\midrule") or raw_row.startswith(r"\toprule"):
            continue
        raw_row = raw_row.replace(r"\hline", "").strip()
        cells = [convert_text_markup(cell.strip()) for cell in raw_row.split("&")]
        rows.append(cells)
    cols = max((len(row) for row in rows), default=1)
    typ_lines = ["table(", f"  columns: {cols},"]
    for row in rows:
        row = row + [""] * (cols - len(row))
        typ_lines.extend(f"  [{cell}]," for cell in row)
    typ_lines.append(")")
    label_part = f" <{typ_label(label)}>" if label else ""
    cap = f", caption: [{caption}]" if caption else ""
    return [f"#figure({chr(10).join(typ_lines)}{cap}){label_part}", ""]


def convert_equation(lines: list[str], env: str) -> list[str]:
    if env == "align":
        joined = "\n".join(lines)
        pieces = re.split(r"\\\\", joined)
        out = []
        for piece in pieces:
            piece = piece.strip()
            if not piece:
                continue
            lm = re.search(r"\\label\{([^}]+)\}", piece)
            label = lm.group(1) if lm else None
            piece = re.sub(r"\\label\{[^}]+\}", "", piece).strip()
            if not piece:
                continue
            label_part = f" <{typ_label(label)}>" if label else ""
            out.append(f"$ {convert_math(piece)} ${label_part}")
            out.append("")
        return out

    label = None
    body_lines = []
    for line in lines:
        lm = re.search(r"\\label\{([^}]+)\}", line)
        if lm:
            label = lm.group(1)
            line = re.sub(r"\\label\{[^}]+\}", "", line)
        body_lines.append(line.rstrip())
    body = "\n".join(body_lines).strip()
    body = convert_math(body)
    if env == "align":
        body = body.replace(r"\\", "\n")
    label_part = f" <{typ_label(label)}>" if label else ""
    return [f"$ {body} ${label_part}", ""]


def convert_bibliography(lines: list[str]) -> list[str]:
    return ['#bibliography("bistable_math_derivation.bib", title: [References])', ""]


def preamble() -> list[str]:
    return [
        "#set page(paper: \"us-letter\", margin: 1.1in)",
        "#set text(size: 11pt, font: \"New Computer Modern\")",
        "#show heading: set block(above: 1.0em, below: 0.5em)",
        "#set heading(numbering: none)",
        "#set math.equation(numbering: \"(1)\")",
        "",
        "#align(center)[",
        "  #text(size: 18pt, weight: \"bold\")[Mathematical Derivation of Bistable Mechanisms\\\\",
        "  for Vibration-Triggered Hook Deployment]",
        "",
        "  #v(0.5em)",
        "  #text(size: 13pt)[Rolly Robot - Hook Bistable Actuator Simulations]",
        "",
        "  #v(0.8em)",
        "  Technical note for the Rolly hook-deployment simulations\\\\",
        "  2026",
        "]",
        "",
        "#outline(title: [Contents])",
        "#pagebreak()",
        "",
    ]


def convert_document(tex: str) -> list[str]:
    body = tex.split(r"\begin{document}", 1)[1].split(r"\end{document}", 1)[0]
    body = flatten_latex_text_commands(body)
    lines = body.splitlines()
    out = preamble()
    enum_stack: list[str] = []
    skip_until_bib = False
    i = 0
    while i < len(lines):
        line = lines[i].rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("%") or stripped in {r"\maketitle", r"\tableofcontents", r"\newpage"}:
            i += 1
            continue
        if stripped == r"\begin{thebibliography}{99}":
            env_lines, end = collect_environment(lines, i, "thebibliography")
            out.extend(convert_bibliography(env_lines))
            i = end + 1
            continue
        if stripped.startswith(r"\section*{"):
            title = re.search(r"\\section\*\{(.+)\}", stripped).group(1)
            out.append(f"= {convert_text_markup(title)}")
            out.append("")
            i += 1
            continue
        for cmd, marks in [(r"\section", "="), (r"\subsection", "=="), (r"\subsubsection", "===")]:
            if stripped.startswith(cmd + "{"):
                title = re.search(re.escape(cmd) + r"\{(.+)\}", stripped).group(1)
                out.append(f"{marks} {convert_text_markup(title)}")
                i += 1
                if i < len(lines) and lines[i].strip().startswith(r"\label{"):
                    lab = re.search(r"\\label\{([^}]+)\}", lines[i].strip()).group(1)
                    out[-1] += f" <{typ_label(lab)}>"
                    i += 1
                out.append("")
                break
        else:
            if stripped.startswith(r"\paragraph{"):
                title = re.search(r"\\paragraph\{(.+)\}", stripped).group(1)
                out.append(f"*{convert_text_markup(title)}*")
                out.append("")
                i += 1
                continue
            if stripped.startswith(r"\label{"):
                i += 1
                continue
            if stripped.startswith(r"\begin{equation}"):
                env_lines, end = collect_environment(lines, i, "equation")
                out.extend(convert_equation(env_lines, "equation"))
                i = end + 1
                continue
            if stripped.startswith(r"\begin{align}"):
                env_lines, end = collect_environment(lines, i, "align")
                out.extend(convert_equation(env_lines, "align"))
                i = end + 1
                continue
            if stripped.startswith(r"\begin{figure}"):
                env_lines, end = collect_environment(lines, i, "figure")
                out.extend(convert_figure(env_lines))
                i = end + 1
                continue
            if stripped.startswith(r"\begin{table}"):
                env_lines, end = collect_environment(lines, i, "table")
                out.extend(convert_table(env_lines))
                i = end + 1
                continue
            if stripped.startswith(r"\begin{longtable}"):
                env_lines, end = collect_environment(lines, i, "longtable")
                out.extend(convert_table(env_lines))
                i = end + 1
                continue
            if stripped.startswith(r"\begin{enumerate}"):
                enum_stack.append("+")
                i += 1
                continue
            if stripped.startswith(r"\begin{itemize}"):
                enum_stack.append("-")
                i += 1
                continue
            if stripped.startswith(r"\end{enumerate}") or stripped.startswith(r"\end{itemize}"):
                if enum_stack:
                    enum_stack.pop()
                out.append("")
                i += 1
                continue
            if stripped.startswith(r"\item"):
                bullet = enum_stack[-1] if enum_stack else "-"
                item = re.sub(r"\\item\s*", "", stripped)
                out.append(f"{bullet} {convert_text_markup(item)}")
                i += 1
                continue
            if stripped.startswith(r"\begin{center}"):
                env_lines, end = collect_environment(lines, i, "center")
                if any(r"\begin{tabular}" in line for line in env_lines):
                    out.extend(convert_table(env_lines))
                    i = end + 1
                    continue
                out.append("#align(center)[")
                out.extend("  " + convert_text_markup(l.strip()) for l in env_lines if l.strip())
                out.append("]")
                out.append("")
                i = end + 1
                continue
            if stripped.startswith("$$"):
                if stripped.endswith("$$") and len(stripped) > 4:
                    out.append(f"$ {convert_math(stripped[2:-2])} $")
                    out.append("")
                    i += 1
                    continue
                math_lines = [stripped.strip("$")]
                i += 1
                while i < len(lines) and not lines[i].strip().endswith("$$"):
                    math_lines.append(lines[i].rstrip())
                    i += 1
                if i < len(lines):
                    math_lines.append(lines[i].strip().strip("$"))
                    i += 1
                out.append(f"$ {convert_math(chr(10).join(math_lines))} $")
                out.append("")
                continue
            if stripped.startswith(r"\noindent"):
                stripped = stripped.replace(r"\noindent", "", 1).strip()
            if stripped.startswith(r"\begingroup") or stripped.startswith(r"\endgroup") or stripped.startswith(r"\renewcommand") or stripped == r"\small":
                i += 1
                continue
            out.append(convert_text_markup(stripped))
            i += 1
            continue
        continue
    return out


def main() -> None:
    tex = TEX_PATH.read_text()
    typ = "\n".join(convert_document(tex)).replace("\n\n\n", "\n\n")
    TYP_PATH.write_text(typ + "\n")
    BIB_PATH.write_text("\n".join(BIB_ENTRIES.values()))


if __name__ == "__main__":
    main()
