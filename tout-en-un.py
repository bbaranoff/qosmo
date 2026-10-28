#!/usr/bin/env python3
"""tout-en-un.py — gathers the .log .sh .md .mmd .py .txt of one or more
directories into ONE Markdown file, shortened WITHOUT LOSS:

  * identical files (same content): a single copy, the others refer to the
    first one;
  * identical consecutive lines: "line  ×N";
  * consecutive lines that only differ by numbers (timestamp, counter, fn...):
    a template where the constant numbers stay in place and the variable
    numbers become ⟨1⟩ ⟨2⟩..., followed by the ordered list of values. Each
    line is rebuilt by putting the values back in order.
  * ANSI colour codes and carriage returns removed (the only loss, deliberate).

    ./tout-en-un.py [dir...]      default: the qosmo overlay (rebuilt by make-overlay.sh
                                  unless run from the overlay)
    SORTIE=/path.md  EXT="log sh md mmd py txt c h"  SEUIL=3 (minimum group size)
"""
import hashlib
import os
import re
import subprocess
import sys
import time

EXT = os.environ.get("EXT", "log sh md mmd py txt c h").split()
SEUIL = int(os.environ.get("SEUIL", "3"))
FORMAT = os.environ.get("FORMAT", "qmd")           # qmd (Quarto) or md
SORTIE = os.environ.get("SORTIE") or "/root/qosmo-%s.%s" % (time.strftime("%Y%m%d-%H%M%S"), FORMAT)
AUTEUR = os.environ.get("AUTEUR", "Banc GSM émulé — banc-max")
IGNORES = {".git", "__pycache__", ".pytest_cache", "node_modules"}
DSP_DUMP = re.compile(r"(^calypso_dsp.*\.txt|^dsp-rom-.*\.txt|\.dis)$")   # DSP ROM dumps (not ours, megabytes of hex)
HTML_SANS_COULEUR = 2_500_000                      # bytes: larger bundles get no syntax highlighting in html
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
TELNET = re.compile(rb"\xff[\xfb-\xfe].|\xff.")           # telnet negotiation (IAC) of the VTY captures
CONTROLE = re.compile("[\x00-\x08\x0b\x0c\x0e-\x1f\x7f\ufffd]")  # control characters, undecodable bytes
NUM = re.compile(r"\d+")
LANG = {"sh": "bash", "py": "python", "md": "markdown", "mmd": "mermaid", "txt": "text", "log": "text",
        "c": "c", "h": "c"}


def dossiers_par_defaut():
    """The qosmo overlay (only the files specific to qosmo, not vanilla QEMU).
    Run from an overlay, the overlay is this directory; run from the qosmo
    tree, make-overlay.sh rebuilds ../qosmo-overlay first."""
    ici = os.path.dirname(os.path.realpath(__file__))
    if os.path.exists(os.path.join(ici, "MANIFEST.ajoutes")):
        overlay = ici
    else:
        overlay = os.path.join(os.path.dirname(ici), "qosmo-overlay")
        subprocess.run([os.path.join(ici, "make-overlay.sh"), overlay], check=True, stdout=subprocess.DEVNULL)
    return [overlay]




def rendu_quarto(racine, d):
    """d is the resource directory of a page rendered by Quarto (<name>_files next to <name>.html)."""
    return d.endswith("_files") and os.path.exists(os.path.join(racine, d[:-len("_files")] + ".html"))


def fichiers(dossier):
    for racine, dirs, noms in os.walk(dossier):
        dirs[:] = sorted(d for d in dirs if d not in IGNORES and not rendu_quarto(racine, d))
        for n in sorted(noms):
            if n.startswith("tout-en-un-"):      # our own outputs
                continue
            if DSP_DUMP.search(n):
                continue
            if n.rsplit(".", 1)[-1].lower() in EXT and "." in n:
                yield os.path.join(racine, n)


def compacter(lignes):
    """Returns (output lines, number of compacted groups)."""
    out, groupes, i, n = [], 0, 0, len(lignes)
    while i < n:
        cle = NUM.sub("\0", lignes[i])
        j = i + 1
        while j < n and NUM.sub("\0", lignes[j]) == cle:
            j += 1
        taille = j - i
        if taille < SEUIL:
            out.extend(lignes[i:j])
            i = j
            continue
        groupes += 1
        if "\0" not in cle:                      # exact repetition
            out.append("%s  ×%d" % (lignes[i], taille))
            i = j
            continue
        valeurs = [NUM.findall(l) for l in lignes[i:j]]
        nb = len(valeurs[0])
        varie = [len({v[k] for v in valeurs}) > 1 for k in range(nb)]
        # template: constant numbers put back, variable ones numbered ⟨k⟩
        morceaux, k, idx = cle.split("\0"), 0, 0
        gab = morceaux[0]
        for m in morceaux[1:]:
            if varie[k]:
                idx += 1
                gab += "⟨%d⟩" % idx
            else:
                gab += valeurs[0][k]
            gab += m
            k += 1
        out.append("%s  ×%d" % (gab, taille))
        if idx:
            tuples = [",".join(v[k] for k in range(nb) if varie[k]) for v in valeurs]
            # value lines of ~100 columns
            ligne, courant = [], "    ⟨⟩ ="
            for t in tuples:
                if len(courant) + len(t) + 3 > 110:
                    ligne.append(courant)
                    courant = "       "
                courant += " (" + t + ")"
            ligne.append(courant)
            out.extend(ligne)
        i = j
    return out, groupes


def synthese(tous):
    """Run, scale, failures, coverage summary: read from verdict.txt / couverture.txt."""
    lignes, vus = [], set()
    for f in tous:
        nom = os.path.basename(f)
        if nom not in ("verdict.txt", "couverture.txt"):
            continue
        run = os.path.basename(os.path.dirname(f))
        if (run, nom) in vus:                 # same run copied in two places
            continue
        vus.add((run, nom))
        try:
            txt = ANSI.sub("", open(f, "rb").read().decode("utf-8", "replace"))
        except OSError:
            continue
        for l in txt.splitlines():
            l = re.sub(r"\s+", " ", l.strip())
            if l.startswith("ELEMENT MAX"):
                lignes.append((0, "| %s | échelle | %s |" % (run, l)))
            elif l.startswith("bilan :"):
                lignes.append((2, "| %s | couverture | %s |" % (run, l[8:])))
            elif l.startswith("couverture couche 1"):
                lignes.append((1, "| %s | mode, date | %s |" % (run, l.split("—", 1)[-1].strip())))
            else:
                m = re.match(r"(\d+) (\S+) (ECHEC|SAUTE) ?(.*)", l)
                if m:
                    lignes.append((3, "| %s | barreau %s %s | %s %s |" % (run, m.group(1), m.group(2), m.group(3), m.group(4))))
    return [l for _, l in sorted(lignes, key=lambda x: x[0])]


MERMAID_OUVRE = "```{mermaid}" if FORMAT == "qmd" else "```mermaid"


def rendre_mmd(lignes):
    return "%s\n%s\n```" % (MERMAID_OUVRE, "\n".join(lignes))


_DEPOTS = {}
QOSMO_GITHUB = "https://github.com/bbaranoff/qosmO"   # overlay without its source tree next to it


def depot_git(chemin):
    """(root, GitHub base URL, branch, name) of the repository holding chemin,
    or None. A qosmo overlay (directory with MANIFEST.ajoutes, made by
    make-overlay.sh) stands for the qosmo repository: same relative paths."""
    d = os.path.abspath(chemin)
    if not os.path.isdir(d):
        d = os.path.dirname(d)
    while not (os.path.exists(os.path.join(d, ".git")) or os.path.exists(os.path.join(d, "MANIFEST.ajoutes"))):
        if d == "/":
            return None
        d = os.path.dirname(d)
    if d not in _DEPOTS:
        source = d
        if not os.path.exists(os.path.join(d, ".git")):         # overlay: its source tree sits next to it
            source = os.path.join(os.path.dirname(d), "qosmo")
        def git(*a):
            r = subprocess.run(["git", "-C", source] + list(a), capture_output=True, text=True)
            return r.stdout.strip() if r.returncode == 0 else ""
        url = git("config", "--get", "remote.origin.url") or (QOSMO_GITHUB if source != d else "")
        url = re.sub(r"^git@([^:]+):", r"https://\1/", url)
        url = re.sub(r"\.git$", "", url).rstrip("/")
        branche = git("rev-parse", "--abbrev-ref", "HEAD")
        _DEPOTS[d] = (d, url, branche if branche and branche != "HEAD" else "main",
                      os.path.basename(source)) if url else None
    return _DEPOTS[d]


def adresse(chemin):
    """GitHub address of chemin (blob/<branch>/<path>), or the local path outside a repository."""
    g = depot_git(chemin)
    if not g:
        return chemin
    rel = os.path.relpath(os.path.abspath(chemin), g[0])
    return "%s/blob/%s/%s" % (g[1], g[2], "" if rel == "." else rel)


def reference(chemin):
    """File reference used in titles: a GitHub link when the file lives in a repository."""
    a = adresse(chemin)
    if not a.startswith("http"):
        return a
    # Quarto opens external links in a new tab (link-external-newwindow);
    # plain Markdown needs an HTML anchor for that
    return "<%s>" % a if FORMAT == "qmd" else '<a href="%s" target="_blank">%s</a>' % (a, a)


def nom_depot():
    """Name of the repository this script belongs to (title of the bundle)."""
    ici = os.path.dirname(os.path.realpath(__file__))
    g = depot_git(ici)
    return g[3] if g else os.path.basename(ici)


LIEN = re.compile(r"(?<!!)(\[[^\]]*\]\()([^)\s]+)((?:\s+\"[^\"]*\")?\))")
LIEN_REF = re.compile(r"^(\s*\[[^\]]+\]:\s*)(\S+)(.*)$")


def lien_github(cible, md):
    """Rewrite a link target of the .md file md to its GitHub address; others stay as they are."""
    if re.match(r"^([a-zA-Z][\w+.-]*:|#|<)", cible):
        return cible
    chemin, _, ancre = cible.partition("#")
    if not chemin:
        return cible
    if not chemin.startswith("/"):
        chemin = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(md)), chemin))
    a = adresse(chemin)
    if not a.startswith("http"):
        return cible
    return a + ("#" + ancre if ancre else "")


def lien_md(m, md):
    cible = lien_github(m.group(2), md)
    if FORMAT != "qmd" and cible.startswith("http"):  # plain Markdown: new tab via an HTML anchor
        return '<a href="%s" target="_blank">%s</a>' % (cible, m.group(1)[1:-2])
    return m.group(1) + cible + m.group(3)


IMAGE = re.compile(r"(!\[[^\]]*\]\()([^)\s]+)((?:\s+\"[^\"]*\")?\))")


def image_locale(cible, md):
    """Relative image of the .md file md: path relative to the output file, so the page
    embeds it offline (Quarto reads absolute paths as project-relative)."""
    if re.match(r"^([a-zA-Z][\w+.-]*:|/|#)", cible):
        return cible
    chemin = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(md)), cible))
    return os.path.relpath(chemin, os.path.dirname(os.path.abspath(SORTIE))) if os.path.exists(chemin) else cible


def liens_github(l, md):
    l = IMAGE.sub(lambda m: m.group(1) + image_locale(m.group(2), md) + m.group(3), l)
    l = LIEN.sub(lambda m: lien_md(m, md), l)
    return LIEN_REF.sub(lambda m: m.group(1) + lien_github(m.group(2), md) + m.group(3), l)


def rendre_md(lignes, md):
    out, dans_bloc, cloture = [], False, ""
    i = 0
    if lignes and lignes[0].strip() == "---":           # YAML front matter: shown as a block, not interpreted
        j = next((k for k in range(1, len(lignes)) if lignes[k].strip() in ("---", "...")), None)
        if j:
            out.append("```yaml"); out.extend(lignes[1:j]); out.append("```")
            i = j + 1
    while i < len(lignes):
        l = lignes[i]; i += 1
        m = re.match(r"^(\s*)(`{3,}|~{3,})(.*)$", l)
        if m and not dans_bloc:
            dans_bloc, cloture = True, m.group(2)
            info = m.group(3).strip()
            if info.startswith("{mermaid}") or info.startswith("mermaid"):
                l = m.group(1) + MERMAID_OUVRE
            elif info.startswith("{"):                     # ```{r ...}: never executed
                l = m.group(1) + m.group(2) + info.strip("{}").split(" ")[0].split(",")[0]
            out.append(l); continue
        if m and dans_bloc and m.group(2)[0] == cloture[0] and len(m.group(2)) >= len(cloture) and not m.group(3).strip():
            dans_bloc = False; out.append(l); continue
        if not dans_bloc:
            l = liens_github(l, md)
            h = re.match(r"^(#{1,6})\s", l)
            if h:
                l = "#" * min(6, len(h.group(1)) + 3) + l[len(h.group(1)):]
            elif l.strip() in ("---", "..."):             # no YAML block nor rule in the middle of the dossier
                l = "* * *"
        out.append(l)
    if dans_bloc:
        out.append(cloture)
    return "\n".join(out)


def main():
    dossiers = sys.argv[1:] or dossiers_par_defaut()
    vus, sections, table = {}, [], []
    total_in = total_out = 0
    # Order: test results (.txt .md .mmd: verdicts, coverage, reports, grafcets)
    # first, then the bench files (.sh .py), the .log last
    CATEGORIES = (("Verdict et couverture", ()), ("Resultats des tests", ("txt", "md", "mmd")),
                  ("Fichiers", ("sh", "py")), ("Logs", ("log",)))
    EN_TETE = ("verdict.txt", "couverture.txt")     # at the very top, in this order
    tous = []
    for d in dossiers:
        if not os.path.isdir(d):
            table.append("| (absent) %s | | | | |" % d)
            continue
        tous.extend(fichiers(d))
    def rang(f):
        if os.path.basename(f) in EN_TETE:
            return 0
        e = f.rsplit(".", 1)[-1].lower()
        for i, (_, exts) in enumerate(CATEGORIES):
            if e in exts:
                return i
        return len(CATEGORIES)
    categorie_courante = None
    def cle(f):
        r = rang(f)
        return (r, EN_TETE.index(os.path.basename(f)) if r == 0 else 0, tous.index(f))
    for f in sorted(tous, key=cle):
        if rang(f) != categorie_courante:
            categorie_courante = rang(f)
            nom = CATEGORIES[categorie_courante][0] if categorie_courante < len(CATEGORIES) else "Autres"
            sections.append("## %s\n" % nom)
        if True:
            try:
                brut = open(f, "rb").read()
            except OSError as e:
                table.append("| %s | | | | illisible : %s |" % (f, e))
                continue
            texte = ANSI.sub("", TELNET.sub(b"", brut).decode("utf-8", "replace")).replace("\r", "")
            texte = CONTROLE.sub("", texte)
            lignes = texte.split("\n")
            if lignes and lignes[-1] == "":
                lignes.pop()
            h = hashlib.sha1(texte.encode()).hexdigest()
            total_in += len(brut)
            if h in vus:
                sections.append("### %s\n\nidentique à %s\n" % (reference(f), reference(vus[h])))
                continue
            vus[h] = f
            ext = f.rsplit(".", 1)[-1].lower()
            if ext in ("md", "mmd"):
                # Rendered, not quoted: .mmd become Mermaid diagrams, .md are
                # included as is (headings demoted under the file title, mermaid
                # blocks rendered, {r}/{python} blocks neutralised). No compaction.
                total_out += len(texte) + 1
                sections.append("### %s\n\n%d octets, %d lignes\n\n%s\n"
                                % (reference(f), len(brut), len(lignes), rendre_md(lignes, f) if ext == "md" else rendre_mmd(lignes)))
                continue
            compact, groupes = compacter(lignes)
            total_out += sum(len(l) + 1 for l in compact)
            note = "%d groupes compactés" % groupes if groupes else ""
            # fence longer than any run of ` in the content: a ``` in a .py or
            # a .md no longer closes the block (pandoc would otherwise read what
            # follows as Markdown: "Could not fetch resource")
            plus_long = max((len(m) for m in re.findall(r"`+", "\n".join(compact))), default=0)
            cloture = "`" * max(3, plus_long + 1)
            sections.append("### %s\n\n%d octets, %d lignes → %d lignes%s\n\n%s%s\n%s\n%s\n"
                            % (reference(f), len(brut), len(lignes), len(compact), (" (%s)" % note) if note else "",
                               cloture, LANG.get(ext, "text"), "\n".join(compact), cloture))
    runs = sorted({os.path.basename(os.path.dirname(f)) for f in tous if os.path.basename(f) == "verdict.txt"})
    with open(SORTIE, "w") as o:
        o.write("---\n")
        o.write("title: \"%s\"\n" % nom_depot())
        o.write("generator: tout-en-un.py\n")
        o.write("subtitle: \"%s\"\n" % (", ".join(runs) if runs else ", ".join(dossiers)))
        o.write("author: \"%s\"\n" % AUTEUR)
        o.write("date: \"%s\"\n" % time.strftime("%Y-%m-%d %H:%M"))
        o.write("lang: fr\n")
        if FORMAT == "qmd":
            o.write("engine: markdown\n")     # pandoc only: knitr would see ```{r} in the included reports and stop
        if FORMAT == "qmd":
            o.write("format:\n  html:\n    toc: true\n    toc-depth: 3\n    toc-location: left\n"
                    "    link-external-newwindow: true\n"
                    "    number-sections: true\n    embed-resources: true\n    theme: cosmo\n"
                    "    code-overflow: wrap\n    fontsize: 0.9em\n")
            # Highlighting wraps every token in a <span>; past ~2.5 MB the DOM is
            # too large for Quarto's HTML post-processing (RangeError: Maximum call
            # stack size exceeded), so large bundles are rendered without it
            if total_out > HTML_SANS_COULEUR:
                o.write("    highlight-style: none\n")
        o.write("---\n\n")
        o.write("## Synthèse\n\n")
        o.write("| run | élément | valeur |\n|---|---|---|\n")
        lignes_syn = synthese(tous)
        o.write("\n".join(lignes_syn) + "\n\n" if lignes_syn else "| (pas de verdict.txt) | | |\n\n")
        o.write("| dossier de run | contenu | |\n|---|---|---|\n")
        o.write("| Verdict et couverture | échelle des barreaux et tableau de couverture couche 1 | |\n")
        o.write("| Résultats des tests | captures VTY, diagnostics, rapports pytest, grafcets (.txt .md .mmd) | |\n")
        o.write("| Fichiers | scripts du banc et des tests (.sh .py) | |\n")
        o.write("| Logs | journaux (.log), en dernier | |\n\n")
        o.write("::: {.callout-note collapse=\"true\"}\n## Méthode\n\n" if FORMAT == "qmd" else "### Méthode\n\n")
        o.write("Dossiers : %s. Extensions : %s. Entrée %d Ko, sortie %d Ko.\n\n"
                % (", ".join(dossiers), " ".join(EXT), total_in // 1024, total_out // 1024))
        o.write("Compactage sans perte : fichiers identiques cités une fois ; lignes consécutives identiques "
                "« ×N » ; lignes ne différant que par des nombres : gabarit avec ⟨k⟩ puis la liste ordonnée "
                "des valeurs (k-uplets), chaque ligne se reconstruit en remettant les valeurs dans l'ordre. "
                "Seuls les codes couleur, la négociation telnet des captures VTY et les caractères de commande "
                "sont retirés. Les .md sont inclus tels quels (titres rétrogradés) et les .mmd rendus en "
                "diagrammes Mermaid, sans compactage.\n")
        o.write(":::\n\n" if FORMAT == "qmd" else "\n")
        for t in table:                      # only absent directories / unreadable files remain
            o.write(t.strip("| ").split(" |")[0] + "\n")
        o.write("\n")
        o.write("\n".join(sections))
    print("%s : %d fichiers (%d uniques), %d Ko -> %d Ko" % (SORTIE, len(tous), len(vus), total_in // 1024,
                                                            os.path.getsize(SORTIE) // 1024))


if __name__ == "__main__":
    main()
