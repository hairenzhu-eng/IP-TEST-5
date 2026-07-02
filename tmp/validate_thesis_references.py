from __future__ import annotations

import json
import re
import time
import unicodedata
from pathlib import Path
from urllib.parse import quote
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "paper" / "colreg_asv_thesis_en_cited.md"
REPORT = ROOT / "literature" / "citation_validation.json"
BIB = ROOT / "literature" / "validated_references.bib"


def norm(value: str) -> str:
    value = unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode()
    return re.sub(r"[^a-z0-9]+", " ", value.lower()).strip()


def similarity(a: str, b: str) -> float:
    aa, bb = set(norm(a).split()), set(norm(b).split())
    return len(aa & bb) / max(1, len(aa | bb))


def escape_bib(value: str) -> str:
    return value.replace("\\", "\\\\").replace("{", "\\{").replace("}", "\\}")


text = SOURCE.read_text(encoding="utf-8")
reference_block = text.split("# References", 1)[1].split("\\pagebreak", 1)[0]
references = [
    line.strip()
    for line in reference_block.splitlines()
    if re.match(r"^\[\d+\]\s", line.strip())
]

results = []
bib_entries = []
for reference in references:
    number = int(re.match(r"^\[(\d+)\]", reference).group(1))
    doi_match = re.search(r"https?://doi\.org/(\S+)", reference, flags=re.I)
    if not doi_match:
        results.append(
            {
                "number": number,
                "status": "manual",
                "reference": reference,
                "reason": "No DOI; requires authoritative-source verification.",
            }
        )
        continue

    doi = doi_match.group(1).rstrip(".,)")
    url = f"https://api.crossref.org/works/{quote(doi, safe='')}"
    request = Request(url, headers={"User-Agent": "Codex citation audit/1.0 (mailto:research@example.com)"})
    try:
        with urlopen(request, timeout=20) as response:
            message = json.load(response)["message"]
        title = " ".join(message.get("title") or [])
        year_parts = (
            message.get("published-print", {}).get("date-parts")
            or message.get("published-online", {}).get("date-parts")
            or message.get("issued", {}).get("date-parts")
            or [[None]]
        )
        year = year_parts[0][0]
        title_match = re.search(r"\(\d{4}[a-z]?\)\.\s*(.+?)\.\s", reference)
        supplied_title = title_match.group(1) if title_match else reference
        score = similarity(supplied_title, title)
        status = "verified" if score >= 0.65 else "mismatch"
        results.append(
            {
                "number": number,
                "status": status,
                "doi": doi,
                "supplied_title": supplied_title,
                "crossref_title": title,
                "crossref_year": year,
                "title_similarity": round(score, 3),
                "type": message.get("type"),
                "container": (message.get("container-title") or [""])[0],
                "volume": message.get("volume"),
                "issue": message.get("issue"),
                "page": message.get("page") or message.get("article-number"),
            }
        )

        authors = []
        for author in message.get("author", []):
            family = author.get("family", "")
            given = author.get("given", "")
            authors.append(f"{family}, {given}".strip(", "))
        key_family = re.sub(r"[^A-Za-z]", "", (message.get("author") or [{}])[0].get("family", "Ref"))
        title_words = [word for word in title.split() if norm(word) not in {"a", "an", "the"}]
        key_word = re.sub(r"[^A-Za-z]", "", title_words[0] if title_words else "Work")
        key = f"{key_family}{year or 'nd'}{key_word}"
        crossref_type = message.get("type")
        if crossref_type in {"reference-book", "monograph", "book"}:
            entry_type = "book"
        elif crossref_type in {"book-chapter", "book-part"}:
            entry_type = "incollection"
        elif crossref_type == "proceedings-article":
            entry_type = "inproceedings"
        else:
            entry_type = "article"
        fields = {
            "author": " and ".join(authors),
            "title": title,
            "year": str(year or ""),
            "doi": doi,
        }
        if entry_type == "article":
            fields["journal"] = (message.get("container-title") or [""])[0]
            for src, dst in (("volume", "volume"), ("issue", "number")):
                if message.get(src):
                    fields[dst] = message[src]
            page = message.get("page") or message.get("article-number")
            if page:
                fields["pages"] = page
        elif entry_type == "book":
            fields["publisher"] = message.get("publisher", "")
            if message.get("ISBN"):
                fields["isbn"] = message["ISBN"][0]
        elif entry_type == "incollection":
            fields["booktitle"] = (message.get("container-title") or [""])[0]
            fields["publisher"] = message.get("publisher", "")
            if message.get("page"):
                fields["pages"] = message["page"]
        else:
            fields["booktitle"] = (message.get("container-title") or [""])[0]
            if message.get("page"):
                fields["pages"] = message["page"]
        field_lines = [f"  {name} = {{{escape_bib(str(value))}}}" for name, value in fields.items() if value]
        bib_entries.append(f"@{entry_type}{{{key},\n" + ",\n".join(field_lines) + "\n}")
    except Exception as exc:
        results.append(
            {
                "number": number,
                "status": "error",
                "doi": doi,
                "reference": reference,
                "reason": f"{type(exc).__name__}: {exc}",
            }
        )
    time.sleep(0.08)

REPORT.write_text(json.dumps({"total": len(references), "results": results}, indent=2, ensure_ascii=False), encoding="utf-8")
BIB.write_text("\n\n".join(bib_entries) + "\n", encoding="utf-8")
print(REPORT)
print(BIB)
