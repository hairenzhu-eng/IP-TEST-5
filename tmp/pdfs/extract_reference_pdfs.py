from pathlib import Path
from pypdf import PdfReader


PDFS = {
    "scoping": Path(r"F:\Desk\research project 2026\final paper\MSc project Scoping study .pdf"),
    "example1": Path(r"F:\Desk\research project 2026\final paper\Paper plus Example 1.pdf"),
    "example2": Path(r"F:\Desk\research project 2026\final paper\Paper plus Example 2.pdf"),
}


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    for key, source in PDFS.items():
        reader = PdfReader(str(source))
        chunks = []
        for page_number, page in enumerate(reader.pages, start=1):
            chunks.append(f"\n\n===== PAGE {page_number} =====\n\n")
            chunks.append(page.extract_text() or "")
        target = output_dir / f"{key}.txt"
        target.write_text("".join(chunks), encoding="utf-8")
        print(f"{key}: {len(reader.pages)} pages -> {target}")


if __name__ == "__main__":
    main()
