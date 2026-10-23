"""Verify the delivered source/data bytes; no third-party dependency required."""
from pathlib import Path
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1]

def main():
    manifest=ROOT/'MANIFEST.sha256.json'
    if not manifest.exists():print('No release manifest in this development tree.');return 2
    data=json.loads(manifest.read_text(encoding='utf-8'));fail=[]
    for rel,digest in data['files'].items():
        p=ROOT/rel
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:fail.append(rel)
    print(json.dumps({'checked':len(data['files']),'mismatches':fail},ensure_ascii=False))
    return 1 if fail else 0
if __name__=='__main__':raise SystemExit(main())
