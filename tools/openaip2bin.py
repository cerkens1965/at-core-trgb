#!/usr/bin/env python3
"""
OpenAIP → AT-VIEW binary AIP files
Downloads CTR polygons + aerodromes and converts to compact .bin for SD upload.

Usage:
    python3 tools/openaip2bin.py --key YOUR_API_KEY
    python3 tools/openaip2bin.py --key YOUR_API_KEY --countries FR BE NL LU
    python3 tools/openaip2bin.py --key YOUR_API_KEY --dump-json  # debug

Register free at https://www.openaip.net to get an API key.

Output (in public/aip/):
    {country}_ctr.bin      CTR polygons per country
    aerodromes.bin         All aerodromes (excluding heliports & hydrobases)

Then upload via AT-VIEW WiFi AP: http://atview.local or http://192.168.4.1
"""

import argparse
import json
import math
import os
import struct
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import NamedTuple, Optional

# ── Config ──────────────────────────────────────────────────────────────────

API_BASE    = "https://api.core.openaip.net/api"
RATE_S      = 0.4        # seconds between API calls (max 3 req/s)
EPSILON_CTR = 0.004      # Douglas-Peucker epsilon in degrees (~400m)
MAX_RETRIES = 3

COUNTRY_NAMES = {
    "FR": "france",
    "BE": "belgium",
    "NL": "netherlands",
    "LU": "luxembourg",
    "DE": "germany",
    "CH": "switzerland",
    "GB": "uk",
    "ES": "spain",
    "IT": "italy",
    "AT": "austria",
}

# Airspace type IDs to include
# 4=CTR (Control Zone, icaoClass=3), 13=ATZ (Aerodrome Traffic Zone, icaoClass=6)
AIRSPACE_TYPE_IDS = {4, 13}

# Airport type IDs to EXCLUDE
# OpenAIP actual type values (verified against data):
#   7 = heliport / hospital helipad
#  10 = hydrobase / water aerodrome
# Everything else is kept: aerodromes (2), airports (3), ULM (6), military (5),
#   glider (4), certified (9), private (1), small (8), etc.
EXCLUDE_AP_TYPES = {7, 10}

# Name-based heliport safety net — catches heliports with wrong type ID
HELI_WORDS = ['HELIPORT','HELIPAD','HELISTATION','HELISURFACE','HÉLISURFACE',
              'HOPITAL ','HÔPITAL ','HOSPITAL ','KLINIKUM','KRANKENHAUS']


# ── Data types ───────────────────────────────────────────────────────────────

class Pt(NamedTuple):
    lat: float
    lon: float


# ── HTTP helpers ─────────────────────────────────────────────────────────────

def api_get(path: str, api_key: str, params: dict, dump_dir: Optional[str]) -> dict:
    url = f"{API_BASE}/{path}?" + urllib.parse.urlencode(params)
    for attempt in range(MAX_RETRIES):
        try:
            req = urllib.request.Request(url, headers={
                "x-openaip-api-key": api_key,
                "Accept":            "application/json",
                "User-Agent":        "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36",
                "Accept-Language":   "en-US,en;q=0.9",
            })
            with urllib.request.urlopen(req, timeout=30) as r:
                data = json.loads(r.read())
            if dump_dir:
                slug = path.replace("/", "_") + "_" + urllib.parse.urlencode(params).replace("&", "_")
                with open(os.path.join(dump_dir, f"{slug}.json"), "w") as f:
                    json.dump(data, f, indent=2)
            return data
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors="replace")
            print(f"  HTTP {e.code} on {url}: {body[:200]}")
            if e.code in (401, 403):
                raise SystemExit("Invalid API key — register at https://www.openaip.net")
            if e.code == 429:
                print(f"  Rate limited, waiting 5s...")
                time.sleep(5)
            elif attempt == MAX_RETRIES - 1:
                raise
        except Exception as e:
            print(f"  Error: {e}")
            if attempt == MAX_RETRIES - 1:
                raise
        time.sleep(RATE_S * (attempt + 1))
    return {}


def fetch_all(path: str, api_key: str, params: dict, dump_dir: Optional[str]) -> list:
    items = []
    page  = 1
    while True:
        data = api_get(path, api_key, {**params, "page": page, "limit": 100}, dump_dir)
        # Handle both pagination formats used by OpenAIP
        batch = (data.get("items")
              or data.get("data")
              or (data if isinstance(data, list) else []))
        items.extend(batch)
        total = (data.get("totalCount")
              or data.get("total")
              or data.get("count")
              or len(items))
        print(f"    page {page}: +{len(batch)} (total {total})")
        if not batch or len(items) >= int(total):
            break
        page += 1
        time.sleep(RATE_S)
    return items


# ── Geometry ──────────────────────────────────────────────────────────────────

def outer_ring(geometry: dict) -> list[Pt]:
    """Extract outer ring from GeoJSON Polygon or MultiPolygon."""
    gtype = geometry.get("type", "")
    if gtype == "Polygon":
        ring = geometry["coordinates"][0]
    elif gtype == "MultiPolygon":
        # Pick largest ring
        ring = max(geometry["coordinates"], key=lambda p: len(p[0]))[0]
    else:
        return []
    return [Pt(lat=c[1], lon=c[0]) for c in ring]


def _perp(p: Pt, a: Pt, b: Pt) -> float:
    """Perpendicular distance from p to line a-b (in degrees)."""
    if a == b:
        return math.hypot(p.lat - a.lat, p.lon - a.lon)
    dx = b.lon - a.lon
    dy = b.lat - a.lat
    mag = math.sqrt(dx * dx + dy * dy)
    return abs(dy * p.lon - dx * p.lat + b.lon * a.lat - b.lat * a.lon) / mag


def simplify(pts: list[Pt], eps: float) -> list[Pt]:
    """Douglas-Peucker polygon simplification."""
    if len(pts) <= 2:
        return pts
    dmax, idx = 0.0, 0
    end = len(pts) - 1
    for i in range(1, end):
        d = _perp(pts[i], pts[0], pts[end])
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        l = simplify(pts[: idx + 1], eps)
        r = simplify(pts[idx:], eps)
        return l[:-1] + r
    return [pts[0], pts[end]]


# ── Type extraction ───────────────────────────────────────────────────────────

def _type_name(val) -> str:
    if isinstance(val, dict):
        return str(val.get("name", val.get("shortName", ""))).upper()
    return str(val).upper()


def _type_id(val) -> int:
    if isinstance(val, dict):
        return int(val.get("value", val.get("id", 0)))
    try:
        return int(val)
    except (ValueError, TypeError):
        return -1


# ── Binary writers ────────────────────────────────────────────────────────────

def write_ctr_bin(path: str, ctrs: list[dict]):
    """
    CTR binary format (little-endian):
      [0-3]  magic "CTR\\x00"
      [4-5]  count uint16
      Per entry:
        [0]     name_len uint8  (max 31 chars)
        [1..n]  name ASCII
        [n+1]   type_id uint8  (4=CTR, 13=ATZ)
        [n+2..n+3]  point_count uint16
        Per point:
          lat_e6 int32  (lat × 1 000 000)
          lon_e6 int32  (lon × 1 000 000)
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"CTR\x00")
        f.write(struct.pack("<H", len(ctrs)))
        for ctr in ctrs:
            name = ctr["name"][:31].encode("ascii", errors="replace")
            f.write(struct.pack("B", len(name)))
            f.write(name)
            f.write(struct.pack("B", ctr.get("type_id", 4)))  # 4=CTR 13=ATZ
            pts = ctr["points"]
            f.write(struct.pack("<H", len(pts)))
            for pt in pts:
                f.write(struct.pack("<ii",
                    round(pt.lat * 1_000_000),
                    round(pt.lon * 1_000_000)))
    sz = os.path.getsize(path)
    total_pts = sum(len(c["points"]) for c in ctrs)
    print(f"  → {os.path.basename(path)}  {len(ctrs)} CTR  {total_pts} pts  {sz//1024}KB")


def write_ad_bin(path: str, ads: list[dict]):
    """
    Aerodromes binary format (little-endian):
      [0-3]  magic "ADEP"
      [4-5]  count uint16
      Per aerodrome (12 bytes fixed):
        [0-3]  icao[4]  null-padded ASCII
        [4-7]  lat_e6   int32
        [8-11] lon_e6   int32
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"ADEP")
        f.write(struct.pack("<H", len(ads)))
        for ad in ads:
            icao = ad["icao"].encode("ascii", errors="replace")[:4].ljust(4, b"\x00")
            f.write(icao)
            f.write(struct.pack("<ii",
                round(ad["lat"] * 1_000_000),
                round(ad["lon"] * 1_000_000)))
    sz = os.path.getsize(path)
    print(f"  → {os.path.basename(path)}  {len(ads)} aerodromes  {sz}B")


# ── Processing ────────────────────────────────────────────────────────────────

def process_country_ctrs(cc: str, api_key: str, out_dir: str, dump_dir: Optional[str]):
    print(f"\n[CTR] {cc}")
    # Try fetching with explicit type filter first, then filter client-side
    raw = fetch_all("airspaces", api_key, {"country": cc}, dump_dir)
    if not raw:
        print("  No airspaces returned.")
        return

    ctrs = []
    for item in raw:
        tid = _type_id(item.get("type", -1))
        if tid not in AIRSPACE_TYPE_IDS:
            continue
        geo = item.get("geometry", {})
        pts = outer_ring(geo)
        if len(pts) < 3:
            continue
        pts = simplify(pts, EPSILON_CTR)
        ctrs.append({
            "name":    item.get("name", "?"),
            "type_id": tid,
            "points":  pts,
        })

    if not ctrs:
        found = sorted({str(_type_id(i.get("type", "?"))) for i in raw})
        print(f"  WARNING: 0 CTR/ATZ found. Type IDs seen: {found}")
        return

    name = COUNTRY_NAMES.get(cc, cc.lower())
    write_ctr_bin(os.path.join(out_dir, f"{name}_ctr.bin"), ctrs)


def process_aerodromes(country_codes: list, api_key: str, out_dir: str, dump_dir: Optional[str]):
    print(f"\n[AERODROMES] {' '.join(country_codes)}")
    all_ads = []
    seen_icao = set()

    for cc in country_codes:
        print(f"  {cc}:")
        raw = fetch_all("airports", api_key, {"country": cc}, dump_dir)
        for item in raw:
            # Exclude heliports and hydrobases
            if _type_id(item.get("type", {})) in EXCLUDE_AP_TYPES:
                continue
            # Require standard 4-letter ICAO designator
            icao = (item.get("icaoCode") or item.get("icao") or "").strip().upper()
            if len(icao) != 4 or not icao.isalpha() or icao in seen_icao:
                continue
            # Name-based heliport safety net
            if any(w in (item.get("name","")).upper() for w in HELI_WORDS):
                continue
            # Extract coordinates from GeoJSON Point geometry
            geo  = item.get("geometry", {})
            coords = geo.get("coordinates", [])
            if not coords or len(coords) < 2:
                continue
            seen_icao.add(icao)
            all_ads.append({"icao": icao, "lat": float(coords[1]), "lon": float(coords[0])})
        time.sleep(RATE_S)

    write_ad_bin(os.path.join(out_dir, "aerodromes.bin"), all_ads)


# ── Verify ────────────────────────────────────────────────────────────────────

def verify_ctr_bin(path: str):
    """Quick sanity-read of a generated CTR bin."""
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"CTR\x00":
            print(f"  VERIFY FAIL: bad magic {magic!r}")
            return
        count = struct.unpack("<H", f.read(2))[0]
        for i in range(count):
            nlen   = struct.unpack("B", f.read(1))[0]
            name   = f.read(nlen).decode(errors="replace")
            tid    = struct.unpack("B", f.read(1))[0]
            npts   = struct.unpack("<H", f.read(2))[0]
            f.read(npts * 8)
            if i == 0:
                tname = "CTR" if tid == 4 else "ATZ" if tid == 13 else f"t{tid}"
                print(f"  VERIFY OK: {count} entries, first='{name}' [{tname}] {npts}pts")
            if i >= 2:
                break


def verify_ad_bin(path: str):
    """Quick sanity-read of generated aerodromes bin."""
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"ADEP":
            print(f"  VERIFY FAIL: bad magic {magic!r}")
            return
        count = struct.unpack("<H", f.read(2))[0]
        icao  = f.read(4).decode(errors="replace").rstrip("\x00")
        lat_e6, lon_e6 = struct.unpack("<ii", f.read(8))
        print(f"  VERIFY OK: {count} aerodromes, first={icao} "
              f"lat={lat_e6/1e6:.4f} lon={lon_e6/1e6:.4f}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Download OpenAIP data and convert to AT-VIEW binary files for SD upload."
    )
    ap.add_argument("--key",       required=True,             help="OpenAIP API key (free at openaip.net)")
    ap.add_argument("--countries", nargs="+", default=["FR", "BE"],
                                                               help="ISO-2 country codes (default: FR BE)")
    ap.add_argument("--out",       default="public/aip",      help="Output directory")
    ap.add_argument("--dump-json", action="store_true",        help="Save raw API JSON for debugging")
    args = ap.parse_args()

    root    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(root, args.out)
    os.makedirs(out_dir, exist_ok=True)

    dump_dir = None
    if args.dump_json:
        dump_dir = os.path.join(root, "public", "aip_debug")
        os.makedirs(dump_dir, exist_ok=True)
        print(f"[DEBUG] JSON dumps → {dump_dir}/")

    for cc in args.countries:
        process_country_ctrs(cc, args.key, out_dir, dump_dir)
        time.sleep(RATE_S)

    process_aerodromes(args.countries, args.key, out_dir, dump_dir)

    # Verify output
    print("\n[VERIFY]")
    for fname in sorted(os.listdir(out_dir)):
        path = os.path.join(out_dir, fname)
        if fname.endswith("_ctr.bin"):
            verify_ctr_bin(path)
        elif fname == "aerodromes.bin":
            verify_ad_bin(path)

    print(f"\n✓ Done — files in {out_dir}/")
    print("  Upload via: http://atview.local  (WiFi AP must be ON in Settings 2/2)")


if __name__ == "__main__":
    main()
