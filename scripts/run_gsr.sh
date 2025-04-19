#!/usr/bin/env bash
set -euo pipefail

SRC_ROOT="/data"
DEST_ROOT="/outputs"

for split in valid test challenge; do
  for sngs in "$SRC_ROOT"/$split/SNGS-*; do
    [ -d "$sngs" ] || continue

    # Find the first-level subdirectory (where your .jpg/.png live)
    img_folder=$(find "$sngs" -mindepth 1 -maxdepth 1 -type d | head -n1)
    if [ -z "$img_folder" ]; then
      echo "⚠️  No image subdir in $sngs, skipping." >&2
      continue
    fi

    # Mirror into outputs/<split>/SNGS-XXX.json
    out_dir="$DEST_ROOT/$split"
    mkdir -p "$out_dir"
    out_json="$out_dir/$(basename "$sngs").json"

    # Run broadtrack on that inner folder
    broadtrack --f "$img_folder" --o "$out_json"
    echo "✔ Processed $img_folder → $out_json"
  done
done

echo "All done – JSON files are in $DEST_ROOT/{valid,test,challenge}/SNGS-*.json"
