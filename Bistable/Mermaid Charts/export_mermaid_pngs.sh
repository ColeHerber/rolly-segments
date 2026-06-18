#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_file="$script_dir/mermaid-bw-config.json"
css_file="$script_dir/mermaid-bw.css"
scale="${MERMAID_SCALE:-5}"

if ! command -v mmdc >/dev/null 2>&1; then
  echo "Error: mmdc is not installed or is not on PATH." >&2
  exit 1
fi

if [[ -z "${PUPPETEER_EXECUTABLE_PATH:-}" ]]; then
  chrome="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
  if [[ -x "$chrome" ]]; then
    export PUPPETEER_EXECUTABLE_PATH="$chrome"
  fi
fi

while IFS= read -r -d '' input; do
  output="${input%.mmd}.png"

  echo "Rendering $(basename "$input") -> $(basename "$output")"
  mmdc \
    -c "$config_file" \
    -C "$css_file" \
    -b white \
    -s "$scale" \
    -i "$input" \
    -o "$output"
done < <(find "$script_dir" -type f -name '*.mmd' -print0 | sort -z)

echo "Done."
