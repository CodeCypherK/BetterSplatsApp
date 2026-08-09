#!/usr/bin/env bash
# Packages a built .app into an unsigned .ipa (Payload zip). The IPA is
# signed later on the user's machine by Sideloadly/AltStore — see
# docs/SIDELOADING.md.
#   usage: package_ipa.sh <path/to/BetterSplats.app> <out.ipa>
set -euo pipefail

APP="${1:?usage: package_ipa.sh <app-bundle> <out-ipa>}"
OUT="${2:?usage: package_ipa.sh <app-bundle> <out-ipa>}"

if [[ ! -d "$APP" ]]; then
  echo "ERROR: app bundle not found: $APP"
  exit 1
fi

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE/Payload"
cp -R "$APP" "$STAGE/Payload/"
(cd "$STAGE" && zip -qry out.ipa Payload)

mkdir -p "$(dirname "$OUT")"
mv "$STAGE/out.ipa" "$OUT"
echo "IPA: $OUT ($(du -h "$OUT" | cut -f1))"
