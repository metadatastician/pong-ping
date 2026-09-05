#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
#
# Delete the RSR template's HTML "TEMPLATE INSTRUCTIONS" comment blocks.
# Run after placeholder substitution. The matcher cannot cross a comment
# terminator, so an earlier SPDX comment is preserved. Idempotent.
set -euo pipefail

root="${1:-.}"
marker='TEMPLATE INSTRUCTIONS'
changed=0

while IFS= read -r -d '' path; do
    case "$path" in
        */.git/*|*/node_modules/*|*/.venv/*|*/target/*|*/dist/*) continue ;;
    esac

    [ -L "$path" ] && continue
    grep -Iq . "$path" || continue
    grep -Fq "$marker" "$path" || continue

    before=$(sha256sum "$path" | cut -d' ' -f1)
    perl -0pi -e \
        '$removed = s/<!--(?:(?!-->).)*?TEMPLATE INSTRUCTIONS(?:(?!-->).)*?-->[ \t]*\n?//sg; s/\n{3,}/\n\n/g if $removed' \
        "$path"
    after=$(sha256sum "$path" | cut -d' ' -f1)

    if [ "$before" != "$after" ]; then
        echo "  instruction block: stripped from $path"
        changed=$((changed + 1))
    fi
done < <(find "$root" -type f -print0 | LC_ALL=C sort -z)

if [ "$changed" -eq 0 ]; then
    echo "  instruction blocks: none found"
else
    echo "  instruction blocks: $changed file(s) cleaned"
fi
