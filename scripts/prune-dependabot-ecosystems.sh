#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
#
# Keep only explicitly selected package ecosystems in dependabot.yml.
# The invalid `nix` ecosystem is never retained. Refuses to produce an empty
# updates list. Idempotent.
set -euo pipefail

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <dependabot.yml> <ecosystem>..." >&2
    exit 2
fi

path=$1
shift

if [ ! -f "$path" ]; then
    echo "  dependabot: $path absent, nothing to prune"
    exit 0
fi

keep_csv=""
for ecosystem in "$@"; do
    [ "$ecosystem" = "nix" ] && continue
    if [ -z "$keep_csv" ]; then
        keep_csv=$ecosystem
    else
        keep_csv="$keep_csv,$ecosystem"
    fi
done

output=$(mktemp)
trap 'rm -f "$output"' EXIT

set +e
summary=$(awk -v keep_csv="$keep_csv" -v output="$output" '
    BEGIN {
        count = split(keep_csv, names, ",")
        for (i = 1; i <= count; i++) {
            if (names[i] != "" && names[i] != "nix") keep[names[i]] = 1
        }
    }

    function flush_entry() {
        if (entry == "") return
        if (name in keep) {
            body = body entry
            kept = kept (kept == "" ? "" : ", ") name
            kept_count++
        } else {
            dropped = dropped (dropped == "" ? "" : ", ") name
        }
        entry = ""
        name = ""
    }

    /^[[:space:]]*-[[:space:]]*package-ecosystem:/ {
        flush_entry()
        entry = $0 ORS
        name = $0
        sub(/^[^:]*:[[:space:]]*/, "", name)
        sub(/[[:space:]]*#.*/, "", name)
        gsub(/["\047\r[:space:]]/, "", name)
        next
    }

    {
        if (entry != "") entry = entry $0 ORS
        else head = head $0 ORS
    }

    END {
        flush_entry()
        if (kept_count == 0 && kept == "" && dropped == "") exit 2
        if (kept_count == 0) exit 3
        printf "%s%s", head, body > output
        if (dropped == "") print "NOTHING"
        else print "KEPT=" kept "\nDROPPED=" dropped
    }
' "$path")
status=$?
set -e

case "$status" in
    0) ;;
    2)
        echo "  dependabot: no ecosystem entries found"
        exit 0
        ;;
    3)
        echo "  dependabot: refusing to prune every entry; left unchanged"
        exit 0
        ;;
    *)
        echo "  dependabot: parser failed" >&2
        exit "$status"
        ;;
esac

if [ "$summary" = "NOTHING" ]; then
    echo "  dependabot: nothing to prune"
    exit 0
fi

mv "$output" "$path"
trap - EXIT
kept=${summary%%$'\n'*}
dropped=${summary#*$'\n'}
echo "  dependabot: kept ${kept#KEPT=} / dropped ${dropped#DROPPED=}"
