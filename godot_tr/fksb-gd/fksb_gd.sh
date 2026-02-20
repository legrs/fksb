#!/bin/sh
printf '\033c\033]0;%s\a' fksb_gd
base_path="$(dirname "$(realpath "$0")")"
"$base_path/fksb_gd.x86_64" "$@"
