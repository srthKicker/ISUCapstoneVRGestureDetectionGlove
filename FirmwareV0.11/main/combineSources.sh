#!/usr/bin/env bash

# combine_sources.sh
# Concatenate all .c and .h files into one output file.

output_file="${1:-allCode.txt}"

# Truncate or create the output file
> "$output_file"

# Expand only if files exist
shopt -s nullglob
files=( *.c *.h )
shopt -u nullglob

if [ ${#files[@]} -eq 0 ]; then
    echo "No .c or .h files found in $(pwd)"
    exit 1
fi

for file in "${files[@]}"; do
    echo "Adding $file"
    {
        echo "// $file"
        cat "$file"
        echo
    } >> "$output_file"
done

echo "Done. Wrote ${#files[@]} files into $output_file"

