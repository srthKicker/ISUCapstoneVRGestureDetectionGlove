combine_sources() {
    output_file="$1"

    if [ -z "$output_file" ]; then
        echo "Usage: combine_sources <output-file>"
        return 1
    fi

    # Truncate or create the output file
    > "$output_file"

    # Expand only if files exist
    shopt -s nullglob 2>/dev/null || :
    files=( *.c *.h )

    if [ ${#files[@]} -eq 0 ]; then
        echo "No .c or .h files found in $(pwd)"
        return 1
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
}
