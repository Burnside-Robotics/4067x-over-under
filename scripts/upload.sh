NAME="${1}"
BIN="${1}.bin"
arm-none-eabi-objcopy -O binary -R .hot_init "${1}" "${BIN}"

IFS='\'

read -ra strarr <<< "$NAME"

REPL="${BIN//\\/\\\\}"

cat > project.pros << EOL
{
    "py/object": "pros.conductor.project.Project",
    "py/state": {
        "project_name": "${strarr[${#ArrayName[@]} - 1]}",
        "target": "v5",
        "templates": {
            "kernel": {
                "location": "",
                "metadata": {
                    "origin": "pros-mainline",
                    "output": "${REPL}"
                },
                "name": "kernel",
                "py/object": "pros.conductor.templates.local_template.LocalTemplate",
                "supported_kernels": null,
                "system_files": [],
                "target": "v5",
                "user_files": [],
                "version": "3.2.0"
            }
        },
        "upload_options": {}
    }
}
EOL

pros ut
