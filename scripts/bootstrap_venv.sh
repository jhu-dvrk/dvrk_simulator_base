#!/usr/bin/env bash
set -euo pipefail

# Generic virtualenv bootstrap for dVRK workspace packages.
# Usage: bootstrap_venv.sh <venv_name> <requirements_file> <display_name> [test_command] [flags...]
# Example: bootstrap_venv.sh .venv-newton requirements.txt "NVIDIA Newton" "-c 'import newton'" -y

VENV_NAME="${1:-.venv}"
REQUIREMENTS_FILE="${2:-requirements.txt}"
DISPLAY_NAME="${3:-dVRK}"

TEST_COMMAND=""
if [[ $# -ge 4 ]]; then
    if [[ "${4}" == "-y" || "${4}" == "--yes" ]]; then
        shift 3
    else
        TEST_COMMAND="${4}"
        shift 4
    fi
else
    shift $(( $# >= 3 ? 3 : $# ))
fi

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

find_workspace_root() {
    local cursor="${SCRIPT_PATH}"
    while [[ "${cursor}" != "/" ]]; do
        case "$(basename "${cursor}")" in
            src|install)
                dirname "${cursor}"
                return 0
                ;;
        esac
        cursor="$(dirname "${cursor}")"
    done
    return 1
}

WORKSPACE_ROOT="$(find_workspace_root || true)"
if [[ -z "${WORKSPACE_ROOT}" || ! -d "${WORKSPACE_ROOT}" ]]; then
    echo "error: could not determine workspace root from ${SCRIPT_PATH}" >&2
    exit 2
fi
WORKSPACE_ROOT="$(readlink -f "${WORKSPACE_ROOT}")"

VENV_DIR="${WORKSPACE_ROOT}/${VENV_NAME}"
if [[ ! -f "${REQUIREMENTS_FILE}" ]]; then
    echo "error: requirements file not found: ${REQUIREMENTS_FILE}" >&2
    exit 2
fi
REQUIREMENTS_FILE="$(readlink -f "${REQUIREMENTS_FILE}")"
PYTHON="${PYTHON:-python3}"

echo "Bootstrapping ${DISPLAY_NAME} virtual environment at: ${VENV_DIR}"
echo "Requirements file: ${REQUIREMENTS_FILE}"

PROCEED=false
for arg in "$@"; do
    case "${arg}" in
        -y|--yes)
            PROCEED=true
            ;;
    esac
done

if [[ "${PROCEED}" = false ]]; then
    if [[ -t 0 ]]; then
        read -r -p "Create venv and install using pip? [y/N] " response
        case "${response}" in
            [yY][eE][sS]|[yY])
                ;;
            *)
                echo "Operation cancelled."
                exit 0
                ;;
        esac
    fi
fi

if [[ -e "${VENV_DIR}" && ! -f "${VENV_DIR}/pyvenv.cfg" ]]; then
    echo "error: refusing to use an existing non-venv path: ${VENV_DIR}" >&2
    exit 2
fi

if [[ ! -f "${VENV_DIR}/pyvenv.cfg" ]]; then
    echo "Creating virtual environment at ${VENV_DIR} with system site packages..."
    "${PYTHON}" -m venv --system-site-packages "${VENV_DIR}"
else
    echo "Reusing existing virtual environment at ${VENV_DIR}."
fi

VENV_PYTHON="${VENV_DIR}/bin/python"
if [[ ! -x "${VENV_PYTHON}" ]]; then
    VENV_PYTHON="${VENV_DIR}/bin/python3"
fi
if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "error: venv Python is missing: ${VENV_PYTHON}" >&2
    exit 2
fi

echo "Installing dependencies from ${REQUIREMENTS_FILE}..."
"${VENV_PYTHON}" -m pip install --upgrade pip
"${VENV_PYTHON}" -m pip install -r "${REQUIREMENTS_FILE}"

echo
echo "${DISPLAY_NAME} environment bootstrapped successfully."
if [[ -n "${TEST_COMMAND}" ]]; then
    echo "Test with: ${VENV_PYTHON} ${TEST_COMMAND}"
fi
