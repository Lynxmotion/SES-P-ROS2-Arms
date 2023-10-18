#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `pro_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/pro_arm.urdf.xacro"

# Default parameters
SIZE=550
DOF=6

# Parse arguments
while getopts "d:s:" arg; do
  case ${arg} in
    d) case ${OPTARG} in
          5|6) DOF="${OPTARG}" ;;
          *) echo "Invalid DOF: ${OPTARG}. DOF can only be 5 or 6." >&2
             exit 1 ;;
        esac ;;
    s) case ${OPTARG} in
          550|900) SIZE="${OPTARG}" ;;
          *) echo "Invalid Size: ${OPTARG}. Size can only be 550 or 900." >&2
             exit 1 ;;
        esac ;;
    *) echo "Invalid option: -${OPTARG}" >&2
       exit 1;;
  esac
done

URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/pro_arm_${DOF}dof_${SIZE}.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=pro_arm_"${DOF}"dof_"${SIZE}"
    dof:="${DOF}"
    size:="${SIZE}"
    collision:=true
    ros2_control:=true
    ros2_control_plugin:=sim
    gazebo_preserve_fixed_joint:=false
)

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null

# Process xacro into URDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${URDF_PATH}" &&
echo "Created new ${URDF_PATH}"
