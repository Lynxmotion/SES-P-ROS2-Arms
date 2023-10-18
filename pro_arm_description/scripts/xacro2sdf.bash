#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `pro_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/pro_arm.urdf.xacro"
TMP_URDF_PATH="/tmp/pro_arm_tmp.urdf"
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


SDF_PATH="$(dirname "${SCRIPT_DIR}")/models/pro_arm_${DOF}dof_${SIZE}/model.sdf"

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

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
ign sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/pro_arm_description\///g" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null