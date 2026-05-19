#!/bin/bash
#SBATCH --job-name=carmaker_unet_multi_gpu
#SBATCH --partition=partition-3090-intel
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gres=gpu:2
#SBATCH --cpus-per-task=8
#SBATCH --time=00:30:00
#SBATCH --output=/home/ailab/AILabSSD/99_Management/slurm-logs/%x_%j.out
#SBATCH --error=/home/ailab/AILabSSD/99_Management/slurm-logs/%x_%j.err
#SBATCH --comment=submitter:hwansoo

set -euo pipefail

# Prevent container Python from loading host's user site-packages (~/.local)
export PYTHONNOUSERSITE=1
export APPTAINERENV_PYTHONNOUSERSITE=1

# Path Configurations
SIF_IMAGE="/home/ailab/AILabSSD/01_SlurmWorkspace/2026-intern-unet-multi.sif"
HOST_WORKSPACE="/home/ailab/AILabSSD/04_Shared_Repository/2026-intern/carmaker"
HOST_REAL_DATA_ROOT=$(readlink -f /home/ailab/AILabSSD/04_Shared_Repository/2026-intern/dataset)
HOST_RUN_ROOT="/home/ailab/AILabSSD/99_Management/tb_runs/2026-intern/"

# Internal Container Paths
CONTAINER_DATA_ROOT="/workspace/src/carmaker_image/data"

[ -f "${SIF_IMAGE}" ] || { echo "SIF not found: ${SIF_IMAGE}"; exit 1; }

echo "Starting Apptainer Training Container..."
echo " - SIF: ${SIF_IMAGE}"
echo " - Data: ${HOST_REAL_DATA_ROOT} -> ${CONTAINER_DATA_ROOT}"
echo " - Runs: ${HOST_RUN_ROOT} -> /runs"

# Execution
apptainer exec --nv \
    --bind ${HOST_WORKSPACE}:/workspace \
    --bind ${HOST_REAL_DATA_ROOT}:${CONTAINER_DATA_ROOT}:ro \
    --bind ${HOST_RUN_ROOT}:/runs \
    ${SIF_IMAGE} \
    torchrun --standalone --nproc_per_node=${SLURM_GPUS_ON_NODE:-2} \
        /workspace/src/segmentation/train.py \
        --config /workspace/src/segmentation/config/segmentation_unet.yaml \
        --data-root ${CONTAINER_DATA_ROOT} \
        --manifest ${CONTAINER_DATA_ROOT}/csv/manifest.csv \
        --run-base /runs
