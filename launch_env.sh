#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

#export SPI_ERR_PROB=0.001

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="10.1"
fi

export STAGING_ROOT="/data/safe_staging"
