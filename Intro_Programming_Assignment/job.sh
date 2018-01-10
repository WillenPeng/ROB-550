#!/bin/bash
NUM=10

python testGen.py $NUM $NUM > A.csv
python testGen.py $NUM $NUM > B.csv
./matmult $NUM $NUM $NUM $NUM
python matmult_pure.py $NUM $NUM $NUM $NUM
python matmult_npy.py $NUM $NUM $NUM $NUM

