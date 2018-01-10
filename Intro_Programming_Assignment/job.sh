#!/bin/bash
NUM=1000

python testGen.py $NUM $NUM > A.txt
python testGen.py $NUM $NUM > B.txt
# ./matmult $NUM $NUM $NUM $NUM
# python matmult_pure.py $NUM $NUM $NUM $NUM
python matmult_npy.py $NUM $NUM $NUM $NUM

