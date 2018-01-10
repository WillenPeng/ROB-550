I run matmult.c, matmult_pure.py and matmult_npy.py 20 times for three different sizes of input matrix, and I took the minimum operation time for each condition and summarized them as below:

									matmult.c	matmult_pure.py		matmult_npy.py
A and B are 10x10 matrices			3 us  		104 us 				103 us
A and B are 100x100 matrices		3607 us  	77922 us 			2065 us
A and B are 1000x1000 matrices		6051256 us	124027181 us		174264 us

From the benchmark table, we can find matmult.c works well in matrix multiplicaiton with small size and matmult_npy.py works well with large matrix size. We can conclude that numpy function has great advantage in speed and efficency when doing some computation, such as matrix multiplication.